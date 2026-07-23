//============================================================================
// smc_x16.sv  -  Commander X16 System Management Controller (SMC) emulation.
//
// Implements the I2C command set documented in x16-smc-main/README.md as a
// slave at address $42 on the bit-banged I2C bus that the X16 KERNAL drives
// via VIA1 PA[0]=SDA, PA[1]=SCL.  Replaces the placeholder smc_i2c_slave.sv
// which returned 0 for everything.
//
// Sources of keyboard / mouse / joystick events:
//   - UART (FT232) byte stream from the host PC via tools/uart_rx.sv +
//     tools/send_input.py.
//   - Stream protocol (matches Scripts/send_input.py):
//       0xFF S DX DY W    -> 4-byte mouse packet (5 bytes total; W = wheel
//                            delta, KERNAL nibble format, upper nibble 0)
//       0xFE STATE        -> joystick A (1 byte payload)
//       0xFD STATE        -> joystick B (1 byte payload)
//       0xE0 SC           -> extended PS/2 scancode (key press)
//       0xE0 0xF0 SC      -> extended PS/2 scancode (key release)
//       0xF0 SC           -> regular PS/2 scancode (key release)
//       anything else     -> regular PS/2 scancode (key press)
//
// PS/2 Set-2 -> IBM System/2 keycode translation matches the table in
// x16-smc-main/ps2.h (PS2_REG_SCANCODES + ps2ext_to_keycode).  Bit 7 of the
// emitted keycode is the release flag (1=release, 0=press).
//
// Implemented I2C commands (per x16-smc-main/README.md):
//   $01  W  Power Off                     -> latches power_off_req
//   $02  W  Reset                         -> latches reset_req
//   $03  W  NMI                           -> latches nmi_req
//   $05  W  Set Activity LED level        -> latches act_led_r
//   $07  R  Get keyboard keycode          -> pops keyboard FIFO, 0 if empty
//   $08  W  Echo                          -> stores echo_byte
//   $08  R  Echo                          -> returns echo_byte (extension)
//   $09  W  Debug output                  -> ignored
//   $09  R  Long-press flag               -> returns 0
//   $0a  R  Get low fuse                  -> $F1
//   $0b  R  Get lock bits                 -> $FF
//   $0c  R  Get extended fuse             -> $FE
//   $0d  R  Get high fuse                 -> $D4
//   $18  R  Get keyboard command status   -> $FA (idle/last-cmd-OK)
//   $19  W  Send keyboard command (1)     -> accepted, no PS/2 sent
//   $1a  W  Send keyboard command (2)     -> accepted, no PS/2 sent
//   $1b  R  Keyboard ready                -> $01 (ready)
//   $20  W  Set requested mouse device ID -> latches mouse_req_id
//   $21  R  Mouse movement                -> 4-byte packet or 0 if empty
//   $22  R  Mouse device ID               -> $03 (Intellimouse: wheel,
//                                            4-byte packets -- the KERNAL
//                                            reads any nonzero ID as such)
//   $30/31/32 R Firmware version          -> 48.1.0 (arbitrary, non-zero)
//   $40  W  Set default read op           -> latches default_request
//   $41  R  Get keycode fast              -> like $07, NACK if empty
//   $42  R  Get mouse fast                -> like $21, NACK if empty
//   $43  R  Get PS/2 data fast            -> kbd byte + mouse packet
//   $8e  W  Get bootloader version (stub) -> stored, returns $FF
//   $8f  W  Start bootloader              -> ignored (no-op)
//   $90  W  Set flash page                -> stored
//   $91  R  Read flash                    -> $FF (no flash backing store)
//   $92  W  Write flash                   -> ignored
//   $93  R  Get flash write mode          -> $00 (disabled)
//   $93  W  Request flash write mode      -> ignored
//
// "Master read without command byte" (read-default behavior) is supported:
// if a read starts without a pending command, the slave behaves as if
// `default_request` (initially $41 = GET_KEYCODE_FAST) was the command.
//
// A command written WITHOUT data survives the STOP: the KERNAL's
// i2c_read_byte / i2c_read_first_byte issue  write cmd, STOP, START,
// addr+R  (a full STOP, not a repeated START), and the real firmware's
// I2C_Receive early-returns for 1-byte writes, keeping the command armed
// for the read.  The pending command is consumed (re-armed to
// default_request) by serving a read or by a write that carried data.
//============================================================================

module smc_x16 #(
    parameter logic [6:0] SLAVE_ADDR = 7'h42
) (
    input  logic clk,                    // cpu_clk
    input  logic reset_n,

    // X16-side I2C bus -- this is the INTERNAL logical bus (idle = 1).
    // The top wires it from a combinational pull-up:
    //   wire bus_sda = ~(via_sda_drv_low | smc_sda_drv_low);
    //   wire bus_scl = ~via_scl_drv_low;
    // We don't touch the external I2C_SCL / I2C_SDA pins -- those are
    // used by adv7513_init and wm8731_init for the real codecs only.
    input  logic sda_bus,
    input  logic scl_bus,
    output logic sda_drive_low,          // 1 -> slave pulls SDA low

    // UART byte stream from host (one byte_valid pulse per byte).
    input  logic [7:0] uart_byte,
    input  logic       uart_byte_valid,

    // Latched control outputs (consumers in C5G_x16 wire these into the
    // CPU reset / NMI / activity LED paths).
    output logic       power_off_req,
    output logic       reset_req,      // I2C cmd $02 OR Ctrl+Alt+Del
    output logic       nmi_req,        // I2C cmd $03 OR Ctrl+Alt+PrtScr
    output logic [7:0] act_led_r,

    // Debug: keyboard FIFO occupancy (0..16) so the LEDR can show it.
    output logic [4:0] dbg_kbd_count,

    // Debug: sticky bits exposing I2C activity for board-LED debug.
    //   dbg_saw_start       any START condition observed
    //   dbg_saw_addr_match  START followed by a valid $42 + R/W byte
    //   dbg_saw_byte        slave RX'd at least one post-address byte
    //   dbg_saw_repeat      saw a repeated START while addressed
    //   dbg_saw_stop        saw STOP after address match
    //   dbg_saw_tx          slave drove at least one byte to master (read)
    output logic        dbg_saw_start,
    output logic        dbg_saw_addr_match,
    output logic        dbg_saw_byte,
    output logic        dbg_saw_repeat,
    output logic        dbg_saw_stop,
    output logic        dbg_saw_tx,
    output logic [7:0]  dbg_last_cmd,

    // Last 8-bit byte sampled in the S_ADDR state (i.e. the address byte
    // the master drove, regardless of whether it matched $42).  Tells us
    // what the X16 KERNAL is actually putting on the bus.  Updated on
    // every completed address phase.
    output logic [7:0]  dbg_last_addr_byte,

    // Phase-e keyboard bring-up taps:
    output logic        dbg_kbd_pop,    // 1-cycle when a keyboard byte is delivered
    output logic [7:0]  dbg_tx_byte     // byte the SMC is serving on a read
);

    // sda_drive_low is a module output; the top resolves it into the
    // internal bus.  No direct pin drive here.

    // ====================================================================
    // I2C command opcodes (mirrors x16-smc-main/x16-smc.ino)
    // ====================================================================
    localparam logic [7:0] CMD_POW_OFF              = 8'h01;
    localparam logic [7:0] CMD_RESET                = 8'h02;
    localparam logic [7:0] CMD_NMI                  = 8'h03;
    localparam logic [7:0] CMD_SET_ACT_LED          = 8'h05;
    localparam logic [7:0] CMD_GET_KEYCODE          = 8'h07;
    localparam logic [7:0] CMD_ECHO                 = 8'h08;
    localparam logic [7:0] CMD_DBG_OUT              = 8'h09;
    localparam logic [7:0] CMD_GET_FUSE_LOW         = 8'h0a;
    localparam logic [7:0] CMD_GET_FUSE_LOCK        = 8'h0b;
    localparam logic [7:0] CMD_GET_FUSE_EXT         = 8'h0c;
    localparam logic [7:0] CMD_GET_FUSE_HIGH        = 8'h0d;
    localparam logic [7:0] CMD_GET_KBD_STATUS       = 8'h18;
    localparam logic [7:0] CMD_KBD_CMD1             = 8'h19;
    localparam logic [7:0] CMD_KBD_CMD2             = 8'h1a;
    localparam logic [7:0] CMD_KBD_INIT_STATE       = 8'h1b;
    localparam logic [7:0] CMD_SET_MOUSE_ID         = 8'h20;
    localparam logic [7:0] CMD_GET_MOUSE_MOV        = 8'h21;
    localparam logic [7:0] CMD_GET_MOUSE_ID         = 8'h22;
    localparam logic [7:0] CMD_GET_VER1             = 8'h30;
    localparam logic [7:0] CMD_GET_VER2             = 8'h31;
    localparam logic [7:0] CMD_GET_VER3             = 8'h32;
    localparam logic [7:0] CMD_SET_DFLT_READ_OP     = 8'h40;
    localparam logic [7:0] CMD_GET_KEYCODE_FAST     = 8'h41;
    localparam logic [7:0] CMD_GET_MOUSE_MOV_FAST   = 8'h42;
    localparam logic [7:0] CMD_GET_PS2DATA_FAST     = 8'h43;
    localparam logic [7:0] CMD_GET_BOOTLDR_VER      = 8'h8e;
    localparam logic [7:0] CMD_BOOTLDR_START        = 8'h8f;
    localparam logic [7:0] CMD_SET_FLASH_PAGE       = 8'h90;
    localparam logic [7:0] CMD_READ_FLASH           = 8'h91;
    localparam logic [7:0] CMD_WRITE_FLASH          = 8'h92;
    localparam logic [7:0] CMD_FLASH_WRITE_MODE     = 8'h93;

    // ====================================================================
    // UART decoder: pushes IBM keycodes into the keyboard FIFO, latest
    // mouse packet into mouse_pkt, and latest joystick state into joy_*.
    // ====================================================================

    // Decoder phase.  IDLE waits for the first byte of a new packet.
    typedef enum logic [3:0] {
        U_IDLE,
        U_EXT_KEY,           // saw 0xE0, awaiting scan code (or 0xF0)
        U_EXT_BREAK_KEY,     // saw 0xE0 0xF0, awaiting scan code (release)
        U_BREAK_KEY,         // saw 0xF0, awaiting regular release scan code
        U_MOUSE_S,           // saw 0xFF, awaiting status byte
        U_MOUSE_DX,          // saw 0xFF S, awaiting DX byte
        U_MOUSE_DY,          // saw 0xFF S DX, awaiting DY byte
        U_MOUSE_W,           // saw 0xFF S DX DY, awaiting wheel (last of pkt)
        U_JOY_BYTE           // saw 0xFE or 0xFD, awaiting state byte
    } u_state_t;
    u_state_t u_state;

    logic        u_joy_is_b;   // which stick is the pending JOY_BYTE for
    logic [7:0]  u_mouse_s;
    logic [7:0]  u_mouse_dx;
    logic [7:0]  u_mouse_dy;

    // PS/2 -> IBM keycode translation tables.
    // Regular (non-extended) scan codes: index 0..130 -> table[scancode-1].
    // Matches PS2_REG_SCANCODES[] in x16-smc-main/ps2.h:365-383.
    function automatic logic [7:0] ps2_to_ibm(input logic [7:0] sc);
        case (sc)
            8'h01: ps2_to_ibm = 8'd120;   // F9
            8'h03: ps2_to_ibm = 8'd116;   // F5
            8'h04: ps2_to_ibm = 8'd114;   // F3
            8'h05: ps2_to_ibm = 8'd112;   // F1
            8'h06: ps2_to_ibm = 8'd113;   // F2
            8'h07: ps2_to_ibm = 8'd123;   // F12
            8'h09: ps2_to_ibm = 8'd121;   // F10
            8'h0a: ps2_to_ibm = 8'd119;   // F8
            8'h0b: ps2_to_ibm = 8'd117;   // F6
            8'h0c: ps2_to_ibm = 8'd115;   // F4
            8'h0d: ps2_to_ibm = 8'd16;    // Tab
            8'h0e: ps2_to_ibm = 8'd1;     // ` ~
            8'h11: ps2_to_ibm = 8'd60;    // Left Alt
            8'h12: ps2_to_ibm = 8'd44;    // Left Shift
            8'h14: ps2_to_ibm = 8'd58;    // Left Ctrl
            8'h15: ps2_to_ibm = 8'd17;    // q
            8'h16: ps2_to_ibm = 8'd2;     // 1
            8'h1a: ps2_to_ibm = 8'd46;    // z
            8'h1b: ps2_to_ibm = 8'd32;    // s
            8'h1c: ps2_to_ibm = 8'd31;    // a
            8'h1d: ps2_to_ibm = 8'd18;    // w
            8'h1e: ps2_to_ibm = 8'd3;     // 2
            8'h21: ps2_to_ibm = 8'd48;    // c
            8'h22: ps2_to_ibm = 8'd47;    // x
            8'h23: ps2_to_ibm = 8'd33;    // d
            8'h24: ps2_to_ibm = 8'd19;    // e
            8'h25: ps2_to_ibm = 8'd5;     // 4
            8'h26: ps2_to_ibm = 8'd4;     // 3
            8'h29: ps2_to_ibm = 8'd61;    // Space
            8'h2a: ps2_to_ibm = 8'd49;    // v
            8'h2b: ps2_to_ibm = 8'd34;    // f
            8'h2c: ps2_to_ibm = 8'd21;    // t
            8'h2d: ps2_to_ibm = 8'd20;    // r
            8'h2e: ps2_to_ibm = 8'd6;     // 5
            8'h31: ps2_to_ibm = 8'd51;    // n
            8'h32: ps2_to_ibm = 8'd50;    // b
            8'h33: ps2_to_ibm = 8'd36;    // h
            8'h34: ps2_to_ibm = 8'd35;    // g
            8'h35: ps2_to_ibm = 8'd22;    // y
            8'h36: ps2_to_ibm = 8'd7;     // 6
            8'h3a: ps2_to_ibm = 8'd52;    // m
            8'h3b: ps2_to_ibm = 8'd37;    // j
            8'h3c: ps2_to_ibm = 8'd23;    // u
            8'h3d: ps2_to_ibm = 8'd8;     // 7
            8'h3e: ps2_to_ibm = 8'd9;     // 8
            8'h41: ps2_to_ibm = 8'd53;    // ,
            8'h42: ps2_to_ibm = 8'd38;    // k
            8'h43: ps2_to_ibm = 8'd24;    // i
            8'h44: ps2_to_ibm = 8'd25;    // o
            8'h45: ps2_to_ibm = 8'd11;    // 0
            8'h46: ps2_to_ibm = 8'd10;    // 9
            8'h49: ps2_to_ibm = 8'd54;    // .
            8'h4a: ps2_to_ibm = 8'd55;    // /
            8'h4b: ps2_to_ibm = 8'd39;    // l
            8'h4c: ps2_to_ibm = 8'd40;    // ;
            8'h4d: ps2_to_ibm = 8'd26;    // p
            8'h4e: ps2_to_ibm = 8'd12;    // -
            8'h52: ps2_to_ibm = 8'd41;    // '
            8'h54: ps2_to_ibm = 8'd27;    // [
            8'h55: ps2_to_ibm = 8'd13;    // =
            8'h58: ps2_to_ibm = 8'd30;    // Caps Lock
            8'h59: ps2_to_ibm = 8'd57;    // Right Shift
            8'h5a: ps2_to_ibm = 8'd43;    // Enter
            8'h5b: ps2_to_ibm = 8'd28;    // ]
            8'h5d: ps2_to_ibm = 8'd29;    // backslash
            8'h66: ps2_to_ibm = 8'd15;    // Backspace
            8'h69: ps2_to_ibm = 8'd93;    // KP 1 (also End in ext)
            8'h6b: ps2_to_ibm = 8'd92;    // KP 4
            8'h6c: ps2_to_ibm = 8'd91;    // KP 7
            8'h70: ps2_to_ibm = 8'd99;    // KP 0
            8'h71: ps2_to_ibm = 8'd104;   // KP .
            8'h72: ps2_to_ibm = 8'd98;    // KP 2
            8'h73: ps2_to_ibm = 8'd97;    // KP 5
            8'h74: ps2_to_ibm = 8'd102;   // KP 6
            8'h75: ps2_to_ibm = 8'd96;    // KP 8
            8'h76: ps2_to_ibm = 8'd110;   // Escape
            8'h77: ps2_to_ibm = 8'd90;    // Num Lock
            8'h78: ps2_to_ibm = 8'd122;   // F11
            8'h79: ps2_to_ibm = 8'd106;   // KP +
            8'h7a: ps2_to_ibm = 8'd103;   // KP 3
            8'h7b: ps2_to_ibm = 8'd105;   // KP -
            8'h7c: ps2_to_ibm = 8'd100;   // KP *
            8'h7d: ps2_to_ibm = 8'd101;   // KP 9
            8'h7e: ps2_to_ibm = 8'd125;   // Scroll Lock
            8'h83: ps2_to_ibm = 8'd118;   // F7
            default: ps2_to_ibm = 8'd0;
        endcase
    endfunction

    // Extended (0xE0 prefix) scan code translation -- subset that produces
    // a non-zero keycode in x16-smc-main/ps2.h:437-479.
    function automatic logic [7:0] ps2ext_to_ibm(input logic [7:0] sc);
        case (sc)
            8'h11: ps2ext_to_ibm = 8'd62;    // Right Alt
            8'h14: ps2ext_to_ibm = 8'd64;    // Right Ctrl
            8'h1f: ps2ext_to_ibm = 8'd59;    // Left GUI
            8'h27: ps2ext_to_ibm = 8'd63;    // Right GUI
            8'h2f: ps2ext_to_ibm = 8'd65;    // Menu
            8'h69: ps2ext_to_ibm = 8'd81;    // End
            8'h70: ps2ext_to_ibm = 8'd75;    // Insert
            8'h71: ps2ext_to_ibm = 8'd76;    // Delete
            8'h6b: ps2ext_to_ibm = 8'd79;    // Left arrow
            8'h6c: ps2ext_to_ibm = 8'd80;    // Home
            8'h75: ps2ext_to_ibm = 8'd83;    // Up arrow
            8'h72: ps2ext_to_ibm = 8'd84;    // Down arrow
            8'h7d: ps2ext_to_ibm = 8'd85;    // Page up
            8'h7a: ps2ext_to_ibm = 8'd86;    // Page down
            8'h74: ps2ext_to_ibm = 8'd89;    // Right arrow
            8'h4a: ps2ext_to_ibm = 8'd95;    // KP /
            8'h5a: ps2ext_to_ibm = 8'd108;   // KP Enter
            8'h7c: ps2ext_to_ibm = 8'd124;   // KP PrtScr
            8'h15: ps2ext_to_ibm = 8'd126;   // Pause/Break
            8'h77: ps2ext_to_ibm = 8'd126;   // Pause/Break (MiSTer ps2_key ext $77)
            default: ps2ext_to_ibm = 8'd0;
        endcase
    endfunction

    // ====================================================================
    // Keyboard-initiated system requests (jyv 2026-07-07) -- mirrors the
    // x16-smc firmware's modifier tracking in ps2.h updateState():
    //   Ctrl+Alt+Del            -> kbd_reset_req (system reset)
    //   Ctrl+Alt+PrtScr/Restore -> kbd_nmi_req   (NMI)
    // The firmware checks non-extended $84 for PrtScr because that is what
    // a real PS/2 keyboard sends for Alt+PrtScr (SysRq substitution).
    // MiSTer Main may deliver PrtScr as plain E0 7C instead, so BOTH forms
    // arm the NMI here.  Del is extended E0 71, like the firmware.
    // ====================================================================
    logic mod_lctrl, mod_rctrl, mod_lalt, mod_ralt;
    logic kbd_reset_req, kbd_nmi_req;
    logic i2c_reset_req, i2c_nmi_req;   // I2C command $02/$03 path (below)
    assign reset_req = i2c_reset_req | kbd_reset_req;
    assign nmi_req   = i2c_nmi_req   | kbd_nmi_req;
    wire  ctrl_alt_down = (mod_lctrl | mod_rctrl) & (mod_lalt | mod_ralt);

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            mod_lctrl     <= 1'b0;
            mod_rctrl     <= 1'b0;
            mod_lalt      <= 1'b0;
            mod_ralt      <= 1'b0;
            kbd_reset_req <= 1'b0;
            kbd_nmi_req   <= 1'b0;
        end else begin
            kbd_reset_req <= 1'b0;      // 1-cycle pulses
            kbd_nmi_req   <= 1'b0;
            if (uart_byte_valid) begin
                case (u_state)
                    U_IDLE: begin           // regular make codes
                        if      (uart_byte == 8'h14) mod_lctrl <= 1'b1;
                        else if (uart_byte == 8'h11) mod_lalt  <= 1'b1;
                        else if (uart_byte == 8'h84 && ctrl_alt_down) kbd_nmi_req <= 1'b1;
                    end
                    U_BREAK_KEY: begin      // after F0
                        if      (uart_byte == 8'h14) mod_lctrl <= 1'b0;
                        else if (uart_byte == 8'h11) mod_lalt  <= 1'b0;
                    end
                    U_EXT_KEY: begin        // after E0 (F0 falls through)
                        if      (uart_byte == 8'h14) mod_rctrl <= 1'b1;
                        else if (uart_byte == 8'h11) mod_ralt  <= 1'b1;
                        else if (uart_byte == 8'h71 && ctrl_alt_down) kbd_reset_req <= 1'b1;
                        else if ((uart_byte == 8'h7C || uart_byte == 8'h77) && ctrl_alt_down) kbd_nmi_req <= 1'b1;
                    end
                    U_EXT_BREAK_KEY: begin  // after E0 F0
                        if      (uart_byte == 8'h14) mod_rctrl <= 1'b0;
                        else if (uart_byte == 8'h11) mod_ralt  <= 1'b0;
                    end
                    default: ;
                endcase
            end
        end
    end

    // ----- Keyboard FIFO (16 deep, 8 wide).  Stores IBM keycodes with
    //       bit 7 = release flag, matching the I2C $07/$41 protocol. -----
    localparam int KFIFO_DEPTH = 16;
    logic [7:0] kfifo [0:KFIFO_DEPTH-1];
    logic [4:0] kfifo_count;
    logic [3:0] kfifo_rd_ptr, kfifo_wr_ptr;
    wire        kfifo_empty = (kfifo_count == 5'd0);
    wire        kfifo_full  = (kfifo_count == 5'(KFIFO_DEPTH));

    // ----- Mouse packet buffer (single 4-byte packet, has-data flag) -----
    logic [7:0] mouse_pkt [0:3];
    logic       mouse_has_pkt;

    // ----- Joystick state (latest STATE byte per stick) -----
    logic [7:0] joy_a, joy_b;
    logic       joy_b_seen;

    // ----- Declarations hoisted above first use (ModelSim requires
    //       declare-before-use; Quartus tolerated the old order) -----
    logic       kbd_pop;             // 1-cycle when a keyboard byte is delivered
    logic       mouse_pop;           // 1-cycle when a complete mouse packet is delivered
    logic [7:0] tx_byte_pre;         // next byte to serve on an I2C read
    logic       unused_state_sink;

    // Combinational: decode the UART byte under the current state and
    // produce (translated_keycode, is_release).  When uart_byte_valid is
    // high and the state implies "this is a scan code", we push.
    logic [7:0] xlated;
    logic       is_release;
    logic       push_key;
    logic       push_mouse;
    logic       push_joy_a, push_joy_b;

    always_comb begin
        // Defaults
        push_key    = 1'b0;
        push_mouse  = 1'b0;
        push_joy_a  = 1'b0;
        push_joy_b  = 1'b0;
        xlated      = 8'h00;
        is_release  = 1'b0;

        if (uart_byte_valid) begin
            case (u_state)
                U_IDLE: begin
                    // Special leading bytes select sub-protocols.
                    // Anything else is a regular PS/2 scancode press.
                    if (uart_byte == 8'hFF) ;                 // -> U_MOUSE_S
                    else if (uart_byte == 8'hFE) ;            // -> U_JOY_BYTE (A)
                    else if (uart_byte == 8'hFD) ;            // -> U_JOY_BYTE (B)
                    else if (uart_byte == 8'hE0) ;            // -> U_EXT_KEY
                    else if (uart_byte == 8'hF0) ;            // -> U_BREAK_KEY
                    else begin
                        xlated     = ps2_to_ibm(uart_byte);
                        is_release = 1'b0;
                        push_key   = (xlated != 8'h00);
                    end
                end

                U_BREAK_KEY: begin
                    xlated     = ps2_to_ibm(uart_byte);
                    is_release = 1'b1;
                    push_key   = (xlated != 8'h00);
                end

                U_EXT_KEY: begin
                    if (uart_byte == 8'hF0) ;                 // -> U_EXT_BREAK_KEY
                    else begin
                        xlated     = ps2ext_to_ibm(uart_byte);
                        is_release = 1'b0;
                        push_key   = (xlated != 8'h00);
                    end
                end

                U_EXT_BREAK_KEY: begin
                    xlated     = ps2ext_to_ibm(uart_byte);
                    is_release = 1'b1;
                    push_key   = (xlated != 8'h00);
                end

                U_MOUSE_W:   push_mouse = 1'b1;

                U_JOY_BYTE:  begin
                    push_joy_a = ~u_joy_is_b;
                    push_joy_b =  u_joy_is_b;
                end

                default: ;
            endcase
        end
    end

    // FIFO + state machine update.
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            u_state       <= U_IDLE;
            u_joy_is_b    <= 1'b0;
            u_mouse_s     <= 8'h00;
            u_mouse_dx    <= 8'h00;
            u_mouse_dy    <= 8'h00;
            kfifo_count   <= 5'd0;
            kfifo_rd_ptr  <= 4'd0;
            kfifo_wr_ptr  <= 4'd0;
            mouse_has_pkt <= 1'b0;
            mouse_pkt[0]  <= 8'h00; mouse_pkt[1] <= 8'h00;
            mouse_pkt[2]  <= 8'h00; mouse_pkt[3] <= 8'h00;
            joy_a         <= 8'h00;
            joy_b         <= 8'h00;
            joy_b_seen    <= 1'b0;
        end
        else begin
            // ---- UART decoder state transitions ----
            if (uart_byte_valid) begin
                case (u_state)
                    U_IDLE: begin
                        if      (uart_byte == 8'hFF) u_state <= U_MOUSE_S;
                        else if (uart_byte == 8'hFE) begin u_state <= U_JOY_BYTE; u_joy_is_b <= 1'b0; end
                        else if (uart_byte == 8'hFD) begin u_state <= U_JOY_BYTE; u_joy_is_b <= 1'b1; end
                        else if (uart_byte == 8'hE0) u_state <= U_EXT_KEY;
                        else if (uart_byte == 8'hF0) u_state <= U_BREAK_KEY;
                        else                          u_state <= U_IDLE;
                    end
                    U_BREAK_KEY:     u_state <= U_IDLE;
                    U_EXT_KEY:       u_state <= (uart_byte == 8'hF0) ? U_EXT_BREAK_KEY : U_IDLE;
                    U_EXT_BREAK_KEY: u_state <= U_IDLE;
                    U_MOUSE_S:       begin u_mouse_s  <= uart_byte; u_state <= U_MOUSE_DX; end
                    U_MOUSE_DX:      begin u_mouse_dx <= uart_byte; u_state <= U_MOUSE_DY; end
                    U_MOUSE_DY:      begin u_mouse_dy <= uart_byte; u_state <= U_MOUSE_W; end
                    U_MOUSE_W:       u_state <= U_IDLE;
                    U_JOY_BYTE:      u_state <= U_IDLE;
                    default:         u_state <= U_IDLE;
                endcase
            end

            // ---- Push translated keycode into FIFO (drop on overflow) ----
            // 4-bit wr_ptr naturally wraps 15 -> 0.
            if (push_key && !kfifo_full) begin
                kfifo[kfifo_wr_ptr] <= {is_release, xlated[6:0]};
                kfifo_wr_ptr        <= kfifo_wr_ptr + 4'd1;
            end

            // ---- Push mouse packet ----
            if (push_mouse) begin
                mouse_pkt[0]  <= u_mouse_s;
                mouse_pkt[1]  <= u_mouse_dx;
                mouse_pkt[2]  <= u_mouse_dy;
                mouse_pkt[3]  <= uart_byte;
                mouse_has_pkt <= 1'b1;
            end

            // ---- Latch joystick state ----
            if (push_joy_a) joy_a <= uart_byte;
            if (push_joy_b) begin
                joy_b      <= uart_byte;
                joy_b_seen <= 1'b1;
            end

            // ---- Combined FIFO count update + pop signal from I2C side ----
            case ({push_key && !kfifo_full, kbd_pop && !kfifo_empty})
                2'b10: kfifo_count <= kfifo_count + 5'd1;
                2'b01: kfifo_count <= kfifo_count - 5'd1;
                default: ;       // 00 or 11 -> no change
            endcase

            // 4-bit rd_ptr naturally wraps 15 -> 0.
            if (kbd_pop && !kfifo_empty)
                kfifo_rd_ptr <= kfifo_rd_ptr + 4'd1;

            if (mouse_pop) mouse_has_pkt <= 1'b0;
        end
    end

    assign dbg_kbd_count = kfifo_count;
    assign dbg_kbd_pop   = kbd_pop;
    assign dbg_tx_byte   = tx_byte_pre;

    // ====================================================================
    // I2C slave state machine (address $42)
    // ====================================================================
    // 3-stage synchronizer on SDA/SCL.
    logic sda_s1, sda_s2, sda_s3;
    logic scl_s1, scl_s2, scl_s3;
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sda_s1 <= 1'b1; sda_s2 <= 1'b1; sda_s3 <= 1'b1;
            scl_s1 <= 1'b1; scl_s2 <= 1'b1; scl_s3 <= 1'b1;
        end else begin
            sda_s1 <= sda_bus; sda_s2 <= sda_s1; sda_s3 <= sda_s2;
            scl_s1 <= scl_bus; scl_s2 <= scl_s1; scl_s3 <= scl_s2;
        end
    end

    // Synchronized bus state (renamed to avoid clashing with the `sda`
    // / `scl` port names of this module).
    wire sda_q      = sda_s2;
    wire scl_q      = scl_s2;
    wire scl_rise   = scl_s2 & ~scl_s3;
    wire scl_fall   = ~scl_s2 & scl_s3;
    wire sda_rise   = sda_s2 & ~sda_s3;
    wire sda_fall   = ~sda_s2 & sda_s3;
    wire start_cond = sda_fall & scl_q;
    wire stop_cond  = sda_rise & scl_q;

    typedef enum logic [3:0] {
        S_IDLE,
        S_ADDR,
        S_ADDR_ACK,
        S_RX_BYTE,
        S_RX_ACK,
        S_TX_BYTE,
        S_TX_PRE_ACK,
        S_TX_ACK
    } sstate_t;
    sstate_t sstate;

    logic [7:0] shift;
    logic [3:0] bitcnt;
    logic       rw_bit;            // 0=write, 1=read
    logic       got_cmd;           // a pending command is latched in cur_cmd
    logic [7:0] cur_cmd;           // the pending command
    // Command persistence (mirrors I2C_Receive/I2C_Send in x16-smc.ino):
    // the KERNAL's i2c_read_byte does  write cmd, STOP, START, addr+R  --
    // a full STOP, not a repeated START -- so the pending command must
    // SURVIVE a STOP.  In the firmware, I2C_Receive early-returns for a
    // 1-byte (command-only) write, leaving I2C_Data[0] = cmd; it resets
    // I2C_Data[0] = defaultRequest only after a write WITH data, and
    // I2C_Send does the same after serving any read.  Track both.
    logic       first_wr_done;     // a write byte was received THIS transaction
    logic       wrote_data;        // this transaction carried data after the cmd
    logic       did_read;          // this transaction served at least one read byte

    // Default-read state.
    logic [7:0] default_request;

    // Mouse multi-byte read state (used for $21 / $42 / $43 commands).
    // Counts which byte of the packet we're about to serve.
    logic [1:0] mouse_rd_idx;

    // PS2 data fast ($43): byte 0 = keycode, then 4 mouse bytes.
    logic [2:0] ps2_rd_phase;      // 0=kbd, 1..4=mouse[0..3]

    // Latched state per the README.
    logic [7:0] echo_byte_r;
    logic [7:0] mouse_req_id_r;
    logic [7:0] flash_page_r;
    logic [7:0] bootldr_ver_r;

    // Sticky one-shots set by the I2C handler, cleared by the consumer
    // (CPU reset / NMI / power-off path in C5G_x16).
    logic [7:0] kbd_status_r;       // last keyboard command status

    // (kbd_pop / mouse_pop declared above the FIFO updater)

    // ---- Versioning ----
    localparam logic [7:0] FW_VER_MAJOR = 8'd48;
    localparam logic [7:0] FW_VER_MINOR = 8'd1;
    localparam logic [7:0] FW_VER_PATCH = 8'd0;

    // Determine effective command:
    //   - If got_cmd has been seen this transaction, that's the cmd.
    //   - Else (master read without preceding write) use default_request.
    wire [7:0] eff_cmd = got_cmd ? cur_cmd : default_request;

    // Compute byte to serve next, given eff_cmd and read sub-state.
    // For multi-byte reads, mouse_rd_idx / ps2_rd_phase tracks position.
    always_comb begin
        case (eff_cmd)
            CMD_GET_KEYCODE,
            CMD_GET_KEYCODE_FAST:
                tx_byte_pre = kfifo_empty ? 8'h00 : kfifo[kfifo_rd_ptr];

            CMD_GET_MOUSE_MOV,
            CMD_GET_MOUSE_MOV_FAST:
                tx_byte_pre = mouse_has_pkt ? mouse_pkt[mouse_rd_idx] : 8'h00;

            CMD_GET_PS2DATA_FAST: begin
                case (ps2_rd_phase)
                    3'd0:    tx_byte_pre = kfifo_empty ? 8'h00 : kfifo[kfifo_rd_ptr];
                    3'd1:    tx_byte_pre = mouse_has_pkt ? mouse_pkt[0] : 8'h00;
                    3'd2:    tx_byte_pre = mouse_has_pkt ? mouse_pkt[1] : 8'h00;
                    3'd3:    tx_byte_pre = mouse_has_pkt ? mouse_pkt[2] : 8'h00;
                    default: tx_byte_pre = mouse_has_pkt ? mouse_pkt[3] : 8'h00;
                endcase
            end

            CMD_GET_KBD_STATUS:    tx_byte_pre = kbd_status_r;
            CMD_KBD_INIT_STATE:    tx_byte_pre = 8'h01;     // always ready
            CMD_GET_MOUSE_ID:      tx_byte_pre = 8'h03;     // Intellimouse (wheel)
            CMD_GET_FUSE_LOW:      tx_byte_pre = 8'hF1;
            CMD_GET_FUSE_LOCK:     tx_byte_pre = 8'hFF;
            CMD_GET_FUSE_EXT:      tx_byte_pre = 8'hFE;
            CMD_GET_FUSE_HIGH:     tx_byte_pre = 8'hD4;
            CMD_GET_VER1:          tx_byte_pre = FW_VER_MAJOR;
            CMD_GET_VER2:          tx_byte_pre = FW_VER_MINOR;
            CMD_GET_VER3:          tx_byte_pre = FW_VER_PATCH;
            CMD_ECHO:              tx_byte_pre = echo_byte_r;
            CMD_DBG_OUT:           tx_byte_pre = 8'h00;     // long-press flag
            CMD_GET_BOOTLDR_VER:   tx_byte_pre = 8'hFF;     // none installed
            CMD_READ_FLASH:        tx_byte_pre = 8'hFF;
            CMD_FLASH_WRITE_MODE:  tx_byte_pre = 8'h00;     // not enabled
            default:               tx_byte_pre = 8'h00;
        endcase

        // Consume sink in live logic; XOR-by-zero has no functional effect.
        tx_byte_pre = tx_byte_pre ^ {8{unused_state_sink & 1'b0}};
    end

    // Latched tx_shift updated on entry to S_TX_BYTE (so a single byte's
    // serialization doesn't see a mid-stream change).
    logic [7:0] tx_shift;

    // Compute the byte that will be served NEXT (after the current ACK
    // commits the pop).  Mirrors the increment logic in S_TX_ACK so the
    // served byte is correct even though mouse_rd_idx / ps2_rd_phase
    // don't visibly update until the next clock edge.  Implemented as a
    // simple combinational mux (not a function call) -- earlier versions
    // used `function automatic` returning via the function name, which
    // crashed Quartus 24.1 Lite during elaboration of the array index
    // expression below.
    logic [1:0] mouse_next_idx;
    logic [7:0] mouse_next_byte;
    logic [7:0] tx_byte_next;
    always_comb begin
        mouse_next_idx  = mouse_rd_idx + 2'd1;       // 2-bit: wraps 3 -> 0
        mouse_next_byte = mouse_pkt[mouse_next_idx];
        case (eff_cmd)
            CMD_GET_KEYCODE,
            CMD_GET_KEYCODE_FAST:
                // After a pop the next byte at rd_ptr+1 isn't visible
                // until the next cycle.  Return 0; master typically
                // NACKs after a single byte for $07/$41.
                tx_byte_next = 8'h00;

            CMD_GET_MOUSE_MOV,
            CMD_GET_MOUSE_MOV_FAST:
                tx_byte_next = mouse_has_pkt ? mouse_next_byte : 8'h00;

            CMD_GET_PS2DATA_FAST: begin
                case (ps2_rd_phase)
                    3'd0:    tx_byte_next = mouse_has_pkt ? mouse_pkt[0] : 8'h00;
                    3'd1:    tx_byte_next = mouse_has_pkt ? mouse_pkt[1] : 8'h00;
                    3'd2:    tx_byte_next = mouse_has_pkt ? mouse_pkt[2] : 8'h00;
                    3'd3:    tx_byte_next = mouse_has_pkt ? mouse_pkt[3] : 8'h00;
                    default: tx_byte_next = kfifo_empty   ? 8'h00       : kfifo[kfifo_rd_ptr];
                endcase
            end

            default: tx_byte_next = tx_byte_pre;     // single-byte responses
        endcase
    end

    // Defer FIFO/buffer pop to the moment the byte has been ACKed by the
    // master (S_TX_ACK + ACK condition).  This avoids double-popping if
    // the master NACKs and never reads.
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sstate          <= S_IDLE;
            shift           <= 8'h00;
            bitcnt          <= 4'h0;
            rw_bit          <= 1'b0;
            got_cmd         <= 1'b0;
            cur_cmd         <= 8'h00;
            first_wr_done   <= 1'b0;
            wrote_data      <= 1'b0;
            did_read        <= 1'b0;
            default_request <= CMD_GET_KEYCODE_FAST;
            mouse_rd_idx    <= 2'd0;
            ps2_rd_phase    <= 3'd0;
            echo_byte_r     <= 8'h00;
            mouse_req_id_r  <= 8'h04;        // SMC default: ID 4 first
            flash_page_r    <= 8'h00;
            bootldr_ver_r   <= 8'hFF;
            kbd_status_r    <= 8'hFA;        // pretend last command OK
            sda_drive_low   <= 1'b0;
            tx_shift        <= 8'h00;
            kbd_pop         <= 1'b0;
            mouse_pop       <= 1'b0;
            power_off_req   <= 1'b0;
            i2c_reset_req   <= 1'b0;
            i2c_nmi_req     <= 1'b0;
            act_led_r       <= 8'h00;
            dbg_saw_start      <= 1'b0;
            dbg_saw_addr_match <= 1'b0;
            dbg_saw_byte       <= 1'b0;
            dbg_saw_repeat     <= 1'b0;
            dbg_saw_stop       <= 1'b0;
            dbg_saw_tx         <= 1'b0;
            dbg_last_cmd       <= 8'h00;
            dbg_last_addr_byte <= 8'h00;
        end
        else begin
            // Default: clear 1-cycle pulses.
            kbd_pop   <= 1'b0;
            mouse_pop <= 1'b0;
            // The system requests are 1-cycle pulses too (the parent
            // edge-detects/stretches them into reset / NMI actions); sticky
            // levels would allow only ONE NMI ever per power cycle.
            power_off_req <= 1'b0;
            i2c_reset_req <= 1'b0;
            i2c_nmi_req   <= 1'b0;

            if (stop_cond) begin
                // End of transaction; release SDA, return to idle.  The
                // pending command is consumed only by a read or by a write
                // that carried data -- a command-only write ($42 W cmd,
                // STOP) leaves cur_cmd armed for the KERNAL's follow-up
                // START + addr+R read (i2c_read_byte does STOP + START
                // between the two halves, NOT a repeated START).
                if (sstate != S_IDLE) dbg_saw_stop <= 1'b1;
                sstate        <= S_IDLE;
                sda_drive_low <= 1'b0;
                if (did_read || wrote_data) got_cmd <= 1'b0;
                first_wr_done <= 1'b0;
                wrote_data    <= 1'b0;
                did_read      <= 1'b0;
                bitcnt        <= 4'h0;
                shift         <= 8'h00;
                mouse_rd_idx  <= 2'd0;
                ps2_rd_phase  <= 3'd0;
            end
            else if (start_cond) begin
                // START or repeated START -- same transaction-boundary
                // bookkeeping as STOP: a command-only write followed by a
                // repeated START + read still serves cur_cmd; a completed
                // read or data write re-arms the default.
                dbg_saw_start <= 1'b1;
                if (sstate != S_IDLE) dbg_saw_repeat <= 1'b1;
                sstate        <= S_ADDR;
                if (did_read || wrote_data) got_cmd <= 1'b0;
                first_wr_done <= 1'b0;
                wrote_data    <= 1'b0;
                did_read      <= 1'b0;
                bitcnt        <= 4'h0;
                shift         <= 8'h00;
                sda_drive_low <= 1'b0;
                mouse_rd_idx  <= 2'd0;
                ps2_rd_phase  <= 3'd0;
            end
            else case (sstate)
                S_IDLE: ;       // wait for START

                S_ADDR: begin
                    if (scl_rise) begin
                        shift  <= {shift[6:0], sda_q};
                        bitcnt <= bitcnt + 4'h1;
                        if (bitcnt == 4'h7) begin
                            // Capture the full 8-bit address byte regardless
                            // of whether it matches -- lets the board show
                            // what the master is actually driving.
                            dbg_last_addr_byte <= {shift[6:0], sda_q};
                            if (shift[6:0] == SLAVE_ADDR) begin
                                rw_bit             <= sda_q;     // last shifted bit
                                dbg_saw_addr_match <= 1'b1;
                                sstate             <= S_ADDR_ACK;
                            end else begin
                                sstate <= S_IDLE;
                            end
                        end
                    end
                end

                S_ADDR_ACK: begin
                    // Pull SDA low on SCL falling, hold through SCL high,
                    // release on the next SCL falling.
                    if (scl_fall && !sda_drive_low) begin
                        sda_drive_low <= 1'b1;
                    end else if (scl_fall && sda_drive_low) begin
                        shift <= 8'h00;
                        if (rw_bit) begin
                            // jyv 2026-06-27: drive the FIRST data bit
                            // (MSB) RIGHT HERE on the ACK's falling edge.
                            // Master will raise SCL next to sample the
                            // bit -- if we wait for S_TX_BYTE's first
                            // scl_fall to drive it, SDA is still
                            // released (1) when the master samples and
                            // every byte's MSB reads as 1.
                            // pre-shift tx_shift so S_TX_BYTE drives the
                            // remaining 7 bits in order.
                            sda_drive_low <= ~tx_byte_pre[7];
                            tx_shift      <= {tx_byte_pre[6:0], 1'b0};
                            bitcnt        <= 4'h1;   // 1 bit driven here
                            did_read      <= 1'b1;   // consume pending cmd at boundary
                            sstate        <= S_TX_BYTE;
                        end else begin
                            sda_drive_low <= 1'b0;
                            bitcnt        <= 4'h0;
                            sstate        <= S_RX_BYTE;
                        end
                    end
                end

                // Master writing to slave: 8 bits, MSB first, on SCL rising.
                S_RX_BYTE: begin
                    if (scl_rise) begin
                        shift  <= {shift[6:0], sda_q};
                        bitcnt <= bitcnt + 4'h1;
                        if (bitcnt == 4'h7) sstate <= S_RX_ACK;
                    end
                end

                S_RX_ACK: begin
                    if (scl_fall && !sda_drive_low) begin
                        // Process the byte right when we ACK it.
                        dbg_saw_byte <= 1'b1;
                        if (!first_wr_done) begin
                            // First write byte of THIS transaction is always
                            // a new command (replaces any leftover cur_cmd).
                            first_wr_done <= 1'b1;
                            cur_cmd       <= shift;
                            got_cmd       <= 1'b1;
                            dbg_last_cmd  <= shift;
                        end
                        else begin
                            // Later bytes of this transaction are data.
                            wrote_data <= 1'b1;
                            case (cur_cmd)
                                CMD_POW_OFF: if (shift == 8'h00) power_off_req <= 1'b1;
                                CMD_RESET:   if (shift == 8'h00) i2c_reset_req <= 1'b1;
                                CMD_NMI:     if (shift == 8'h00) i2c_nmi_req   <= 1'b1;
                                CMD_SET_ACT_LED:        act_led_r       <= shift;
                                CMD_ECHO:               echo_byte_r     <= shift;
                                CMD_DBG_OUT:            /* ignore */ ;
                                CMD_KBD_CMD1,
                                CMD_KBD_CMD2:           kbd_status_r    <= 8'hFA; // pretend ok
                                CMD_SET_MOUSE_ID:       mouse_req_id_r  <= shift;
                                CMD_SET_DFLT_READ_OP:   default_request <= shift;
                                CMD_BOOTLDR_START:      /* ignore */ ;
                                CMD_SET_FLASH_PAGE:     flash_page_r    <= shift;
                                CMD_WRITE_FLASH:        /* ignore */ ;
                                CMD_FLASH_WRITE_MODE:   /* ignore */ ;
                                CMD_GET_BOOTLDR_VER:    bootldr_ver_r   <= shift;
                                default: ;
                            endcase
                        end
                        sda_drive_low <= 1'b1;
                    end
                    else if (scl_fall && sda_drive_low) begin
                        sda_drive_low <= 1'b0;
                        bitcnt        <= 4'h0;
                        shift         <= 8'h00;
                        sstate        <= S_RX_BYTE;
                    end
                end

                // Master reading from slave: drive each bit on SCL falling
                // so it is stable when master clocks SCL high.
                S_TX_BYTE: begin
                    if (scl_fall) begin
                        dbg_saw_tx    <= 1'b1;
                        sda_drive_low <= ~tx_shift[7];
                        tx_shift      <= {tx_shift[6:0], 1'b0};
                        bitcnt        <= bitcnt + 4'h1;
                        // After driving bit 0, wait through one more
                        // scl_rise (master samples it) before releasing
                        // SDA for the ACK exchange.
                        if (bitcnt == 4'h7) sstate <= S_TX_PRE_ACK;
                    end
                end

                S_TX_PRE_ACK: begin
                    // SCL is low (we just drove bit 0).  Master will clock
                    // SCL high to sample bit 0, then low.  On that scl_fall
                    // we release SDA and proceed to the ACK sample state.
                    if (scl_fall) begin
                        sda_drive_low <= 1'b0;
                        sstate        <= S_TX_ACK;
                    end
                end

                S_TX_ACK: begin
                    if (scl_rise) begin
                        // Sample SDA: 0 = ACK (more bytes), 1 = NACK (stop).
                        // I2C semantics: the byte was successfully delivered
                        // either way (slave drove 8 bits, master read them).
                        // ACK/NAK only tells the slave whether to send a
                        // NEXT byte.  So pop/advance the source buffer in
                        // both cases; only the next-state differs.
                        case (eff_cmd)
                            CMD_GET_KEYCODE,
                            CMD_GET_KEYCODE_FAST: begin
                                if (!kfifo_empty) kbd_pop <= 1'b1;
                            end
                            CMD_GET_MOUSE_MOV,
                            CMD_GET_MOUSE_MOV_FAST: begin
                                if (mouse_rd_idx == 2'd3) begin
                                    if (mouse_has_pkt) mouse_pop <= 1'b1;
                                    mouse_rd_idx <= 2'd0;
                                end else begin
                                    mouse_rd_idx <= mouse_rd_idx + 2'd1;
                                end
                            end
                            CMD_GET_PS2DATA_FAST: begin
                                if (ps2_rd_phase == 3'd0) begin
                                    if (!kfifo_empty) kbd_pop <= 1'b1;
                                    ps2_rd_phase <= 3'd1;
                                end else if (ps2_rd_phase == 3'd4) begin
                                    if (mouse_has_pkt) mouse_pop <= 1'b1;
                                    ps2_rd_phase <= 3'd0;
                                end else begin
                                    ps2_rd_phase <= ps2_rd_phase + 3'd1;
                                end
                            end
                            default: ;
                        endcase

                        if (sda_q == 1'b0) begin
                            // ACK -> prepare next byte for back-to-back read.
                            tx_shift <= tx_byte_next;
                            bitcnt   <= 4'h0;
                            sstate   <= S_TX_BYTE;
                        end else begin
                            // NAK -> master is done; wait for STOP/repeated START.
                            sstate <= S_IDLE;
                        end
                    end
                end

                default: sstate <= S_IDLE;
            endcase
        end
    end

    // These latched states are intentionally retained for compatibility and
    // future feature work even when not consumed by current datapath.
    always_comb begin
        unused_state_sink = ^{joy_a, joy_b, joy_b_seen, mouse_req_id_r,
                              flash_page_r, bootldr_ver_r};
    end

endmodule
