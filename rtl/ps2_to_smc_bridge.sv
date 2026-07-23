//============================================================================
// ps2_to_smc_bridge.sv  -  MiSTer hps_io ps2_key  ->  smc_x16 uart_byte stream
//
// Phase e of the X16 MiSTer port.  The smc_x16 module consumes a host byte
// stream on (uart_byte, uart_byte_valid) -- the same protocol the C5G
// reference fed over a real UART from Scripts/send_input.py:
//
//     key make (press)    : extended ? {E0, code} : {code}
//     key break (release) : extended ? {E0, F0, code} : {F0, code}
//
// where `code` is the raw PS/2 Set-2 base scancode (no E0/F0 prefixes).
//
// MiSTer's hps_io already decodes PS/2 into a clean event word:
//     ps2_key[7:0]  raw Set-2 scancode (E0/F0 prefixes stripped)
//     ps2_key[8]    extended  (key had an E0 prefix)
//     ps2_key[9]    pressed   (1 = make, 0 = break)
//     ps2_key[10]   strobe    (toggles once per key event)
//
// so [7:0] maps directly to `code`.  This bridge watches the strobe bit and,
// on each toggle, emits the 1-, 2-, or 3-byte sequence above, one byte per
// cpu_clk cycle with a single-cycle uart_byte_valid pulse per byte.
//
// hps_io runs in the 100 MHz system domain; x16.sv synchronizes its held event
// word into cpu_clk.  Capture every observed toggle into a small queue before
// serializing it.  Without this queue, two toggles while a key/mouse packet was
// being emitted could return the strobe to its previous level and lose both
// events.
//============================================================================
// MOUSE (2026-07-05, wheel added 2026-07-07): hps_io's ps2_mouse[24:0] is
// forwarded as the SMC injection protocol's 5-byte packet
//   FF, status, dx, dy, wheel                     (see smc_x16.sv header)
// [24] is the new-event toggle and [7:0]/[15:8]/[23:16] are the PS/2 packet
// bytes 0/1/2 verbatim (buttons+signs / X / Y).  The wheel delta arrives on
// hps_io's ps2_mouse_ext[7:0] (8-bit signed, updated with the same toggle);
// the KERNAL consumes only a SIGNED 4-BIT wheel nibble (Intellimouse byte-4
// format), so the delta is saturated to [-8..+7] and sent in the low nibble.
// A packet is emitted atomically (never interleaved with a key sequence);
// events are ms apart vs a 5-cycle emission, so one pending flag per source
// suffices.
// TYPEMATIC (2026-07-07): on real X16 hardware the PS/2 KEYBOARD repeats the
// make code of the last held key (500 ms delay, then ~10.9 cps); the KERNAL
// does no software repeat.  MiSTer Main drops Linux key-repeat events for
// non-ps2ctl cores (send_keycode: `if (press > 1 && !use_ps2ctl) return;`),
// so this bridge -- being the keyboard -- synthesizes typematic: the last
// make is re-emitted while held, retargeted by any newer make, stopped by
// its own break (a break of a DIFFERENT key does not stop it, like real
// typematic).  Pause (ext $77) never repeats: hps_io never sends its break.
module ps2_to_smc_bridge #(
    parameter int TPM_DELAY = 4_000_000,  // 500 ms  @ 8 MHz cpu_clk
    parameter int TPM_RATE  =   736_000   // 10.9 cps @ 8 MHz cpu_clk
) (
    input  logic        clk,             // cpu_clk
    input  logic        reset_n,         // cpu_reset_n

    input  logic [10:0] ps2_key,         // from hps_io {strobe,pressed,ext,code}
    input  logic [24:0] ps2_mouse,       // from hps_io {strobe,dy,dx,status}
    input  logic [7:0]  ps2_mouse_wheel, // from hps_io ps2_mouse_ext[7:0] (signed)

    output logic [7:0]  uart_byte,
    output logic        uart_byte_valid
);

    // Latched event being emitted.
    logic       ev_ext;
    logic       ev_pressed;
    logic [7:0] ev_code;

    // Byte for position `i` of the current sequence (combinational on ev_*).
    function automatic logic [7:0] seq_byte(input logic [1:0] i);
        unique case ({ev_ext, ev_pressed})
            2'b11:   seq_byte = (i == 2'd0) ? 8'hE0 : ev_code;                       // ext  make : E0,code
            2'b10:   seq_byte = (i == 2'd0) ? 8'hE0 : (i == 2'd1) ? 8'hF0 : ev_code; // ext  break: E0,F0,code
            2'b01:   seq_byte = ev_code;                                             // reg  make : code
            default: seq_byte = (i == 2'd0) ? 8'hF0 : ev_code;                       // reg  break: F0,code
        endcase
    endfunction

    // Number of bytes in the current sequence.
    logic [1:0] nbytes;
    always_comb begin
        unique case ({ev_ext, ev_pressed})
            2'b01:   nbytes = 2'd1;  // reg make
            2'b10:   nbytes = 2'd3;  // ext break
            default: nbytes = 2'd2;  // ext make / reg break
        endcase
    end

    localparam logic [1:0] S_IDLE = 2'd0, S_EMIT = 2'd1, S_MOUSE = 2'd2;
    logic [1:0] state;
    logic [2:0] idx;
    logic       last_strobe, last_mstrobe;

    // Decouple toggle capture from byte emission.  Eight entries comfortably
    // cover MiSTer modifier/key bursts while the serializer drains at up to
    // three clocks per keyboard event.
    logic [9:0] key_fifo [0:7];             // {pressed, extended, code}
    logic [2:0] key_rd_ptr, key_wr_ptr;
    logic [3:0] key_count;
    wire        key_arrive = (ps2_key[10] != last_strobe);
    wire        key_push   = key_arrive && (key_count != 4'd8);
    wire        key_pop    = (state == S_IDLE) && (key_count != 4'd0);

    // Typematic: last held make (retargeted by any newer make, cleared by
    // its own break) and the delay/rate countdown.
    logic        tpm_valid;
    logic        tpm_ext;
    logic [7:0]  tpm_code;
    logic [31:0] tpm_cnt;
    logic [31:0] mev;                    // latched mouse packet {wheel,dy,dx,status}

    // wheel delta saturated to the KERNAL's signed 4-bit nibble
    wire signed [7:0] wz = $signed(ps2_mouse_wheel);
    wire [3:0] wheel_nib = (wz > 8'sd7)  ? 4'h7 :
                           (wz < -8'sd8) ? 4'h8 : wz[3:0];

    // byte for position `i` of a mouse packet: FF, status, dx, dy, wheel
    function automatic logic [7:0] mseq_byte(input logic [2:0] i);
        case (i)
            3'd0:    mseq_byte = 8'hFF;
            3'd1:    mseq_byte = mev[7:0];
            3'd2:    mseq_byte = mev[15:8];
            3'd3:    mseq_byte = mev[23:16];
            default: mseq_byte = mev[31:24];
        endcase
    endfunction

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state           <= S_IDLE;
            idx             <= 2'd0;
            last_strobe     <= 1'b0;
            last_mstrobe    <= 1'b0;
            key_rd_ptr      <= 3'd0;
            key_wr_ptr      <= 3'd0;
            key_count       <= 4'd0;
            ev_ext          <= 1'b0;
            ev_pressed      <= 1'b0;
            ev_code         <= 8'h00;
            mev             <= 32'h0;
            uart_byte       <= 8'h00;
            uart_byte_valid <= 1'b0;
            tpm_valid       <= 1'b0;
            tpm_ext         <= 1'b0;
            tpm_code        <= 8'h00;
            tpm_cnt         <= 32'h0;
        end else begin
            uart_byte_valid <= 1'b0;   // default: single-cycle pulses only

            // Observe key toggles in every FSM state, including while a mouse
            // packet or an earlier key sequence is being serialized.
            if (key_arrive) last_strobe <= ps2_key[10];
            if (key_push) begin
                key_fifo[key_wr_ptr] <= ps2_key[9:0];
                key_wr_ptr           <= key_wr_ptr + 3'd1;
            end
            if (key_pop) key_rd_ptr <= key_rd_ptr + 3'd1;
            case ({key_push, key_pop})
                2'b10: key_count <= key_count + 4'd1;
                2'b01: key_count <= key_count - 4'd1;
                default: ;
            endcase

            // typematic countdown runs regardless of FSM state
            if (tpm_valid && tpm_cnt != 0) tpm_cnt <= tpm_cnt - 32'd1;

            case (state)
                S_IDLE: begin
                    if (key_count != 0) begin
                        ev_ext      <= key_fifo[key_rd_ptr][8];
                        ev_pressed  <= key_fifo[key_rd_ptr][9];
                        ev_code     <= key_fifo[key_rd_ptr][7:0];
                        idx         <= 3'd0;
                        state       <= S_EMIT;
                        if (key_fifo[key_rd_ptr][9]) begin
                            // make: retarget typematic (except Pause, which
                            // never gets a break from hps_io)
                            tpm_valid <= ~(key_fifo[key_rd_ptr][8] &
                                           (key_fifo[key_rd_ptr][7:0] == 8'h77));
                            tpm_ext   <= key_fifo[key_rd_ptr][8];
                            tpm_code  <= key_fifo[key_rd_ptr][7:0];
                            tpm_cnt   <= TPM_DELAY;
                        end else if (key_fifo[key_rd_ptr][8] == tpm_ext &&
                                     key_fifo[key_rd_ptr][7:0] == tpm_code) begin
                            // break of the held key stops its repeat
                            tpm_valid <= 1'b0;
                        end
                    end else if (ps2_mouse[24] != last_mstrobe) begin
                        last_mstrobe <= ps2_mouse[24];
                        mev          <= {4'h0, wheel_nib, ps2_mouse[23:0]};
                        idx          <= 3'd0;
                        state        <= S_MOUSE;
                    end else if (tpm_valid && tpm_cnt == 0) begin
                        // typematic repeat: re-emit the held key's make
                        ev_ext      <= tpm_ext;
                        ev_pressed  <= 1'b1;
                        ev_code     <= tpm_code;
                        idx         <= 3'd0;
                        state       <= S_EMIT;
                        tpm_cnt     <= TPM_RATE;
                    end
                end

                S_EMIT: begin
                    uart_byte       <= seq_byte(idx[1:0]);
                    uart_byte_valid <= 1'b1;
                    if (idx[1:0] == nbytes - 2'd1) begin
                        if (ev_ext && (ev_code == 8'h77) && ev_pressed) begin
                            // Pause key make complete: auto-emit break sequence (E0 F0 77)
                            // because hps_io never sends a break event for Pause.
                            ev_pressed <= 1'b0;
                            idx        <= 3'd0;
                        end else begin
                            state      <= S_IDLE;
                        end
                    end else begin
                        idx <= idx + 3'd1;
                    end
                end

                S_MOUSE: begin
                    uart_byte       <= mseq_byte(idx);
                    uart_byte_valid <= 1'b1;
                    if (idx == 3'd4) state <= S_IDLE;
                    idx <= idx + 3'd1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
