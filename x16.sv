//============================================================================
//  Commander X16 for MiSTer  --  Phase d top-level wiring
//
//  Mirrors c:\quartus\projects\x16_monitor\rtl\C5G_x16.sv (which targets
//  the Terasic C5G board) but drives the MiSTer `emu` framework signals
//  instead of C5G pins.
//
//  Phase status:
//    a-c done.  Phase d wires every module; bitstream will compile.
//    Phase e  -- DONE: ps2_to_smc_bridge feeds hps_io ps2_key into smc_x16.
//    Phase f  -- SDRAM HiRAM controller (currently ext_ram returns 0).
//    Phase g  -- HPS ROM loader (rom_banks uses $readmemh rom/rom.hex).
//
//  PLL: instantiates the `pll` IP (root pll.qip), generated for a 50 MHz
//  reference (fed CLK_50M below).  Output order: outclk_0 25.0 MHz (pix),
//  outclk_1 12.5 MHz (aud), outclk_2 8.0 MHz (cpu), outclk_3 100.0 MHz (sdram).
//
//  NOTE: the emu-level instance is named `pll` and the IP's inner instance is
//  `pll_inst`, so the timing path is emu|pll|pll_inst|altera_pll_i|... -- this
//  is REQUIRED for sys/sys_top.sdc line 14 to match and decouple the core
//  clocks from the HDMI/audio/system clock groups.  Do not rename to u_pll.
//============================================================================

module emu
(
    `include "sys/emu_ports.vh"
);

    // ========================================================================
    // Clock generation
    // ========================================================================
    wire pll_locked;
    wire pix_clk;       // VERA pixel clock          -- 25.0 MHz
    wire aud_mclk;      // IKAOPM master clock        -- 12.5 MHz
    wire cpu_clk;       // CPU + VIA + ext_ram clock  -- 8.0 MHz
    wire sdram_clk;     // SDRAM controller clock     -- 100.0 MHz

    // PLL reset: FREE-RUN the core PLL (rst=0) instead of tying it to the
    // framework RESET.  jyv 2026-07-13 (MiSTer Pi total-freeze fix):
    //   hps_io (OSD + reset-button handler) and the pixel clock are BOTH
    //   clocked by this PLL's outputs (sdram_clk / pix_clk).  Feeding the
    //   framework RESET into .rst stops those clocks every time the framework
    //   asserts reset -- HDMI, OSD and reset all die together, exactly the
    //   reported MiSTer Pi symptom.  Recovery depends on the PLL re-locking
    //   before the framework/hps_io handshake times out; on a slow-corner
    //   board that race can deadlock into a permanent freeze (works on most
    //   units, hangs on one).  The core reset is ALREADY handled internally --
    //   sys_rst_n and mem_rst_n_raw both gate on (~RESET & pll_locked) -- so
    //   the PLL reset was redundant AND uniquely harmful.  Let it free-run:
    //   the OSD/reset path stays alive from FPGA config onward.
    pll pll (
        .refclk            (CLK_50M),
        .rst               (1'b0),
        .outclk_0          (pix_clk),     // 25.0 MHz  (VERA VGA)
        .outclk_1          (aud_mclk),    // 12.5 MHz  (IKAOPM)
        .outclk_2          (cpu_clk),     // 8.0 MHz   (CPU + VIA + I2C)
        .outclk_3          (sdram_clk),   // 100.0 MHz (SDRAM controller)
        .locked            (pll_locked)
    );

    // Phase g: hold the whole core in reset while the HPS streams a ROM in
    // (boot1.rom auto-load at core start, or OSD "Load ROM").  rom_loading is
    // quasi-static (seconds); each domain's 2-FF chain synchronizes release.
    // Same for cart_loading (boot2.rom / OSD "Load Cart" -> SDRAM bank 32+):
    // the CPU restarts after the stream, so the KERNAL's boot-time cartridge
    // check sees the freshly loaded bank 32.
    wire rom_loading, cart_loading;

    // SMC-requested reset (I2C command $02; power-off $01 acts as reset too,
    // there is no PSU to switch here).  smc_x16 emits a 1-cycle cpu_clk
    // pulse; stretch it (~32 us) so every domain's reset synchronizer sees
    // it, and so the SMC itself resets and drops the request.
    wire smc_reset_req, smc_nmi_req, smc_power_off_req;
    wire [7:0] smc_act_led;
    reg  [7:0] smc_rst_stretch = 8'd0;
    always @(posedge cpu_clk)
        if (smc_reset_req | smc_power_off_req) smc_rst_stretch <= 8'd255;
        else if (smc_rst_stretch != 8'd0)      smc_rst_stretch <= smc_rst_stretch - 8'd1;
    wire smc_reset_hold = (smc_rst_stretch != 8'd0);

    // Cart-restore reset hold (jyv 2026-07-07): mounting a .CRT reboots
    // into it, like the F2 cart download does.  crt_restoring comes from
    // cart_backer (sdram domain, quasi-static ~0.5 s level).
    wire crt_restoring;

    // OSD CPU-choice change (status[1] vs the latched cpu02_sel) -> forced
    // system reset; driven next to the CPU instantiation (see u_cpu816).
    wire cpusel_reset_hold;

    // SC0 hot-swap reboot (jyv 2026-07-22): mounting a NEW guest SD while the
    // core runs must REBOOT the X16 so the KERNAL DOS re-mounts the new card.
    // A hot-swap otherwise keeps the old card's cached BPB/FAT ("290 MB free /
    // no files" until a relaunch); deselecting the card alone (vsd_hold) did
    // not clear the DOS mount, so pulse a real reset like Mount Cartridge does.
    // Driven from the sdram domain below (next to vsd_hold), same CDC path as
    // crt_restoring.  mem_reset_n excludes it, so SDRAM (HiRAM + cart) survives.
    wire sd_remount_hold;

    wire sys_rst_n = ~RESET & pll_locked & ~rom_loading & ~cart_loading
                   & ~smc_reset_hold & ~crt_restoring & ~cpusel_reset_hold
                   & ~sd_remount_hold;

    // Memory-side reset: excludes the download holds -- ext_ram_sdram must
    // stay ALIVE (refresh + loader port) while the cart streams into SDRAM
    // and while a ROM download holds the CPU.  The CPU is in reset then, so
    // its bus is quiet (r65c02 parks at $0000, rwn=1 -> no cs into SDRAM).
    wire mem_rst_n_raw = ~RESET & pll_locked;
    reg [1:0] mem_rst_sync = 2'b00;
    always @(posedge cpu_clk or negedge mem_rst_n_raw)
        if (!mem_rst_n_raw) mem_rst_sync <= 2'b00;
        else                mem_rst_sync <= {mem_rst_sync[0], 1'b1};
    wire mem_reset_n = mem_rst_sync[1];

    // ----- Per-domain reset synchronizers -----
    // (VERA needs no external reset: its top has an internal reset_sync off
    //  clk25.  IKAOPM now lives in pix_clk with its own ymrst_sync below;
    //  aud_mclk is unused since the YM clocking fix.)
    reg [1:0] cpu_rst_sync = 2'b00;
    always @(posedge cpu_clk or negedge sys_rst_n)
        if (!sys_rst_n) cpu_rst_sync <= 2'b00;
        else            cpu_rst_sync <= {cpu_rst_sync[0], 1'b1};
    wire cpu_reset_n = cpu_rst_sync[1];

    // ========================================================================
    // HPS_IO  --  minimal config; full integration in Phase e/g
    // ========================================================================
    localparam CONF_STR = {
        // MiSTer parses UART settings from config-string item 1, i.e. the
        // second semicolon-delimited field immediately after the core name.
        "X16;UART2400:4800:9600:19200:38400:57600:115200;-;",
        // jyv 2026-07-07: "SC" (not plain "S") -- Main only remembers and
        // AUTO-REMOUNTS SC entries at core start (user_io.cpp checks 'S','C').
        "SC0,IMG,Mount SD;",      // X16 guest SD = a FAT32 image file on the boot SD
        "SC1,NVR,Mount NVRAM;",   // 512-byte image backing the RTC NVRAM (KERNAL settings)
        "SC2,CRT,Mount Cart;",    // cart image on SC2 -- raw all-RAM (persistent) OR typed .crt (per-bank), per the "Cart mount" toggle below
        "-;",
        "F1,BINROM,Load ROM;",    // ioctl_index = 1; *.bin/*.rom = the SYSTEM ROM (banks 0-15)
        "F2,BINROM,Load Cart;",   // ioctl_index = 2 -> a cart ROM image into bank 32+ (run it)
        "-;",
        "O[1],CPU,65C816,65C02;", // status[1] -> cpu02_sel (latched in reset)
        "O[7:6],CPU Speed,1x (8 MHz),2x (16 MHz),4x (32 MHz);",
        "O[2],MiSTer UART,$9FE0,$9FE8;", // fixed host-UART guest address
        "-;",
        "O[5],VERA2 Bitmap Layer,Off,On;", // status[5] master-enable: SDRAM 640x480 4/8bpp bitmap ($9F60). Moved off O[2] -- that's the serial card's MiSTer UART selector.
        "O[4],Cart mount,RAM,Typed;", // status[4]: how "Mount Cart" (SC2) reads the image -- 0=RAM (raw all-RAM image, every bank 32-255 writable, persistent dirty-sector save-back), 1=Typed (parse the X16 .crt header for per-bank ROM/RAM/NVRAM, volatile)
        "O[3],Cart Banks 32+,By Cart,All RAM;", // status[3]: 0=By Cart (per-bank ROM/RAM from the mount's mask; no cart = ROM), 1=All RAM (force every cart bank 32-255 writable regardless, no cart needed)
        "-;",
        "J1,A,B,X,Y,L,R,Select,Start;",  // SNES pad buttons -> joy[4..11]
        "V,v1.3"
    };

    wire [127:0] status;

    // ========================================================================
    // CPU Speed Selection & Clock Enables (1x = 8 MHz, 2x = 16 MHz, 4x = 32 MHz)
    // ========================================================================
    reg [1:0] speed_cnt = 2'd0;
    always @(posedge cpu_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) speed_cnt <= 2'd0;
        else            speed_cnt <= speed_cnt + 2'd1;
    end

    wire ce_1x = (speed_cnt == 2'd0);          // 8 MHz  (1 out of 4 cycles)
    wire ce_2x = (speed_cnt[0] == 1'b0);       // 16 MHz (2 out of 4 cycles)
    wire ce_4x = 1'b1;                         // 32 MHz (4 out of 4 cycles)

    wire [1:0] cpu_speed_sel = status[7:6];
    wire cpu_ce = (cpu_speed_sel == 2'b00) ? ce_1x :
                  (cpu_speed_sel == 2'b01) ? ce_2x : ce_4x;
    wire [1:0]   buttons;
    wire         forced_scandoubler;
    wire         direct_video;
    wire [10:0]  ps2_key;
    wire [24:0]  ps2_mouse;
    wire [15:0]  ps2_mouse_ext;   // [7:0] wheel delta (signed), [15:8] btns 4/5
    wire [31:0]  joystick_0, joystick_1;
    wire [64:0]  hps_rtc;
    wire         ioctl_download;
    wire         ioctl_wr;
    wire [26:0]  ioctl_addr;
    wire [7:0]   ioctl_dout;
    wire [15:0]  ioctl_index;

    // Virtual SD block interface (HPS-backed disk image) -- one drive (VDNUM=1).
    // jyv 2026-07-07: 3 virtual disks -- slot 0 = guest SD image, slot 1 =
    // RTC NVRAM image (rtl/nvram_backer.sv), slot 2 = cart RAM image
    // (rtl/cart_backer.sv, uses 16 KB multi-block reads -> sd_buff_addr is
    // the full hps_io [13:0]; slots 0/1 use the low 9 bits).
    wire [31:0]  sd_lba[3];
    wire  [5:0]  sd_blk_cnt[3];
    wire  [2:0]  sd_rd;
    wire  [2:0]  sd_wr;
    wire  [2:0]  sd_ack;
    wire [13:0]  sd_buff_addr;
    wire  [7:0]  sd_buff_dout;
    wire  [7:0]  sd_buff_din[3];
    wire         sd_buff_wr;
    wire  [2:0]  img_mounted;
    wire         img_readonly;
    wire [63:0]  img_size;
    assign sd_blk_cnt[0] = 6'd0;          // single 512-byte block per transfer
    assign sd_blk_cnt[1] = 6'd0;          // (sd_blk_cnt[2] driven by u_cart)

    // BUG3 FINAL FIX: hps_io (and sd_card below) run on the 100 MHz clock.
    // At 8 MHz clk_sys the FPGA->HPS readout direction (SAVE) undersampled
    // the HPS bus strobes -- Main re-read stale io_dout words -> repeated
    // bytes -> corrupted sectors (probe-proven: data perfect through the
    // sd_card buffer; io_strobe widths unsample-able at 8 MHz; C64 core on
    // the same MiSTer works at 32 MHz).  Every working core runs hps_io at
    // 32-112 MHz; 96 MHz (BBC) is proven territory for this framework.
    // CDC: ps2_key is 2-FF synced into cpu_clk; the guest SPI wires cross
    // into 100 MHz oversampled (the standard MiSTer guest pattern); all
    // sd_* signals stay inside the 100 MHz domain.
    wire cart_ld_busy;                 // loader FIFO backpressure (sdram_clk)

    hps_io #(.CONF_STR(CONF_STR), .VDNUM(3)) u_hps (
        .clk_sys           (sdram_clk),
        .HPS_BUS           (HPS_BUS),
        .buttons           (buttons),
        .status            (status),
        .forced_scandoubler(forced_scandoubler),
        .direct_video      (direct_video),
        .ps2_key           (ps2_key),
        .ps2_mouse         (ps2_mouse),
        .ps2_mouse_ext     (ps2_mouse_ext),
        .joystick_0        (joystick_0),
        .joystick_1        (joystick_1),
        .RTC               (hps_rtc),
        .ioctl_download    (ioctl_download),
        .ioctl_wr          (ioctl_wr),
        .ioctl_addr        (ioctl_addr),
        .ioctl_dout        (ioctl_dout),
        .ioctl_index       (ioctl_index),
        .ioctl_wait        (cart_ld_busy),

        .img_mounted       (img_mounted),
        .img_readonly      (img_readonly),
        .img_size          (img_size),
        .sd_lba            (sd_lba),
        .sd_blk_cnt        (sd_blk_cnt),
        .sd_rd             (sd_rd),
        .sd_wr             (sd_wr),
        .sd_ack            (sd_ack),
        .sd_buff_addr      (sd_buff_addr),
        .sd_buff_dout      (sd_buff_dout),
        .sd_buff_din       (sd_buff_din),
        .sd_buff_wr        (sd_buff_wr)
    );

    // ========================================================================
    // Guest SD card  --  VERA's SPI SD master <-> HPS virtual block device.
    //   sd_card.sv is an SD-SPI *slave* backed by the FAT32 disk-image file
    //   mounted in the OSD (S0); that image lives on the MiSTer boot SD, so the
    //   X16's LOAD/SAVE persist to the boot card.  (The board's J9 "Micro SD_2"
    //   is on the HPS SDMMC -- HPS-owned, unreachable from FPGA fabric -- so a
    //   direct second-card path is not possible; the image is the way.)
    //   SPI side runs in VERA's pix_clk domain (sck = pix_clk/2); block side
    //   runs in cpu_clk, matching hps_io.
    // ========================================================================
    wire vera_spi_sck, vera_spi_mosi, vera_spi_ssel_n;   // VERA SD SPI master (driven below)
    wire sd_card_miso;
    // The SD SPI is driven by a purpose-built master (rtl/spi_sd_master100.sv,
    // 100 MHz domain, sck = 12.5 MHz -- see its header for the full speed
    // saga).  This replaces VERA's pix_clk spictrl for the SD, whose
    // cross-domain resync of sck/miso was intermittently corrupting bytes.
    // Driven by the u_spimaster instance below (declared here so sd_card
    // can connect to them):
    wire       m_sck, m_mosi, m_sel;   // SPI clock / MOSI / chip-select (1=selected)

    // Keep sd_card DESELECTED until an image is actually mounted (the standard
    // MiSTer idiom, e.g. BBCMicro/ZX-Spectrum: `.ss(sdss | ~vsd_sel)`).  Without
    // this, the ROM's boot-time SD probe runs against sd_card with NO image:
    // init succeeds (sd_card replies to CMD0/8/41/58 regardless), then CMD17
    // wedges read_state in RD_STATE_WAIT_IO (HPS never acks sd_rd) with sd_rd
    // and the prefetch machine left pending.  img_mounted resets NONE of that,
    // so the first post-mount read serves a never-filled prefetch bank -> the
    // CPU receives a valid 0xFE token (a constant) followed by 512 x 0x00, and
    // sd_card rejects every later command except CMD12 (read_state != IDLE).
    // Deselected-until-mounted, the pre-mount probe just times out cleanly
    // (miso = 0xFF = "no card"), state stays IDLE, and the ROM's per-access
    // CMD13 alive-check re-inits the card once the image appears.
    //
    // sd_card lives in the 100 MHz domain with hps_io (BUG3 final fix).
    // jyv 2026-07-07 (SD-speed saga, act 3): the SPI master moved into the
    // SAME 100 MHz domain (rtl/spi_sd_master100.sv, sck = 12.5 MHz = real
    // VERA fast speed), so sck/mosi/ss/miso connect directly -- no
    // per-signal synchronizers, no false-pathed SPI timing.  Only the
    // reset is synced into the domain.
    reg [1:0] hs_rst_sync;
    always @(posedge sdram_clk) hs_rst_sync <= {hs_rst_sync[0], ~cpu_reset_n};

    // jyv 2026-07-15 (RELAUNCH wedge, tester report): SC0 is an AUTO-REMOUNT
    // slot -- on a core relaunch Main re-announces the image BEFORE the CPU
    // boots, so vsd_sel was already 1 when the boot probe ran and the gate
    // above no longer protected it: the probe's abandoned CMD17 wedged
    // read_state exactly as described -- locked boot / garbage first read,
    // "fixed" by remount or CPU-toggle (any reset with the mount settled).
    // FIX: force the card DESELECTED for ~1.3 s after EVERY reset release
    // (vsd_hold), so the boot probe always sees "no card" -- byte-for-byte
    // the HW-proven first-launch path.  DOS"$/F7 comes seconds later, long
    // after the hold expires.  Every reset re-runs the ROM's boot probe
    // (framework, CPU-toggle, SMC, Load ROM), so the hold re-arms on all of
    // them via hs_rst_sync.  Reproduced + fix proven in sim/tb_relaunch.v
    // (run.sh relaunch: FIX=0 wedges/corrupts, FIX=4 reads clean).
    reg        vsd_mnt = 0;
    reg [26:0] vsd_hold = '1;              // 2^27-1 @ 100 MHz = ~1.34 s
    reg [23:0] sd_mnt_rst = 24'd0;         // SC0 hot-mount reboot hold (~0.17 s)
    always @(posedge sdram_clk) begin
        if (img_mounted[0]) vsd_mnt <= |img_size;
        // re-arm on every core reset AND on a fresh SD mount: the SD stays
        // deselected ~1.34 s so the ROM boot probe runs clean before the DOS
        // re-mounts the new card (pairs with the reboot pulse below).
        if (hs_rst_sync[1] || img_mounted[0]) vsd_hold <= '1;
        else if (vsd_hold != 0)               vsd_hold <= vsd_hold - 1'd1;
        // reboot pulse on a fresh mount: hold sys_rst_n low ~0.17 s then release,
        // so the guest reboots and its DOS re-mounts the newly swapped card.
        if (img_mounted[0])       sd_mnt_rst <= 24'hFFFFFF;
        else if (sd_mnt_rst != 0) sd_mnt_rst <= sd_mnt_rst - 24'd1;
    end
    wire   vsd_sel         = vsd_mnt & (vsd_hold == 0);
    assign sd_remount_hold = (sd_mnt_rst != 0);
    // memory-side reset in the sdram domain: stays RELEASED through ROM/cart
    // download holds and the cart-restore hold -- the nvram/cart backers run
    // on this (a sys_rst_n-derived reset would deadlock the restore hold,
    // and the NVRAM restore now completes while the CPU is still held).
    reg [1:0] mem_rstf_sync;
    always @(posedge sdram_clk or negedge mem_rst_n_raw)
        if (!mem_rst_n_raw) mem_rstf_sync <= 2'b00;
        else                mem_rstf_sync <= {mem_rstf_sync[0], 1'b1};
    wire bk_reset_n = mem_rstf_sync[1];
    wire sd_ss_gated = ~m_sel | ~vsd_sel;  // active-low CS, HIGH until mounted

    sd_card u_sdcard (
        .clk_sys      (sdram_clk),
        .reset        (hs_rst_sync[1]),
        .sdhc         (1'b1),
        .img_mounted  (img_mounted[0]),
        .img_size     (img_size),
        .sd_lba       (sd_lba[0]),
        .sd_rd        (sd_rd[0]),
        .sd_wr        (sd_wr[0]),
        .sd_ack       (sd_ack[0]),
        .sd_buff_addr (sd_buff_addr[8:0]),
        .sd_buff_dout (sd_buff_dout),
        .sd_buff_din  (sd_buff_din[0]),
        .sd_buff_wr   (sd_buff_wr),
        .clk_spi      (sdram_clk),         // same domain as clk_sys (hps_io)
        .ss           (sd_ss_gated),
        .sck          (m_sck),
        .mosi         (m_mosi),
        .miso         (sd_card_miso)
    );

    // MISO back to VERA: sd_card_miso is cpu_clk, VERA samples in pix_clk -> 2-FF sync.
    reg [1:0] sd_miso_sync;
    always @(posedge pix_clk) sd_miso_sync <= {sd_miso_sync[0], sd_card_miso};
    wire vera_sd_miso = sd_miso_sync[1];

    // ========================================================================
    // CPU bus signals
    // ========================================================================
    wire [15:0] cpu_a;
    wire [15:0] cpu_pc;
    wire  [7:0] cpu_di;
    wire  [7:0] cpu_do;
    wire        cpu_rwn;
    wire        cpu_sync;
    wire        cpu_rdy;   // forward-decl, driven by VERA-read stall

    // VERA IRQ network (sync to cpu_clk)
    wire vera_irq_n;
    reg  [1:0] vera_irq_sync = 2'b11;
    always @(posedge cpu_clk or negedge cpu_reset_n)
        if (!cpu_reset_n) vera_irq_sync <= 2'b11;
        else              vera_irq_sync <= {vera_irq_sync[0], vera_irq_n};
    wire vera_irq_n_sync = vera_irq_sync[1];

    wire via1_irq_n, via2_irq_n, ym_irq_n;   // forward-decl (driven below)

    // YM2151 IRQ (timers) lives in the aud_mclk domain -> 2-FF sync.
    reg [1:0] ym_irq_sync = 2'b11;
    always @(posedge cpu_clk or negedge cpu_reset_n)
        if (!cpu_reset_n) ym_irq_sync <= 2'b11;
        else              ym_irq_sync <= {ym_irq_sync[0], ym_irq_n};
    wire ym_irq_n_sync = ym_irq_sync[1];

    // Full X16 IRQ network: VERA | VIA1 | VIA2 | YM2151 (active-low AND).
    // via*_irq_n are already cpu_clk-synchronous.
    wire cpu_irq_n = vera_irq_n_sync & via1_irq_n & via2_irq_n & ym_irq_n_sync;

    // SMC NMI request (I2C command $03) -> stretched low pulse on the CPU's
    // edge-sensitive NMI input.
    reg [3:0] smc_nmi_stretch = 4'd0;
    always @(posedge cpu_clk or negedge cpu_reset_n)
        if (!cpu_reset_n)                   smc_nmi_stretch <= 4'd0;
        else if (smc_nmi_req)               smc_nmi_stretch <= 4'd15;
        else if (smc_nmi_stretch != 4'd0)   smc_nmi_stretch <= smc_nmi_stretch - 4'd1;
    wire cpu_nmi_n = (smc_nmi_stretch == 4'd0);

    // 65C816 branch: P65C816 (rtl/cpu/65C816_x16, the X16-fixed copy) as a
    // drop-in for r65c02_wrap.  The R49 ROM detects the '816 and runs NATIVE
    // mode with the c816 interrupt paths -- both the hooked-cinv trampoline
    // AND the default irq_emulated_impl path are sim-proven clean under
    // stalls (sim816/run.sh nat + def), and the full bank/cart/ramtas suite
    // passes on this CPU (sim/run.sh bank816).  Unlike the r65c02, RDY_IN is
    // a global clock enable (writes DO stall) -- ext_ram_sdram's ready has
    // the `| we` term to keep write cycles flowing into its FIFO.
    // CPU SELECT: uncomment the define to build the r65c02 (65C02) variant
    // from this branch -- same fixed infrastructure, only the CPU differs
    // (the ideal control build for '816-vs-'02 bug comparisons).
//`define CPU_R65C02

    wire cpu_vpb;
    wire cpu_bus_valid;
`ifdef CPU_R65C02
    // jyv 2026-07-07: WAI/STP shim -- r65c02_tc executes $CB/$DB as 1-cycle
    // NOPs (blows through), so X16 software WAI-pacing on VSYNC breaks.  The
    // shim freezes the CPU on the WAI fetch until IRQ/NMI (STP: until reset)
    // and substitutes NOP for the opcode.  The P65C816 build (default) has
    // both natively.  NOTE: r65c02_wrap's `sync` is LOW on the opcode-fetch
    // cycle (bus-trace proven in sim/tb_wai.v), hence the inversion.
    wire [7:0] cpu_di_wai;
    wire       cpu_rdy_wai;
    wai_shim u_wai_shim (
        .sync   (~cpu_sync),
        .din_in (cpu_di),
        .irq_n  (cpu_irq_n),
        .nmi_n  (cpu_nmi_n),
        .rdy_in (cpu_rdy & cpu_ce),
        .din_out(cpu_di_wai),
        .rdy_out(cpu_rdy_wai)
    );
    r65c02_wrap u_cpu (
        .clk    (cpu_clk),
        .enable (cpu_rdy_wai),
        .res_n  (cpu_reset_n),
        .irq_n  (cpu_irq_n),
        .nmi_n  (cpu_nmi_n),
        .rdy    (1'b1),
        .r_w_n  (cpu_rwn),
        .sync   (cpu_sync),
        .addr   (cpu_a),
        .din    (cpu_di_wai),
        .dout   (cpu_do),
        .pc     (cpu_pc)
    );
    // no VPB pin (its 6502-mode RAM stubs re-bank themselves); every cycle
    // is a valid bus cycle
    assign cpu_vpb       = 1'b1;
    assign cpu_bus_valid = 1'b1;
    assign cpusel_reset_hold = 1'b0;   // single-CPU debug build: no OSD switch
`else
    // ------------------------------------------------------------------
    // Runtime CPU choice (jyv 2026-07-11): OSD "CPU" = status[1]
    //   0 = 65C816 (P65C816, default -- the shipping CPU)
    //   1 = 65C02  (MJoergen cpu_65c02, rtl/cpu/mj65c02 -- a REAL 65C02,
    //       Klaus-Dormann-validated; sim-proven on tb_wai/tb_bank/tb_fullboot)
    // BOTH cores are instantiated; the unselected one is held in reset and
    // muxed out.  The selection is latched (cpu02_sel) only while the core
    // is in reset: changing it in the OSD raises cpusel_reset_hold (same
    // stretch pattern as an SMC reset), the system resets, the latch takes
    // the new value, and the chosen CPU cold-starts from its vector.
    // cpu02_sel is quasi-static, so the output muxes add no comb hazards.
    // ------------------------------------------------------------------
    reg [1:0] cpu02_req_sync = 2'b00;   // status is hps-side; 2FF into cpu_clk
    always @(posedge cpu_clk) cpu02_req_sync <= {cpu02_req_sync[0], status[1]};

    reg       cpu02_sel      = 1'b0;
    reg [7:0] cpusel_stretch = 8'd0;
    always @(posedge cpu_clk) begin
        if (cpu02_req_sync[1] != cpu02_sel) cpusel_stretch <= 8'd255;
        else if (cpusel_stretch != 8'd0)    cpusel_stretch <= cpusel_stretch - 8'd1;
        if (!cpu_reset_n) cpu02_sel <= cpu02_req_sync[1];
    end
    assign cpusel_reset_hold = (cpusel_stretch != 8'd0);

    wire [15:0] a816, pc816;
    wire  [7:0] do816;
    wire        rwn816, sync816, vpb816, bv816;
    p65c816_wrap u_cpu816 (
        .clk      (cpu_clk),
        .enable   (cpu_rdy & cpu_ce),
        .res_n    (cpu_reset_n & ~cpu02_sel),
        .irq_n    (cpu_irq_n),
        .nmi_n    (cpu_nmi_n),
        .rdy      (1'b1),
        .r_w_n    (rwn816),
        .sync     (sync816),
        .addr     (a816),
        .din      (cpu_di),
        .dout     (do816),
        .pc       (pc816),
        .emu_mode (),                  // observability only
        .i_flag   (),
        .vpb      (vpb816),            // vector pull -> ROM bank latch = 0
        .bus_valid(bv816)              // VDA|VPA: qualify all chip selects
    );

    // mj65c02: Rockwell set, no WAI/STP -- wai_shim provides them (purely
    // combinational, so no stale state while the CPU sits unselected in
    // reset).  Its sync is HIGH on the fetch cycle: no inversion, unlike
    // r65c02_wrap above.  ce_i freezes writes too ('816 RDY semantics), and
    // must stay active during reset (forced-BRK reset) -- it is, cpu_rdy
    // idles high.
    wire [15:0] a02, pc02;
    wire  [7:0] do02, di02_wai;
    wire        rwn02, sync02, bv02, rdy02_wai;
    wai_shim u_wai02 (
        .sync   (sync02),
        .din_in (cpu_di),
        .irq_n  (cpu_irq_n),
        .nmi_n  (cpu_nmi_n),
        .rdy_in (cpu_rdy & cpu_ce),
        .din_out(di02_wai),
        .rdy_out(rdy02_wai)
    );
    mj65c02_wrap u_cpu02 (
        .clk      (cpu_clk),
        .enable   (rdy02_wai),
        .res_n    (cpu_reset_n & cpu02_sel),
        .irq_n    (cpu_irq_n),
        .nmi_n    (cpu_nmi_n),
        .rdy      (1'b1),
        .r_w_n    (rwn02),
        .sync     (sync02),
        .addr     (a02),
        .din      (di02_wai),
        .dout     (do02),
        .pc       (pc02),
        .bus_valid(bv02)               // rd_en|wr_en: internal cycles gated
    );

    assign cpu_a         = cpu02_sel ? a02    : a816;
    assign cpu_do        = cpu02_sel ? do02   : do816;
    assign cpu_rwn       = cpu02_sel ? rwn02  : rwn816;
    assign cpu_sync      = cpu02_sel ? sync02 : sync816;
    assign cpu_pc        = cpu02_sel ? pc02   : pc816;
    assign cpu_vpb       = cpu02_sel ? 1'b1   : vpb816;  // mj: no VPB pin
    assign cpu_bus_valid = cpu02_sel ? bv02   : bv816;
`endif

    // VPB (X16 R44+ platform requirement, x16-rom README/release notes):
    // an interrupt vector pull SETS the hardware ROM bank latch to 0; the
    // lowram shadow at $0001 keeps the old value for the handler to save.
    // Without this, the c816 KERNAL RAM stub ($038B: JMP $F80A = a bank-0
    // ROM address) executes from whatever ROM bank is live at IRQ time --
    // e.g. BANNEX's $AA filler mid-jsrfar -- and the machine wanders off
    // SEI'd forever = the splash-screen freeze.  (The r65c02's 6502-mode
    // stub does stz rom_bank itself before touching ROM, which is why the
    // r65c02 core never needed this.)

    // ========================================================================
    // Address decode
    // ========================================================================
    // All chip selects are qualified with dec_valid: on the '816's internal
    // cycles A_OUT carries in-flight address math -- ghost addresses that
    // must not reach IO with read side effects (VERA data-port
    // auto-increment, VIA flag clears) nor start SDRAM accesses.
    // WRITE cycles are exempt (|~cpu_rwn): WE is only asserted on true
    // write cycles, and dropping a write whose VA flag mis-synthesizes is
    // catastrophic (the '816 STA abs,X writes vanished on silicon -> the
    // check_rom boot crash).  Ghost READS remain fully gated.
    // (The 65C02 drives a valid bus cycle every clock: constant 1.)
    wire dec_valid = cpu_bus_valid | ~cpu_rwn;
    wire kernal_cs = dec_valid & (cpu_a[15:14] == 2'b11);            // $C000-$FFFF
    wire io_cs     = dec_valid & (cpu_a[15:8]  == 8'h9F);            // $9F00-$9FFF
    wire vera_cs   = dec_valid & (cpu_a[15:5]  == 11'b10011111001);  // $9F20-$9F3F
    wire ym_cs     = dec_valid & (cpu_a[15:4]  == 12'h9F4);          // $9F40-$9F4F
    wire via1_cs   = dec_valid & (cpu_a[15:4]  == 12'h9F0);          // $9F00-$9F0F
    wire via2_cs   = dec_valid & (cpu_a[15:4]  == 12'h9F1);          // $9F10-$9F1F
    // TexElec's dual-UART serial/network card defaults to IO7-Low and exposes
    // two independent 8-byte UART windows at $9FE0 and $9FE8.
    wire serial0_cs = dec_valid & (cpu_a[15:3] == 13'h13FC);       // $9FE0-$9FE7
    wire serial1_cs = dec_valid & (cpu_a[15:3] == 13'h13FD);       // $9FE8-$9FEF
    wire bmp_cs    = dec_valid & (cpu_a[15:4]  == 12'h9F6);          // $9F60-$9F6F bitmap
    wire hi_ram_cs = dec_valid & (cpu_a[15:13] == 3'b101);           // $A000-$BFFF
    wire lowram_cs = dec_valid & ~(cpu_a[15:14] == 2'b11)
                   & ~(cpu_a[15:8] == 8'h9F) & ~(cpu_a[15:13] == 3'b101); // $0000-$9EFF

    // ========================================================================
    // ROM_BANK / RAM_BANK registers
    // ========================================================================
    reg [7:0] rom_bank_r = 8'h0;          // full 256 ROM banks (32-255 = cart, r/w)
    reg [7:0] ram_bank_r = 8'h0;          // full 256 RAM banks (HiRAM via SDRAM)
    always @(posedge cpu_clk or negedge cpu_reset_n) begin
        if (!cpu_reset_n) begin
            rom_bank_r <= 8'h0;
            ram_bank_r <= 8'h0;
        end else if (!cpu_vpb) begin
            rom_bank_r <= 8'h00;     // vector pull -> KERNAL bank (see u_cpu)
        end else if (lowram_cs && ~cpu_rwn) begin
            if (cpu_a == 16'h0001) rom_bank_r <= cpu_do[7:0];
            if (cpu_a == 16'h0000) ram_bank_r <= cpu_do[7:0];
        end
    end

    // Vector reads always come from ROM bank 0:
    //   $FFFA-$FFFF  emulation-mode vectors (NMI/RESET/IRQ)
    //   $FFE4-$FFEF  65C816 NATIVE-mode vectors (COP/BRK/ABORT/NMI/IRQ)
    // (Every official ROM bank replicates the vector table, so this is a
    // no-op for ROM banks; it only pins down what a cart bank sees.)
    // VECTOR forcing to ROM bank 0:
    // * '816: by the VPB pin ONLY -- NOT by address.  An address-based match
    //   on $FFE4-$FFEF (the native vector range) also catches PROGRAM
    //   FETCHES there: that range doubles as the KERNAL API jump table
    //   (GETIN $FFE4 / CLALL $FFE7 / UDTIM $FFEA / SCREEN $FFED), so a
    //   `jsr $FFED` from any non-KERNAL bank fetched bank 0's JMP opcode,
    //   then continued in the caller's bank -> wild jump (the splash's
    //   SCREEN call from BANNEX = the post-check_rom boot crash).  VPB is
    //   asserted exclusively during true vector reads (emulation $FFFA-$FFFF
    //   AND native $FFE4-$FFEF) -- exactly the real X16's HW behavior.
    // * r65c02: no VPB pin -> the proven address-based match on the
    //   emulation vectors only ($FFFA-$FFFF minus the $FFF8/$FFF9 bytes),
    //   same as main.  The API-table hazard doesn't exist there ($FFE4+
    //   fetches are never forced).
`ifdef CPU_R65C02
    wire vector_fetch        = cpu_rwn & (cpu_a[15:3] == 13'b1111_1111_1111_1)
                                       & (cpu_a[2:0]  != 3'b000)
                                       & (cpu_a[2:0]  != 3'b001);
`else
    // dual-CPU build: '816 by VPB; mj 65C02 (no VPB pin) by the same
    // address match as the r65c02 above -- only $FFFA-$FFFF, so the
    // $FFE4+ KERNAL-API-table hazard cannot arise.  cpu02_sel is
    // quasi-static and this feeds only the DATA-side bank mux (not the
    // rdy cone), so the BUG2 comb-loop class stays impossible.
    wire vector_fetch        = cpu02_sel
                             ? (cpu_rwn & (cpu_a[15:3] == 13'b1111_1111_1111_1)
                                        & (cpu_a[2:0]  != 3'b000)
                                        & (cpu_a[2:0]  != 3'b001))
                             : ~cpu_vpb;
`endif
    wire [7:0] effective_rom_bank = vector_fetch ? 8'h00 : rom_bank_r;

    // $C000-$FFFF window routing by ROM bank (matches X16 hardware layout):
    //   banks   0-15  -> on-chip BRAM system ROM (rom_banks.sv, read-only)
    //   banks  16-31  -> unpopulated system-ROM space: reads $FF (empty
    //                    flash), writes ignored
    //   banks  32-255 -> CARTRIDGE space, READ/WRITE, backed by SDRAM
    //                    (software may treat a bank as ROM or RAM)
    //
    // cart_sel (DATA mux) honors the vector-page redirect; cart_cs (the SDRAM
    // chip-select, which feeds the cpu_rdy stall cone) deliberately does NOT:
    // vector_fetch is f(cpu_a[15:3], cpu_rwn), and adding it to the rdy cone
    // recreates the comb-loop class this core was bitten by (BUG2).  A vector
    // fetch with a cart bank selected just runs one harmless SDRAM read whose
    // data the mux ignores in favor of ROM bank 0.
    wire rom_open_sel = (effective_rom_bank[7:4] == 4'h1);   // (banks 0-15 = the mux default)
    wire cart_sel     = |effective_rom_bank[7:5];            // data-mux steer
    wire cart_cs      = kernal_cs & (|rom_bank_r[7:5]);      // SDRAM cs / stall cone

    // ========================================================================
    // ROM banks ($C000-$FFFF)
    // ========================================================================
    wire [7:0] rom_data;
    // Phase g: the HPS ROM loader.  ioctl index 1 = the CONF_STR "F1" slot,
    // served either automatically at core start (put rom.bin as boot1.rom in
    // the core's folder on the MiSTer SD) or manually via OSD "Load ROM".
    // The whole core sits in reset during the stream (rom_loading above), so
    // the new ROM boots cleanly the moment the download ends.  The baked-in
    // rom.hex remains the default when no file is provided.
    // ioctl_index encoding (MiSTer Main): OSD file picks send
    // {ext_index, slot[5:0]} -- "Load ROM" = 0x0001 for a *.bin pick and
    // 0x0041 for a *.rom pick -- while the bootN.rom auto-load loop sends
    // N<<6 (boot1.rom = 0x0040, boot2.rom = 0x0080; slot bits = 0).  Match
    // the slot bits for OSD picks plus the exact N<<6 codes for boot files.
    // (The old [7:0]==N compare missed both boot files and *.rom picks.)
    assign rom_loading = ioctl_download & ((ioctl_index[5:0] == 6'd1) |
                                           (ioctl_index      == 16'h0040));
    // 16-byte "CX16 CARTRIDGE\r\n" cartridge magic -- same bytes cart_backer.sv
    // checks; used just below to reject a cartridge dropped on the ROM slot.
    function [7:0] rom_magic(input [3:0] i);
        case (i)
            4'd0: rom_magic=8'h43; 4'd1: rom_magic=8'h58;   // C X
            4'd2: rom_magic=8'h31; 4'd3: rom_magic=8'h36;   // 1 6
            4'd4: rom_magic=8'h20; 4'd5: rom_magic=8'h43;   // _ C
            4'd6: rom_magic=8'h41; 4'd7: rom_magic=8'h52;   // A R
            4'd8: rom_magic=8'h54; 4'd9: rom_magic=8'h52;   // T R
            4'd10: rom_magic=8'h49; 4'd11: rom_magic=8'h44; // I D
            4'd12: rom_magic=8'h47; 4'd13: rom_magic=8'h45; // G E
            4'd14: rom_magic=8'h0D; 4'd15: rom_magic=8'h0A; // CR LF
            default: rom_magic=8'h00;
        endcase
    endfunction

    // Load-ROM cart guard: a "Load ROM" file that starts with the "CX16
    // CARTRIDGE" magic is a cartridge on the ROM slot -- suppress every write
    // that continues the magic prefix, so such a file writes NOTHING and the
    // KERNAL banks are never overwritten (the old ROM stays intact and reboots).
    // A real system ROM does not start with the magic, so its writes are wholly
    // untouched -- the guard only engages while the incoming header == the magic.
    reg  rom_hdr_match = 1'b1;                     // header prefix still == magic
    reg  rom_reject    = 1'b0;                     // full magic seen -> block all
    wire rom_hdr_byte  = rom_loading & ioctl_wr & (ioctl_addr[26:4] == 23'd0);  // offset 0-15
    wire rom_hdr_hit   = rom_hdr_match & (ioctl_dout == rom_magic(ioctl_addr[3:0]));
    always @(posedge sdram_clk) begin
        if (!rom_loading) begin                   // re-arm between loads
            rom_hdr_match <= 1'b1;
            rom_reject    <= 1'b0;
        end else if (rom_hdr_byte) begin
            rom_hdr_match <= rom_hdr_hit;          // stays 1 while matching, 0 on mismatch
            if (ioctl_addr[3:0] == 4'd15 && rom_hdr_hit) rom_reject <= 1'b1;  // all 16 -> cart
        end
    end
    wire   rom_wr_en   = ioctl_wr & rom_loading & ~ioctl_addr[26] & (ioctl_addr[25:18] == 8'd0)
                       & ~rom_reject                      // confirmed cart -> block everything
                       & ~(rom_hdr_byte & rom_hdr_hit);   // magic-prefix byte -> don't write it

    // Cart loader (OSD "Load Cart" -> ioctl index 2, or boot2.rom auto-load):
    // the file streams RAW into the SDRAM cart space starting at ROM bank 32
    // ($C000 of bank 32 = file offset 0 = where the KERNAL looks for the
    // cartridge signature).  Max 3.5 MB = 224 banks; excess bytes are dropped.
    // Everything here is in the sdram_clk domain (= hps_io clk_sys).
    assign cart_loading = ioctl_download & ((ioctl_index[5:0] == 6'd2) |
                                            (ioctl_index      == 16'h0080));
    wire   cart_wr_en   = ioctl_wr & cart_loading & ~ioctl_addr[26]
                        & (ioctl_addr[25:22] == 4'd0)
                        & (ioctl_addr[21:0]  <  22'h380000);
    wire [24:0] cart_ld_addr = 25'h480000 + {3'd0, ioctl_addr[21:0]};

    // cart_backer <-> ext_ram_sdram wiring (jyv 2026-07-07, cart save-back)
    wire        crt_ld_wr;
    wire [24:0] crt_ld_addr;
    wire  [7:0] crt_ld_data;
    wire        crt_bk_rd, crt_bk_ack;
    wire [24:0] crt_bk_addr;
    wire  [7:0] crt_bk_rdata;
    wire        crt_wr_snoop;
    wire [24:0] crt_wr_snoop_addr;

    rom_banks u_rom (
        .clk     (cpu_clk),
        .bank    (effective_rom_bank[3:0]),   // BRAM ROM = banks 0-15 only
        .addr    (cpu_a[13:0]),
        .rd_data (rom_data),
        .wr_clk  (sdram_clk),              // hps_io / ioctl clock domain
        .wr_en   (rom_wr_en),              // clamped to the 256 KB array
        .wr_addr (ioctl_addr[17:0]),
        .wr_data (ioctl_dout)
    );

    // ========================================================================
    // LowRAM ($0000-$9EFF)  -- 40 KB M10K BRAM
    // ========================================================================
    wire [7:0] lowram_data;
    wire       lowram_we = lowram_cs & ~cpu_rwn;
    lowram_bram u_lowram (
        .clk     (cpu_clk),
        .addr    (cpu_a),
        .cs      (lowram_cs),
        .we      (lowram_we),
        .wr_data (cpu_do),
        .rd_data (lowram_data)
    );

    // ========================================================================
    // HiRAM ($A000-$BFFF) + cart banks ($C000-$FFFF, ROM banks 32-255)
    // via SDRAM  (Phase f + cart extension)
    // ========================================================================
    wire [7:0] ext_ram_data;
    wire       hiram_we = hi_ram_cs & ~cpu_rwn;

    // ---- HYBRID HiRAM: banks 0..BRAM_BANKS-1 live in fast/reliable on-chip
    //   BRAM (single-cycle, no stall), banks BRAM_BANKS..255 live in SDRAM.
    //   Bank 1 holds the KERNAL keymap, so the keyboard is immune to the SDRAM
    //   access path.  RAM detection still sees 256 distinct banks -> 2048K.
    //   Each HiRAM access is routed to exactly ONE backing store by the bank.
    localparam [7:0] BRAM_BANKS = 8'd2;                  // banks 0,1 -> BRAM
    wire       bram_sel = (ram_bank_r < BRAM_BANKS);
    wire [7:0] sdram_rd,  bram_rd;
    wire       sdram_rdy, bram_rdy;
    assign ext_ram_data = bram_sel ? bram_rd  : sdram_rd;

    // One SDRAM controller serves both windows; the address mux places
    // HiRAM at 0x000000-0x1FFFFF and cart banks at 0x400000 + bank*16K
    // (banks 32-255 -> 0x480000-0x7FFFFF).  cart reads AND writes both go
    // to SDRAM -- the cart space is RAM-semantics (software decides whether
    // a bank holds "ROM" images or data).
    // ---- bitmap layer interconnect (driven by u_bmp_regs / u_bmp_engine below) ----
    wire        bmp_master_en = status[5];   // O[5] (O[2] is the serial UART selector)
    wire        bmp_sel = bmp_cs & bmp_master_en;   // $9F6x selected AND OSD-enabled
                                                    // (OSD off -> device inert = stock)
    wire  [7:0] bmp_do;
    wire        bmp_enable;
    wire  [1:0] bmp_mode;
    wire        bmp_passthru;            // CTRL[3]: VERA opaque pixels over the bitmap
    wire        bmp_fb_wr, bmp_fb_rd;    // $9F65 write / read cycle (RAW, OSD-gated)
    wire [24:0] bmp_fb_addr;             // planar byte address for the access
    wire        bmp_pal_we;
    wire  [7:0] bmp_pal_idx;
    wire [11:0] bmp_pal_data;
    wire        bmp_fb_go,  bmp_fb_valid, bmp_fb_done;
    wire [23:0] bmp_fb_base;
    wire [10:0] bmp_fb_len;
    wire [15:0] bmp_fb_word;
    wire  [3:0] bmp_r, bmp_g, bmp_b;
    wire        bmp_active;
    wire        bmp_blit_start, bmp_blit_done;   // SDRAM->SDRAM save-under copy
    wire [19:0] bmp_blit_src, bmp_blit_dst, bmp_blit_len;   // 20-bit (1 MB fb space)

    // Framebuffer DATA ($9F65) rides the ext_ram_sdram CPU port at the planar
    // address: writes through the write FIFO, reads through the read path (which
    // stalls the CPU for the SDRAM latency, like a HiRAM read -- this is what
    // makes GUI save-under possible).  bmp_fb_wr/rd are RAW decodes (no cpu_rdy)
    // so they cannot close a comb loop through `ready`; u_bmp_regs advances the
    // pointer on the cpu_rdy commit.  Both are OSD-gated via the regs' cs.
    wire        ext_sdram_cs   = (hi_ram_cs & ~bram_sel) | cart_cs | bmp_fb_wr | bmp_fb_rd;
    // Cart write control ($C000-$FFFF banks 32-255):
    //   O[3]=0 "By Cart"  -> per-bank from the mounted .crt's bank_info table:
    //                        ROM banks drop CPU writes, RAM/NVRAM banks accept
    //                        them (cart_wmask, parsed by u_cart).  No cart = 0.
    //   O[3]=1 "All RAM"  -> force every cart bank writable (scratch RAM, no cart).
    // Only the CPU write path is gated; the loader (Load Cart / Mount Cartridge)
    // uses the separate ld_/bk_ ports.  cart_wmask is loaded while the CPU is in
    // reset (quasi-static), and feeds `we` ONLY -- never cpu_rdy (no BUG2 loop).
    wire [255:0] cart_wmask;                                    // driven by u_cart
    wire         cart_bank_wr  = cart_wmask[rom_bank_r];        // this bank writable?
    wire         cart_writable = status[3] | cart_bank_wr;      // O[3] All-RAM override
    wire         ext_sdram_we  = (ext_sdram_cs & ~cpu_rwn)      // read: cpu_rwn=1 -> we=0
                               & ~(cart_cs & ~cart_writable);   // ROM bank -> drop write
    wire [24:0] ext_sdram_addr = (bmp_fb_wr | bmp_fb_rd) ? bmp_fb_addr
                               : cart_cs                 ? {3'b001, rom_bank_r, cpu_a[13:0]}
                               :                           {4'd0,   ram_bank_r, cpu_a[12:0]};

    ext_ram_sdram #(.FB_BASE_WORD(24'h800000)) u_hiram (
        .clk        (cpu_clk),
        .sdram_clk  (sdram_clk),
        .reset_n    (mem_reset_n),           // stays alive during ROM/cart downloads
        .cs         (ext_sdram_cs),          // HiRAM banks 2-255 or cart banks 32-255
        .we         (ext_sdram_we),
        .byte_addr  (ext_sdram_addr),
        .wr_data    (cpu_do),
        // loader port: ioctl "Load Cart" stream OR the cart_backer restore
        // (never concurrent: ioctl holds the system reset, the backer only
        // runs on mount events)
        .ld_wr      (cart_wr_en | crt_ld_wr),
        .ld_addr    (cart_wr_en ? cart_ld_addr : crt_ld_addr),
        .ld_data    (cart_wr_en ? ioctl_dout   : crt_ld_data),

        // cart save-back taps (jyv 2026-07-07, rtl/cart_backer.sv)
        .bk_rd          (crt_bk_rd),
        .bk_addr        (crt_bk_addr),
        .bk_rdata       (crt_bk_rdata),
        .bk_ack         (crt_bk_ack),
        .wr_snoop       (crt_wr_snoop),
        .wr_snoop_addr  (crt_wr_snoop_addr),

        // Framebuffer streaming read port -- driven by u_bmp_engine (OSD-gated;
        // idle when the bitmap layer is disabled -> zero SDRAM impact).
        .fb_go      (bmp_fb_go),
        .fb_base    (bmp_fb_base),
        .fb_len     (bmp_fb_len),
        .fb_valid   (bmp_fb_valid),
        .fb_word    (bmp_fb_word),
        .fb_done    (bmp_fb_done),
        .blit_start (bmp_blit_start),
        .blit_src   (bmp_blit_src),
        .blit_dst   (bmp_blit_dst),
        .blit_len   (bmp_blit_len),
        .blit_done  (bmp_blit_done),

        .ld_busy    (cart_ld_busy),
        .rd_data    (sdram_rd),
        .ready      (sdram_rdy),
        .SDRAM_A    (SDRAM_A),
        .SDRAM_DQ   (SDRAM_DQ),
        .SDRAM_BA   (SDRAM_BA),
        .SDRAM_nCS  (SDRAM_nCS),
        .SDRAM_nWE  (SDRAM_nWE),
        .SDRAM_nRAS (SDRAM_nRAS),
        .SDRAM_nCAS (SDRAM_nCAS),
        .SDRAM_CKE  (SDRAM_CKE),
        .SDRAM_CLK  (SDRAM_CLK),
        .SDRAM_DQML (SDRAM_DQML),
        .SDRAM_DQMH (SDRAM_DQMH)
    );

    // BRAM HiRAM instance -- serves banks 0,1 (keymap is bank 1) in the hybrid.
    ext_ram_bram u_hiram_bram (
        .clk     (cpu_clk),
        .cs      (hi_ram_cs & bram_sel),   // BRAM serves banks 0,1 only
        .we      (hiram_we),
        .bank    (ram_bank_r),
        .addr    (cpu_a[12:0]),
        .wr_data (cpu_do),
        .rd_data (bram_rd),
        .ready   (bram_rdy)
    );

    // ========================================================================
    // VERA  --  CPU bus pipeline + register-bank glue
    // ========================================================================
    // Mirrors the 4-cycle write / 2-cycle read-stall pipeline in C5G_x16.sv
    // (lines 385-446).  The pipeline absorbs the cpu_clk vs pix_clk skew so
    // VERA sees stable address/data/strobe for the full transaction.

    // NOTE: vera_access must NOT be gated by cpu_rdy (gating kills vera_access
    // during the read stall), and the write/read strobes must use the LATCHED
    // q-flags, not the live cpu_rwn -- a 6502 write is 1 cycle, so cpu_rwn is
    // already back high during the delayed q1/q2 window.  Restored verbatim
    // from C5G_x16.sv lines 381-446 (the earlier MiSTer simplification here was
    // the bug that left VERA never seeing a write strobe -> no video).
    wire        vera_access      = vera_cs;
    wire        vera_write       = vera_access & ~cpu_rwn;
    wire        vera_read        = vera_access &  cpu_rwn;

    // (only the q1/q2 stages of access/read are consumed -- the q3 stage is
    //  needed just for the WRITE strobe stretch and the data-hold cpu_do_q3)
    reg         vera_access_q1, vera_access_q2;
    reg         vera_write_q1,  vera_write_q2,  vera_write_q3;
    reg         vera_read_q1,   vera_read_q2;
    reg  [7:0]  cpu_do_q1,      cpu_do_q2,      cpu_do_q3;
    reg  [4:0]  cpu_a5_q1;

    always @(posedge cpu_clk or negedge cpu_reset_n) begin
        if (!cpu_reset_n) begin
            vera_access_q1 <= 1'b0; vera_access_q2 <= 1'b0;
            vera_write_q1  <= 1'b0; vera_write_q2  <= 1'b0; vera_write_q3  <= 1'b0;
            vera_read_q1   <= 1'b0; vera_read_q2   <= 1'b0;
            cpu_do_q1      <= 8'h00; cpu_do_q2     <= 8'h00; cpu_do_q3     <= 8'h00;
            cpu_a5_q1      <= 5'h00;
        end else begin
            vera_access_q1 <= vera_access;
            vera_access_q2 <= vera_access_q1;
            vera_write_q1  <= vera_write;
            vera_write_q2  <= vera_write_q1;
            vera_write_q3  <= vera_write_q2;
            vera_read_q1   <= vera_read;
            vera_read_q2   <= vera_read_q1;
            if (vera_access) cpu_a5_q1 <= cpu_a[4:0];
            if (vera_write)  cpu_do_q1 <= cpu_do;
            cpu_do_q2      <= cpu_do_q1;
            cpu_do_q3      <= cpu_do_q2;
        end
    end

    wire vera_access_bw = vera_access_q1 | vera_access_q2;
    wire vera_write_bw  = vera_write_q1  | vera_write_q2;
    wire vera_read_bw   = vera_read_q1   | vera_read_q2;

    // Read stall -- pause CPU until VERA returns data
    reg [1:0] vera_read_stall = 2'h0;
    always @(posedge cpu_clk or negedge cpu_reset_n) begin
        if (!cpu_reset_n)         vera_read_stall <= 2'h0;
        else if (vera_read) begin
            if (vera_read_stall != 2'd3) vera_read_stall <= vera_read_stall + 2'd1;
        end else                  vera_read_stall <= 2'h0;
    end
    // spi_stall (from the cpu_clk SD-SPI master below) holds the CPU during a $9F3E
    // access while an SPI transfer is in flight, so R49 never drops or grabs a stale
    // byte.  It is the ONLY extra term on cpu_rdy for the SD path now.
    wire spi_stall;   // from the SD SPI master below (holds $9F3E reads only)
    // each backing store's ready idles at 1 when unselected, so a plain AND
    // combines HiRAM-BRAM, HiRAM-SDRAM and cart-SDRAM stalls
    wire mem_ready = bram_rdy & sdram_rdy;
    wire cpu_rdy_base = (~vera_read | (vera_read_stall >= 2'd2)) & mem_ready;
    assign cpu_rdy = cpu_rdy_base & ~spi_stall;   // spi_stall: hold CPU during a $9F3E xfer

    wire [4:0] vera_a_out = vera_access ? cpu_a[4:0] : cpu_a5_q1;
    wire [7:0] vera_d_out = vera_write    ? cpu_do    :
                            vera_write_q1 ? cpu_do_q1 :
                            vera_write_q2 ? cpu_do_q2 :
                                            cpu_do_q3;

    // Bidirectional data bus tee
    wire [7:0] vera_extbus_d;
    wire       vera_d_drive  = vera_write | vera_write_q1 | vera_write_q2 | vera_write_q3;
    assign vera_extbus_d = vera_d_drive ? vera_d_out : 8'hZZ;

    // ----- VERA outputs -----
    wire [3:0] vera_r, vera_g, vera_b;
    wire       vera_hs, vera_vs, vera_de;
    wire       vera_opaque;          // 1 = VERA drew a sprite/layer pixel here
    wire       vera_audio_lrck, vera_audio_bck, vera_audio_data;

    top u_vera (
        .clk25           (pix_clk),

        .extbus_cs_n     (~vera_access_bw),
        .extbus_rd_n     (~vera_read_bw),
        .extbus_wr_n     (~vera_write_bw),
        .extbus_a        (vera_a_out),
        .extbus_d        (vera_extbus_d),
        .extbus_irq_n    (vera_irq_n),

        .vga_r           (vera_r),
        .vga_g           (vera_g),
        .vga_b           (vera_b),
        .vga_hsync       (vera_hs),
        .vga_vsync       (vera_vs),
        .vga_de          (vera_de),
        .vga_opaque      (vera_opaque),

        .spi_sck         (vera_spi_sck),
        .spi_mosi        (vera_spi_mosi),
        .spi_miso        (vera_sd_miso),        // guest SD via HPS image bridge
        .spi_ssel_n_sd   (vera_spi_ssel_n),

        .audio_lrck      (vera_audio_lrck),
        .audio_bck       (vera_audio_bck),
        .audio_data      (vera_audio_data),

        .dbg_wrdata_r    (),
        .dbg_wraddr_r    (),
        .dbg_do_write    (),
        .dbg_video_mode  (),
        .dbg_dcsel       (),
        .spi_busy_out    (),
        .spi_autotx_out  (),

        .composite_luma  (),
        .composite_chroma()
    );

    // ====================================================================
    // Custom SD-SPI master (rtl/spi_sd_master100.sv, 100 MHz domain)
    // --------------------------------------------------------------------
    // R49 drives the X16 SD card by bit-banging VERA regs $9F3E (data) and
    // $9F3F (ctrl: bit2 autotx, bit1 slow, bit0 select; read bit7 = busy).
    // Runs entirely in cpu_clk -- the SAME clock as sd_card -- so there is
    // NO clock crossing.  Register reads are steered to this master in the
    // cpu_di mux; a $9F3E READ is stalled (spi_stall) until every queued
    // byte has shifted out.  $9F3E WRITES are NOT stalled -- they go into
    // the master's 512-byte write FIFO (BUG3 fix): the r65c02 ignores RDY
    // on write cycles, so a stall can never protect a write, and the ROM's
    // fast-write loop (back-to-back STA $9F3E, no polling) would drop
    // bytes against this master's 2 us/byte pace (sim/tb_write.v).
    // sck = cpu_clk/2 = 4 MHz (jyv 2026-07-07, was 1 MHz: streaming demos
    // -- 8BG video player, continuous MACPTR -> PCM FIFO -- underran at
    // ~110 KB/s; now ~440 KB/s, sim/tb_spi4.v proves byte integrity in
    // this exact topology).
    // bus_valid-qualified: a GHOST read of $9F3E on an '816 internal cycle
    // would fire an SPI auto-transmit and desync the SD byte stream (the
    // boot-time SD probe then feeds CBDOS garbage -> wild jump -> monitor).
    wire acc_9f3e = cpu_bus_valid & (cpu_a == 16'h9F3E);
    wire acc_9f3f = cpu_bus_valid & (cpu_a == 16'h9F3F);
    wire rd_9f3e  = cpu_rdy & cpu_rwn & acc_9f3e;           // committed read (auto-tx)
    wire [7:0] m_data, m_status;

    // jyv 2026-07-07 (SD-speed saga, act 3): the master now lives in the
    // 100 MHz sd_card domain (rtl/spi_sd_master100.sv), sck = 12.5 MHz --
    // real VERA fast-SPI speed, 640 ns/byte, reads never stall.  History:
    // 1 MHz starved streaming demos (PCM underruns); 4 MHz from the cpu_clk
    // domain did not boot (false-pathed miso CDC); 2 MHz was safe but still
    // underran the 8BG video demo.  The clock crossing is now the REGISTER
    // interface (toggle handshakes, events >= 1.6 us apart) instead of the
    // SPI wires; every SPI path is single-clock and fully timed by Quartus.
    spi_sd_master100 u_spimaster (
        .clk_cpu   (cpu_clk),
        .rst_cpu_n (cpu_reset_n),
        .rd_data   (rd_9f3e),
        .cpu_do    (cpu_do),
        .acc_data  (acc_9f3e),
        .acc_ctrl  (acc_9f3f),
        .cpu_rwn   (cpu_rwn),
        .stall     (spi_stall),
        .data_q    (m_data),
        .status_q  (m_status),

        .clk       (sdram_clk),
        .rst_n     (~hs_rst_sync[1]),
        .sck       (m_sck),
        .mosi      (m_mosi),
        .sel       (m_sel),
        .miso      (sd_card_miso)
    );

    // ========================================================================
    // IKAOPM (YM2151)
    //
    // CLOCKING (audio-bug fix 2026-07-06): the X16's YM2151 runs phiM at
    // 3.579545 MHz.  The old wiring ran EMUCLK=12.5 MHz with the clock
    // enable permanently asserted = a 3.5x-overclocked OPM (2 octaves
    // sharp, wrong timers) whose 8 MHz-domain bus strobes were sampled
    // unsynchronized (metastable register writes -> randomly silent
    // channels).  Now: EMUCLK = pix_clk (25 MHz) with a /7 phiM clock
    // enable = 3.5714 MHz (-0.23% = ~4 cents flat, inaudible), and the CPU
    // bus is handshaked across properly:
    //   * writes: captured once per bus-write edge in the cpu domain
    //     (works for both the r65c02's 1-cycle writes and the '816's held
    //     writes), toggle-synced into pix_clk, presented to IKAOPM for a
    //     full 2+ phiM periods;
    //   * reads: IKAOPM idles in "status read" (CS+RD held) so o_D is
    //     continuously captured; the CPU reads a synced copy with the
    //     write-pending flag OR'd into BUSY (bit 7), so a busy-poll issued
    //     right after a write can never sneak past the handshake latency.
    // Bonus: o_EMU_L/R now live in pix_clk = the mixer's domain (the old
    // cross-domain sample tearing is gone).
    // ========================================================================
    wire        ym_wr     = ym_cs & ~cpu_rwn;
    wire [15:0] ym_emu_r, ym_emu_l;
    wire [7:0]  ym_od;

    // pix-domain reset for the OPM core
    reg [1:0] ymrst_sync = 2'b00;
    always @(posedge pix_clk or negedge sys_rst_n)
        if (!sys_rst_n) ymrst_sync <= 2'b00;
        else            ymrst_sync <= {ymrst_sync[0], 1'b1};
    wire ym_reset_n = ymrst_sync[1];

    // phiM clock enable: 25 MHz / 7 = 3.5714 MHz (active-low, 1-of-7)
    reg [2:0] ym_div = 3'd0;
    always @(posedge pix_clk) ym_div <= (ym_div == 3'd6) ? 3'd0 : ym_div + 3'd1;
    wire ym_pcen_n = (ym_div != 3'd0);

    // ---- cpu-domain write capture (one capture per bus-write edge) ----
    reg        ym_req_t = 1'b0;
    reg        ym_wr_d  = 1'b0;
    reg        ym_wa0   = 1'b0;
    reg  [7:0] ym_wdat  = 8'h00;
    always @(posedge cpu_clk) begin
        ym_wr_d <= ym_wr;
        if (ym_wr & ~ym_wr_d) begin
            ym_wa0   <= cpu_a[0];
            ym_wdat  <= cpu_do;
            ym_req_t <= ~ym_req_t;
        end
    end

    // ---- pix-domain bus FSM: status-read idle, timed write bursts ----
    reg  [2:0] ym_req_s = 3'b000;
    always @(posedge pix_clk) ym_req_s <= {ym_req_s[1:0], ym_req_t};
    wire ym_req_edge = ym_req_s[2] ^ ym_req_s[1];

    reg        ym_ack_t  = 1'b0;
    reg        ym_bus_wr = 1'b0;
    reg  [4:0] ym_hold   = 5'd0;
    reg        ym_a0_r   = 1'b0;
    reg  [7:0] ym_d_r    = 8'h00;
    reg  [7:0] ym_status = 8'h00;
    // post-write: for DATA writes (A0=1), hold the pending/busy shadow until
    // the OPM's own BUSY flag is seen completing (rise then fall) -- IKAOPM
    // silently drops a data write issued while the previous one is still
    // unconsumed (like the real chip), and the raw delivery-ack window left
    // a gap where a fast busy-poll could sneak the next write in too early.
    reg  [1:0] ym_post   = 2'd0;
    reg  [8:0] ym_tmo    = 9'd0;
    always @(posedge pix_clk) begin
        case (ym_post)
        2'd0: begin
            ym_status <= ym_od;              // idle = continuous status capture
            if (ym_req_edge) begin
                ym_a0_r   <= ym_wa0;         // stable: written 2+ cpu cycles ago
                ym_d_r    <= ym_wdat;
                ym_bus_wr <= 1'b1;
                ym_hold   <= 5'd15;          // 16 pix cycles = 640 ns > 2 phiM
                ym_post   <= 2'd1;
            end
        end
        2'd1: begin
            if (ym_hold != 5'd0) ym_hold <= ym_hold - 5'd1;
            else begin
                ym_bus_wr <= 1'b0;
                if (ym_a0_r) begin           // data write: track OPM busy
                    ym_post <= 2'd2;
                    ym_tmo  <= 9'd200;       // ~8 us guard for the rise
                end else begin               // address write: done
                    ym_ack_t <= ~ym_ack_t;
                    ym_post  <= 2'd0;
                end
            end
        end
        2'd2: begin                          // wait busy rise (or timeout)
            ym_tmo <= ym_tmo - 9'd1;
            if (ym_od[7]) begin
                ym_post <= 2'd3;
                ym_tmo  <= 9'd500;           // ~20 us >= real busy duration
            end
            else if (ym_tmo == 9'd0) begin
                ym_ack_t <= ~ym_ack_t;       // consumed faster than visible
                ym_post  <= 2'd0;
            end
        end
        2'd3: begin                          // wait busy fall (bounded: the
            ym_tmo <= ym_tmo - 9'd1;         // TEST reg can repurpose o_D)
            if (!ym_od[7] || ym_tmo == 9'd0) begin
                ym_ack_t <= ~ym_ack_t;
                ym_post  <= 2'd0;
            end
        end
        endcase
    end

    // ---- cpu-domain read data: synced status + pending-write busy ----
    reg [1:0] ym_ack_s = 2'b00;
    reg [7:0] ym_status_s = 8'h00, ym_status_c = 8'h00;
    always @(posedge cpu_clk) begin
        ym_ack_s    <= {ym_ack_s[0], ym_ack_t};
        ym_status_s <= ym_status;         // 2-stage sample of quasi-static byte
        ym_status_c <= ym_status_s;
    end
    wire ym_pending = ym_req_t ^ ym_ack_s[1];
    wire [7:0] ym_rd_data = {ym_status_c[7] | ym_pending, ym_status_c[6:0]};

    IKAOPM #(
        .FULLY_SYNCHRONOUS  (1),
        .FAST_RESET         (1),
        .USE_BRAM           (0)
    ) u_ym2151 (
        .i_EMUCLK           (pix_clk),
        .i_phiM_PCEN_n      (ym_pcen_n),
        .i_IC_n             (ym_reset_n),
        .i_CS_n             (1'b0),               // always selected (see FSM)
        .i_RD_n             (ym_bus_wr),          // idle: continuous status read
        .i_WR_n             (~ym_bus_wr),
        .i_A0               (ym_bus_wr ? ym_a0_r : 1'b0),
        .i_D                (ym_d_r),
        .o_D                (ym_od),
        .o_D_OE             (),
        .o_CT1              (),
        .o_CT2              (),
        .o_IRQ_n            (ym_irq_n),
        .o_SH1              (),
        .o_SH2              (),
        .o_SO               (),
        .o_EMU_R            (ym_emu_r),
        .o_EMU_L            (ym_emu_l),
        .o_EMU_R_SAMPLE     (),
        .o_EMU_L_SAMPLE     ()
    );

    // ========================================================================
    // VIA #1 (NES gamepad + internal I2C bit-bang to SMC)
    // ========================================================================
    wire [7:0] via1_data;
    wire [7:0] via1_pa_in, via1_pa_out, via1_pa_oe;
    wire [7:0] via1_pb_in, via1_pb_out, via1_pb_oe;

    // Internal I2C bus -- VIA1 PA[1] = SCL, PA[0] = SDA.  Open-drain combined
    // with smc_x16's sda_drive_low.
    // RTC_PRESENT: hardware-bisect knob for the 20260706 splash-freeze hunt.
    // 0 = the RTC never drives SDA -> the KERNAL sees a clean NACK at $6F
    // (exactly like the pre-RTC builds) while ALL other 0706 logic stays in.
    // If a =0 build still freezes, the RTC is innocent and the failure is the
    // BUG2-class placement/latch flakiness; if =0 boots, the RTC's silicon
    // behavior is the culprit and gets debugged next.
    localparam RTC_PRESENT = 1'b1;

    wire smc_sda_drv_low, rtc_sda_drv_low;
    wire via_sda_drv_low = via1_pa_oe[0] & ~via1_pa_out[0];
    wire via_scl_drv_low = via1_pa_oe[1] & ~via1_pa_out[1];
    wire bus_sda         = ~(via_sda_drv_low | smc_sda_drv_low
                             | (rtc_sda_drv_low & RTC_PRESENT));
    wire bus_scl         = ~via_scl_drv_low;

    // MCP7940N RTC + NVRAM, second I2C slave (addr $6F), wall clock from HPS
    wire [6:0] nv_addr, nv_snoop_addr;
    wire [7:0] nv_wdata, nv_snoop_data;
    wire       nv_we, nv_snoop_we, nv_dirty_toggle;

    rtc_x16 u_rtc (
        .clk           (cpu_clk),
        .reset_n       (cpu_reset_n),
        .sda_bus       (bus_sda),
        .scl_bus       (bus_scl),
        .sda_drive_low (rtc_sda_drv_low),
        .hps_rtc       (hps_rtc),

        // NVRAM persistence port (cpu_clk domain; u_nvram owns the CDC)
        .nv_we         (nv_we),
        .nv_addr       (nv_addr),
        .nv_wdata      (nv_wdata),
        .nv_snoop_we   (nv_snoop_we),
        .nv_snoop_addr (nv_snoop_addr),
        .nv_snoop_data (nv_snoop_data),
        .nv_dirty_toggle(nv_dirty_toggle)
    );

    // ------------------------------------------------------------------
    // RTC NVRAM persistence (jyv 2026-07-07): the KERNAL keeps its settings
    // (screen mode etc.) in the RTC SRAM; back it with the S1 "NVRAM" disk
    // image so they survive power cycles.  Restore on mount, autosave ~1 s
    // after the last CPU write.  See rtl/nvram_backer.sv.
    // ------------------------------------------------------------------
    nvram_backer u_nvram (
        .clk            (sdram_clk),
        .cpu_clk        (cpu_clk),
        .reset_n        (bk_reset_n),
        .img_mounted    (img_mounted[1]),
        .img_readonly   (img_readonly),
        .img_size       (img_size),
        .sd_lba         (sd_lba[1]),
        .sd_rd          (sd_rd[1]),
        .sd_wr          (sd_wr[1]),
        .sd_ack         (sd_ack[1]),
        .sd_buff_addr   (sd_buff_addr[8:0]),
        .sd_buff_dout   (sd_buff_dout),
        .sd_buff_din    (sd_buff_din[1]),
        .sd_buff_wr     (sd_buff_wr),
        .nv_we          (nv_we),
        .nv_addr        (nv_addr),
        .nv_wdata       (nv_wdata),
        .nv_snoop_we    (nv_snoop_we),
        .nv_snoop_addr  (nv_snoop_addr),
        .nv_snoop_data  (nv_snoop_data),
        .nv_dirty_toggle(nv_dirty_toggle)
    );

    // ------------------------------------------------------------------
    // Cart RAM persistence (jyv 2026-07-07): back ROM banks 32-255 (SDRAM
    // cart space) with the SC2 disk image -- restore on mount through the
    // loader port, dirty-sector save-back through the bk_* read port.
    // See rtl/cart_backer.sv.
    // ------------------------------------------------------------------
    cart_backer u_cart (
        .clk            (sdram_clk),
        .reset_n        (bk_reset_n),
        .img_mounted    (img_mounted[2]),
        .img_readonly   (img_readonly),
        .img_size       (img_size),
        .typed_mode     (status[4]),      // OSD "Cart mount": 0=raw all-RAM, 1=typed .crt
        .sd_lba         (sd_lba[2]),
        .sd_blk_cnt     (sd_blk_cnt[2]),
        .sd_rd          (sd_rd[2]),
        .sd_wr          (sd_wr[2]),
        .sd_ack         (sd_ack[2]),
        .sd_buff_addr   (sd_buff_addr),
        .sd_buff_dout   (sd_buff_dout),
        .sd_buff_din    (sd_buff_din[2]),
        .sd_buff_wr     (sd_buff_wr),
        .rst_ld_wr      (crt_ld_wr),
        .rst_ld_addr    (crt_ld_addr),
        .rst_ld_data    (crt_ld_data),
        .ld_busy        (cart_ld_busy),
        .bk_rd          (crt_bk_rd),
        .bk_addr        (crt_bk_addr),
        .bk_rdata       (crt_bk_rdata),
        .bk_ack         (crt_bk_ack),
        .wr_snoop       (crt_wr_snoop),
        .wr_snoop_addr  (crt_wr_snoop_addr),
        .restoring      (crt_restoring),
        .cart_wmask     (cart_wmask)
    );

    // SNES controllers (r49 joystick.s pinout): PA2 = LATCH, PA3 = CLK
    // (outputs, both pads), PA7 = pad #1 DATA, PA6 = pad #2 DATA, PA5/PA4 =
    // pads #3/#4 (not populated -> read 1 = "absent", like open lines).
    // MiSTer pads arrive on hps_io joystick_0/1 (100 MHz domain, quasi-static
    // button levels) -> 2-FF sync into cpu_clk.
    reg [11:0] joy0_s1, joy0_s2, joy1_s1, joy1_s2;
    always @(posedge cpu_clk) begin
        joy0_s1 <= joystick_0[11:0];  joy0_s2 <= joy0_s1;
        joy1_s1 <= joystick_1[11:0];  joy1_s2 <= joy1_s1;
    end

    wire snes_latch = via1_pa_out[2] | ~via1_pa_oe[2];   // pulled up when undriven
    wire snes_clk   = via1_pa_out[3] | ~via1_pa_oe[3];
    wire snes_data1, snes_data2;

    snes_pad u_pad1 (
        .clk(cpu_clk), .reset_n(cpu_reset_n),
        .joy(joy0_s2), .latch(snes_latch), .jclk(snes_clk), .data(snes_data1)
    );
    snes_pad u_pad2 (
        .clk(cpu_clk), .reset_n(cpu_reset_n),
        .joy(joy1_s2), .latch(snes_latch), .jclk(snes_clk), .data(snes_data2)
    );

    assign via1_pa_in[0]   = bus_sda;
    assign via1_pa_in[1]   = bus_scl;
    assign via1_pa_in[3:2] = via1_pa_out[3:2] | ~via1_pa_oe[3:2];
    assign via1_pa_in[4]   = 1'b1;          // pad #4 absent
    assign via1_pa_in[5]   = 1'b1;          // pad #3 absent
    assign via1_pa_in[6]   = snes_data2;    // pad #2
    assign via1_pa_in[7]   = snes_data1;    // pad #1
    assign via1_pb_in[7:6] = 2'b11;
    assign via1_pb_in[5:0] = via1_pb_out[5:0] | ~via1_pb_oe[5:0];

    via65c22 u_via1 (
        .clk     (cpu_clk),
        .reset_n (cpu_reset_n),
        .ce      (ce_1x),
        .cs      (via1_cs),
        .rwn     (cpu_rwn),
        .enable  (cpu_rdy),
        .addr    (cpu_a[3:0]),
        .di      (cpu_do),
        .do_o    (via1_data),
        .pa_in   (via1_pa_in),
        .pa_out  (via1_pa_out),
        .pa_oe   (via1_pa_oe),
        .pb_in   (via1_pb_in),
        .pb_out  (via1_pb_out),
        .pb_oe   (via1_pb_oe),
        .ca1_in  (1'b0),
        .ca2_in  (1'b0),
        .cb1_in  (1'b0),
        .cb2_in  (1'b0),
        .ca2_out (),
        .ca2_oe  (),
        .cb1_out (),
        .cb1_oe  (),
        .cb2_out (),
        .cb2_oe  (),
        .irq_n   (via1_irq_n)
    );

    // ========================================================================
    // VIA #2 (user port)
    //
    // Leave the user-port pins floating/pulled-up like the stock machine.
    // Serial-card emulation below owns the MiSTer host UART at $9FE0.
    // ========================================================================
    wire [7:0] via2_data;
    wire [7:0] via2_pa_in, via2_pa_out, via2_pa_oe;
    wire [7:0] via2_pb_in, via2_pb_out, via2_pb_oe;

    assign via2_pa_in = via2_pa_out | ~via2_pa_oe;
    assign via2_pb_in = via2_pb_out | ~via2_pb_oe;

    via65c22 u_via2 (
        .clk     (cpu_clk),
        .reset_n (cpu_reset_n),
        .ce      (ce_1x),
        .cs      (via2_cs),
        .rwn     (cpu_rwn),
        .enable  (cpu_rdy),
        .addr    (cpu_a[3:0]),
        .di      (cpu_do),
        .do_o    (via2_data),
        .pa_in   (via2_pa_in),
        .pa_out  (via2_pa_out),
        .pa_oe   (via2_pa_oe),
        .pb_in   (via2_pb_in),
        .pb_out  (via2_pb_out),
        .pb_oe   (via2_pb_oe),
        .ca1_in  (UART_RXD),
        .ca2_in  (1'b0),
        .cb1_in  (1'b0),
        .cb2_in  (1'b0),
        .ca2_out (), .ca2_oe (),
        .cb1_out (), .cb1_oe (),
        .cb2_out (), .cb2_oe (),
        .irq_n   (via2_irq_n)
    );

    // ========================================================================
    // Serial/network card emulation at $9FE0-$9FEF
    //
    // ROMTERM expects the X16 serial/network card default at IO7-Low with
    // two distinct UARTs at $9FE0 and $9FE8. MiSTer only exposes one host
    // UART, so status[2] maps it to one guest UART at a time. This mirrors the
    // card's separate network ($9FE0) and physical serial ($9FE8) connectors
    // without trying to infer ROMTERM's selected port from register traffic.
    // Feeding RX to both banks duplicates replies into the companion FIFO.
    // Changing the OSD mapping resets both banks to discard stale data. The
    // physical card does not cross-connect DTR between UARTs and has no RI
    // input. ROMTERM expects the companion RI bit to stay low while probing
    // DTR; cross-connecting it rejects the card.
    // ========================================================================
    wire [7:0] serial0_data;
    wire [7:0] serial1_data;
    wire serial0_txd, serial0_rts, serial0_dtr;
    wire serial1_txd, serial1_rts, serial1_dtr;
    reg [1:0] serial_port_sync = 2'b00;
    reg serial_uart_sel = 1'b0;
    reg [1:0] serial_reset_hold = 2'b00;
    wire serial_reset_n = cpu_reset_n & (serial_reset_hold == 2'b00);
    wire serial0_rxd = serial_uart_sel ? 1'b1 : UART_RXD;
    wire serial1_rxd = serial_uart_sel ? UART_RXD : 1'b1;

    always @(posedge cpu_clk or negedge cpu_reset_n) begin
        if (!cpu_reset_n) begin
            serial_port_sync <= 2'b00;
            serial_uart_sel <= 1'b0;
            serial_reset_hold <= 2'b00;
        end
        else begin
            serial_port_sync <= {serial_port_sync[0], status[2]};
            if (serial_port_sync[1] != serial_uart_sel) begin
                serial_uart_sel <= serial_port_sync[1];
                serial_reset_hold <= 2'b11;
            end else if (serial_reset_hold != 2'b00) begin
                serial_reset_hold <= serial_reset_hold - 2'b01;
            end
        end
    end

    x16_serial_card #(
        .CLK_HZ         (32_000_000),
        .DEFAULT_DIVISOR(16'd32)
    ) u_serial0 (
        .clk      (cpu_clk),
        .reset_n  (serial_reset_n),
        .cs       (serial0_cs),
        .rwn      (cpu_rwn),
        .enable   (cpu_rdy),
        .addr     (cpu_a[2:0]),
        .di       (cpu_do),
        .do_o     (serial0_data),
        .uart_rxd (serial0_rxd),
        .uart_cts (UART_CTS),
        .uart_dsr (UART_DSR),
        .uart_ri  (1'b0),
        .uart_txd (serial0_txd),
        .uart_rts (serial0_rts),
        .uart_dtr (serial0_dtr)
    );

    x16_serial_card #(
        .CLK_HZ         (32_000_000),
        .DEFAULT_DIVISOR(16'd32)
    ) u_serial1 (
        .clk      (cpu_clk),
        .reset_n  (serial_reset_n),
        .cs       (serial1_cs),
        .rwn      (cpu_rwn),
        .enable   (cpu_rdy),
        .addr     (cpu_a[2:0]),
        .di       (cpu_do),
        .do_o     (serial1_data),
        .uart_rxd (serial1_rxd),
        .uart_cts (UART_CTS),
        .uart_dsr (UART_DSR),
        .uart_ri  (1'b0),
        .uart_txd (serial1_txd),
        .uart_rts (serial1_rts),
        .uart_dtr (serial1_dtr)
    );

    assign UART_TXD = serial_uart_sel ? serial1_txd : serial0_txd;
    assign UART_RTS = serial_uart_sel ? serial1_rts : serial0_rts;
    assign UART_DTR = serial_uart_sel ? serial1_dtr : serial0_dtr;

    // ========================================================================
    // SDRAM-backed bitmap layer ($9F60-$9F6F).  OSD master-enable = status[5].
    // Register block (cpu_clk) + scanout engine (pix_clk fill in sdram_clk).
    // Composited over VERA by the VGA mux below; VERA is untouched.
    // ========================================================================
    bitmap_regs u_bmp_regs (
        .clk          (cpu_clk),
        .reset_n      (cpu_reset_n),
        .cs           (bmp_sel),          // OSD-gated: off -> device inert
        .rwn          (cpu_rwn),
        .en           (cpu_rdy),
        .addr         (cpu_a[3:0]),
        .di           (cpu_do),
        .do_o         (bmp_do),
        .master_en    (bmp_master_en),
        .bmp_enable   (bmp_enable),
        .bmp_mode     (bmp_mode),
        .bmp_passthru (bmp_passthru),
        .fb_wr_sel    (bmp_fb_wr),
        .fb_rd_sel    (bmp_fb_rd),
        .fb_addr      (bmp_fb_addr),
        .pal_we       (bmp_pal_we),
        .pal_idx      (bmp_pal_idx),
        .pal_data     (bmp_pal_data),
        .blit_start   (bmp_blit_start),
        .blit_src     (bmp_blit_src),
        .blit_dst     (bmp_blit_dst),
        .blit_len     (bmp_blit_len),
        .blit_done    (bmp_blit_done)
    );

    bitmap_engine u_bmp_engine (
        .pix_clk    (pix_clk),
        .reset_n    (sys_rst_n),
        .enable     (bmp_enable),
        .mode       (bmp_mode),
        .de         (vera_de),
        .vs         (vera_vs),
        .bmp_r      (bmp_r),
        .bmp_g      (bmp_g),
        .bmp_b      (bmp_b),
        .bmp_active (bmp_active),
        .pal_clk    (cpu_clk),
        .pal_we     (bmp_pal_we),
        .pal_idx    (bmp_pal_idx),
        .pal_data   (bmp_pal_data),
        .sdram_clk  (sdram_clk),
        .fb_go      (bmp_fb_go),
        .fb_base    (bmp_fb_base),
        .fb_len     (bmp_fb_len),
        .fb_valid   (bmp_fb_valid),
        .fb_word    (bmp_fb_word),
        .fb_done    (bmp_fb_done)
    );

    // ========================================================================
    // PS/2 keyboard bridge (Phase e):  hps_io ps2_key -> smc_x16 byte stream
    // ========================================================================
    wire [7:0] smc_uart_byte;
    wire       smc_uart_byte_valid;

    // ps2_key now originates in the 100 MHz hps_io domain: 2-FF sync the
    // vector into cpu_clk.  Bit 10 is a toggle marking each new event and the
    // payload is held stable between events (ms apart), so a plain vector
    // sync is safe -- the bridge acts on the toggle edge after settling.
    reg [10:0] ps2_key_s1, ps2_key_s2;
    always @(posedge cpu_clk) begin
        ps2_key_s1 <= ps2_key;
        ps2_key_s2 <= ps2_key_s1;
    end

    // ps2_mouse crosses like ps2_key: [24] toggles per event, the payload is
    // stable between events (ms apart) -> plain vector 2-FF sync is safe.
    // The wheel delta (ps2_mouse_ext[7:0]) is written by the same HPS command
    // before the toggle flips, so it rides the same sync recipe.
    reg [24:0] ps2_mouse_s1, ps2_mouse_s2;
    reg [7:0]  ps2_mwheel_s1, ps2_mwheel_s2;
    always @(posedge cpu_clk) begin
        ps2_mouse_s1  <= ps2_mouse;
        ps2_mouse_s2  <= ps2_mouse_s1;
        ps2_mwheel_s1 <= ps2_mouse_ext[7:0];
        ps2_mwheel_s2 <= ps2_mwheel_s1;
    end

    ps2_to_smc_bridge u_ps2_bridge (
        .clk             (cpu_clk),
        .reset_n         (cpu_reset_n),
        .ps2_key         (ps2_key_s2),
        .ps2_mouse       (ps2_mouse_s2),
        .ps2_mouse_wheel (ps2_mwheel_s2),
        .uart_byte       (smc_uart_byte),
        .uart_byte_valid (smc_uart_byte_valid)
    );


    // ========================================================================
    // SMC (I2C slave at $42 -- keyboard / mouse / system control)
    // ========================================================================
    smc_x16 u_smc (
        .clk             (cpu_clk),
        .reset_n         (cpu_reset_n),
        .sda_bus         (bus_sda),
        .scl_bus         (bus_scl),
        .sda_drive_low   (smc_sda_drv_low),
        .uart_byte       (smc_uart_byte),
        .uart_byte_valid (smc_uart_byte_valid),
        .power_off_req   (smc_power_off_req),   // treated as reset (no PSU here)
        .reset_req       (smc_reset_req),
        .nmi_req         (smc_nmi_req),
        .act_led_r       (smc_act_led),
        .dbg_kbd_count   (),
        .dbg_saw_start   (),
        .dbg_saw_addr_match(),
        .dbg_saw_byte    (),
        .dbg_saw_repeat  (),
        .dbg_saw_stop    (),
        .dbg_saw_tx      (),
        .dbg_last_cmd    (),
        .dbg_last_addr_byte (),
        .dbg_kbd_pop     (),
        .dbg_tx_byte     ()
    );

    // ========================================================================
    // CPU data-in mux
    // ========================================================================
    // Open-bus emulation (jyv 2026-07-07): unmapped reads -- notably the
    // $9F60-$9FFF expansion-card IO space -- return the last byte seen on
    // the CPU data bus, like real hardware's floating bus.  Returning $00
    // here made expansion-detection code (second VERA, MIDI, network
    // cards) false-positive on "device present at $00".
    reg [7:0] open_bus_r = 8'h00;
    always @(posedge cpu_clk)
        if (cpu_rdy) open_bus_r <= cpu_rwn ? cpu_di : cpu_do;

    assign cpu_di = kernal_cs ? (cart_sel     ? sdram_rd :   // ROM banks 32-255 (cart, SDRAM)
                                 rom_open_sel ? 8'hFF    :   // ROM banks 16-31 (unpopulated)
                                                rom_data ) : // ROM banks 0-15 (BRAM)
                    hi_ram_cs ? ext_ram_data :
                    acc_9f3e  ? m_data       :   // SD SPI data  -> cpu_clk master
                    acc_9f3f  ? m_status     :   // SD SPI status/busy -> cpu_clk master
                    vera_cs   ? vera_extbus_d :
                    ym_cs     ? ym_rd_data   :
                    via1_cs   ? via1_data    :
                    via2_cs   ? via2_data    :
                    serial0_cs ? serial0_data :
                    serial1_cs ? serial1_data :
                    bmp_fb_rd  ? sdram_rd     :   // $9F65 read = framebuffer byte (SDRAM)
                    bmp_sel    ? bmp_do       :   // $9F60-$9F6F bitmap regs (OSD-gated)
                    lowram_cs ? lowram_data  :
                                open_bus_r;   // unmapped: floating bus

    // ========================================================================
    // Video output  --  drive MiSTer VGA_* from VERA
    // ========================================================================
    assign CLK_VIDEO   = pix_clk;
    assign CE_PIXEL    = 1'b1;

    // Bitmap layer composited with VERA.  HS/VS/DE stay VERA's -- the bitmap
    // rides its raster.  Default (passthru=0): the bitmap fully replaces VERA.
    // With CTRL[3] passthru=1: wherever VERA has an opaque pixel (a sprite --
    // e.g. the mouse cursor -- or a layer) VERA shows OVER the bitmap; the
    // bitmap fills everywhere VERA is transparent.  That keeps the hardware
    // mouse pointer visible on a bitmap desktop (see vera_2.md / CXRF).
    // ---- bitmap / VERA horizontal alignment -----------------------------
    // The bitmap engine emits each pixel BMP_ALIGN pix_clks after vera_de
    // (linebuf-read + palette-read + output regs).  VERA's own pixels come
    // out at vera_de with no such delay, so composited directly the bitmap
    // sits BMP_ALIGN px to the right and VERA peeks through the left edge
    // (visible only on real hardware -- the emulator composites per pixel).
    // Delay the whole VERA output bus by the same amount so the two line up.
    // Gated by the OSD master-enable: with the layer Off the VERA path is
    // bit-for-bit stock timing.  bitmap_engine is still fed the UNDELAYED
    // vera_de/vera_vs, so its internal timing is unchanged.
    localparam integer BMP_ALIGN = 4;
    reg [15:0] vera_dly [0:BMP_ALIGN-1];
    integer bai;
    always @(posedge pix_clk) begin
        vera_dly[0] <= {vera_hs, vera_vs, vera_de, vera_opaque,
                        vera_r, vera_g, vera_b};
        for (bai = 1; bai < BMP_ALIGN; bai = bai + 1)
            vera_dly[bai] <= vera_dly[bai-1];
    end
    wire [15:0] vdl  = vera_dly[BMP_ALIGN-1];
    wire        a_hs = bmp_master_en ? vdl[15]   : vera_hs;
    wire        a_vs = bmp_master_en ? vdl[14]   : vera_vs;
    wire        a_de = bmp_master_en ? vdl[13]   : vera_de;
    wire        a_op = bmp_master_en ? vdl[12]   : vera_opaque;
    wire  [3:0] a_r  = bmp_master_en ? vdl[11:8] : vera_r;
    wire  [3:0] a_g  = bmp_master_en ? vdl[7:4]  : vera_g;
    wire  [3:0] a_b  = bmp_master_en ? vdl[3:0]  : vera_b;

    wire use_bmp = bmp_active & ~(bmp_passthru & a_op);
    assign VGA_R       = use_bmp ? {bmp_r, bmp_r} : {a_r, a_r};
    assign VGA_G       = use_bmp ? {bmp_g, bmp_g} : {a_g, a_g};
    assign VGA_B       = use_bmp ? {bmp_b, bmp_b} : {a_b, a_b};
    assign VGA_HS      = a_hs;
    assign VGA_VS      = a_vs;
    assign VGA_DE      = a_de;
    assign VGA_F1      = 1'b0;
    assign VGA_SL      = 2'b00;
    assign VGA_SCALER  = 1'b0;
    assign VGA_DISABLE = 1'b0;
    assign VIDEO_ARX   = 13'd4;
    assign VIDEO_ARY   = 13'd3;
    assign HDMI_FREEZE    = 1'b0;
    assign HDMI_BLACKOUT  = 1'b0;
    assign HDMI_BOB_DEINT = 1'b0;

    // ========================================================================
    // Audio output  --  VERA (PSG+PCM) + YM2151 mixed into AUDIO_L/R
    //
    // VERA serializes its audio as I2S on (lrck, bck, data); the i2s_rx
    // module (end of this file) recovers 16-bit L/R samples in pix_clk.
    wire signed [15:0] vera_al, vera_ar;
    i2s_rx u_i2s_rx (
        .clk   (pix_clk),
        .lrck  (vera_audio_lrck),
        .bck   (vera_audio_bck),
        .data  (vera_audio_data),
        .left  (vera_al),
        .right (vera_ar)
    );

    // Saturating 16-bit mix of YM2151 and VERA.  The two run on different
    // clocks (aud_mclk / pix_clk) but both are audio-rate sample streams; the
    // framework resamples AUDIO_L/R on its own clock, so a plain registered
    // add is the standard MiSTer pattern.
    function automatic [15:0] sat16(input signed [17:0] s);
        sat16 = (s >  18'sd32767) ? 16'h7FFF :
                (s < -18'sd32768) ? 16'h8000 : s[15:0];
    endfunction

    // jyv 2026-07-07 (tester report "PSG a bit too low"): VERA's full scale
    // through the top-16 I2S tap is +/-16K (the 17-bit PSG+PCM mix sits in
    // bits [23:7], so the tap sees mix>>1) while the YM runs a full +/-32K
    // -- ALL VERA audio was 6 dB under the YM and the emulator.  Scale VERA
    // x2 in the mix: full-scale VERA == full-scale YM, like the real
    // board's analog mixer.
    reg [15:0] audio_l_r, audio_r_r;
    always @(posedge pix_clk) begin
        audio_l_r <= sat16($signed({{2{ym_emu_l[15]}}, ym_emu_l})
                         + ($signed({{2{vera_al[15]}}, vera_al}) <<< 1));
        audio_r_r <= sat16($signed({{2{ym_emu_r[15]}}, ym_emu_r})
                         + ($signed({{2{vera_ar[15]}}, vera_ar}) <<< 1));
    end
    //   VERA audio is I2S-serial (audio_data); skipping the deserializer in
    //   Phase d -- only IKAOPM is audible until Phase g.
    // ========================================================================
    // CLK_AUDIO (24.576 MHz) is an input from sys_top; mixer uses it later.
    assign AUDIO_L   = audio_l_r;
    assign AUDIO_R   = audio_r_r;
    assign AUDIO_S   = 1'b1;        // signed
    assign AUDIO_MIX = 2'b00;       // no internal mix

    // ========================================================================
    // Tie-offs for unused framework signals
    // ========================================================================
    assign LED_USER  = ioctl_download | (smc_act_led != 8'h00);
    assign LED_POWER = 2'b00;
    assign LED_DISK  = 2'b00;
    assign BUTTONS   = 2'b00;

    // SD-SPI (unused -- VERA will drive once Phase g wires it through hps_io)
    assign SD_SCK    = 1'b0;
    assign SD_MOSI   = 1'b0;
    assign SD_CS     = 1'b1;

    // DDRAM (unused)
    assign DDRAM_CLK      = cpu_clk;
    assign DDRAM_BURSTCNT = 8'h00;
    assign DDRAM_ADDR     = 29'h0;
    assign DDRAM_RD       = 1'b0;
    assign DDRAM_DIN      = 64'h0;
    assign DDRAM_BE       = 8'h00;
    assign DDRAM_WE       = 1'b0;

    // SDRAM driven by ext_ram_sdram (HiRAM) -- see u_hiram above.

    assign USER_OUT = 7'h7F;

    // ADC (unused)
    assign ADC_BUS  = 4'hZ;

endmodule
