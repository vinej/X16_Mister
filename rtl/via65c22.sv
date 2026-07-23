// ============================================================================
// via65c22.sv -- 65C22 "Versatile Interface Adapter" model for the X16 FPGA.
//
// Two instances live inside C5G_x16:
//   VIA#1 @ $9F00-$9F0F : I2C bit-bang on PA[1:0], NES controllers on
//                         PA[7:2], IEC serial on PB.  IRQ -> CPU IRQ.
//   VIA#2 @ $9F10-$9F1F : user port.  All pins free.  IRQ -> CPU IRQ or NMI
//                         (jumper on real boards).
//
// What this module implements (enough for the KERNAL's needs):
//   - Full 16 register set with read/write side effects per WDC W65C22.
//   - ORA/ORB/DDRA/DDRB with per-bit output-enable on the pa_*/pb_* buses.
//   - T1 counter+latch, one-shot and free-run modes (ACR[7:6]).
//   - T2 counter+latch, one-shot mode (ACR[5]=0) -- KERNAL uses T2 for
//     timing IEC serial-bus transactions.  PB6-pulse mode is NOT modelled.
//   - IFR/IER with the standard "write IER with bit7=1 -> set those bits,
//     bit7=0 -> clear those bits" semantics; IFR bit 7 (any-irq) tracked.
//   - irq_n output asserted (low) while (IFR & IER & 7'h7F) != 0.
//   - Read side effects (clear IFR bit on read of T1C-L / T2C-L / SR / IRA /
//     IRB) trigger exactly once per CPU bus cycle -- the parent gates the
//     CPU enable so address is stable for one cycle per read.
//
// What this module does NOT implement (and probably doesn't need to):
//   - Shift register clocking modes (SR is plain storage).
//   - CA2 / CB2 as outputs.  Handshake auto-set/clear on CA1/CB1 edges.
//   - T1 PB7 output square wave.
//   - T2 PB6 pulse-count mode.
//
// Port-pin contract:
//   pa_in[7:0]  -- external pin levels read INTO the VIA (driven by parent).
//   pa_out[7:0] -- value the VIA is driving OUT  (= ORA).
//   pa_oe[7:0]  -- which bits the VIA is driving (= DDRA).  Parent decides
//                  how to combine these onto a real open-drain bus etc.
//   Same shape for pb_*.
//
// Address decode is done by the parent; this module gets `cs` already
// asserted and the four LSBs of the address in `addr`.
// ============================================================================

module via65c22 (
    input  logic        clk,
    input  logic        reset_n,
    input  logic        ce,        // clock enable for 1x 8 MHz timer ticks

    // CPU bus
    input  logic        cs,        // parent says "this access is mine"
    input  logic        rwn,       // 1 = read, 0 = write
    input  logic        enable,    // CPU enable -- gates one-shot side effects
    input  logic  [3:0] addr,
    input  logic  [7:0] di,        // CPU write data
    output logic  [7:0] do_o,      // CPU read data (combinational)

    // Port A pins (open-drain style: oe says drive, out says level)
    input  logic  [7:0] pa_in,
    output logic  [7:0] pa_out,
    output logic  [7:0] pa_oe,

    // Port B pins
    input  logic  [7:0] pb_in,
    output logic  [7:0] pb_out,
    output logic  [7:0] pb_oe,

    // Control inputs (edges latch into IFR per PCR)
    input  logic        ca1_in,
    input  logic        ca2_in,
    input  logic        cb1_in,
    input  logic        cb2_in,

    // IRQ to CPU (active low)
    output logic        irq_n
);

    // --- Register storage -------------------------------------------------
    logic [7:0] ora_r, orb_r;
    logic [7:0] ddra_r, ddrb_r;
    logic [7:0] t1_lat_lo_r, t1_lat_hi_r;
    logic [7:0] t2_lat_lo_r;
    logic [7:0] sr_r;
    logic [7:0] acr_r, pcr_r;
    logic [7:0] ifr_r;             // bit 7 = computed, bits 6:0 = real
    logic [6:0] ier_r;             // IER has 7 real bits; bit 7 of a read is forced 1

    logic [15:0] t1_cnt_r;
    logic [15:0] t2_cnt_r;
    logic        t1_running_r;     // re-arm in free-run; one-shot disables
    logic        t2_running_r;

    // --- CA1 / CB1 edge detect for IFR auto-set ---------------------------
    logic ca1_q, cb1_q;
    logic ca2_q, cb2_q;
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ca1_q <= 1'b0; cb1_q <= 1'b0;
            ca2_q <= 1'b0; cb2_q <= 1'b0;
        end else begin
            ca1_q <= ca1_in; cb1_q <= cb1_in;
            ca2_q <= ca2_in; cb2_q <= cb2_in;
        end
    end
    wire ca1_active_edge = pcr_r[0] ? (~ca1_q &  ca1_in)   // pos edge
                                    : ( ca1_q & ~ca1_in);  // neg edge
    wire cb1_active_edge = pcr_r[4] ? (~cb1_q &  cb1_in)
                                    : ( cb1_q & ~cb1_in);
    // CA2/CB2 in input modes (PCR[3:1]=00x for CA2, PCR[7:5]=00x for CB2)
    wire ca2_input_mode = (pcr_r[3:1] == 3'b000) || (pcr_r[3:1] == 3'b001);
    wire cb2_input_mode = (pcr_r[7:5] == 3'b000) || (pcr_r[7:5] == 3'b001);
    wire ca2_active_edge = ca2_input_mode &
                           (pcr_r[2] ? (~ca2_q &  ca2_in)
                                     : ( ca2_q & ~ca2_in));
    wire cb2_active_edge = cb2_input_mode &
                           (pcr_r[6] ? (~cb2_q &  cb2_in)
                                     : ( cb2_q & ~cb2_in));

    // --- Read-path data (combinational; no side effects) ------------------
    // IRA/IRB pin read: bits with DDR=1 take ORA/ORB, bits with DDR=0 take pin.
    wire [7:0] pa_read = (ddra_r & ora_r) | (~ddra_r & pa_in);
    wire [7:0] pb_read = (ddrb_r & orb_r) | (~ddrb_r & pb_in);

    logic any_irq;
    assign any_irq = (ifr_r[6:0] & ier_r[6:0]) != 7'h0;
    assign irq_n   = ~any_irq;

    always_comb begin
        case (addr)
            4'h0: do_o = pb_read;
            4'h1: do_o = pa_read;
            4'h2: do_o = ddrb_r;
            4'h3: do_o = ddra_r;
            4'h4: do_o = t1_cnt_r[7:0];
            4'h5: do_o = t1_cnt_r[15:8];
            4'h6: do_o = t1_lat_lo_r;
            4'h7: do_o = t1_lat_hi_r;
            4'h8: do_o = t2_cnt_r[7:0];
            4'h9: do_o = t2_cnt_r[15:8];
            4'hA: do_o = sr_r;
            4'hB: do_o = acr_r;
            4'hC: do_o = pcr_r;
            4'hD: do_o = {any_irq, ifr_r[6:0]};
            4'hE: do_o = {1'b1, ier_r[6:0]};
            4'hF: do_o = pa_read;          // IRA no-handshake
            default: do_o = 8'h00;
        endcase
    end

    // --- Port-pin output drive --------------------------------------------
    assign pa_out = ora_r;
    assign pa_oe  = ddra_r;
    assign pb_out = orb_r;
    assign pb_oe  = ddrb_r;

    // --- Side-effect helpers ----------------------------------------------
    // A CPU read fires for one cpu_clk cycle when enable is high (parent gates
    // VERA stalls etc.); we detect that with cs & rwn & enable.  Similarly
    // for writes.  No edge-detection needed -- the address changes next cycle.
    wire bus_active = cs & enable;
    wire bus_read   = bus_active &  rwn;
    // WRITES ARE NOT GATED BY enable (cpu_rdy): the r65c02 blows through
    // rdy on its write cycle, so a VIA write colliding with a global stall
    // (ext_ram_sdram wf_hi) was silently LOST -- fatal for the I2C bit-bang
    // ($9F01/$9F03), where one dropped edge corrupts the whole transfer
    // (the r65c02-only PRINT DATE$ crash: RTC reads came back $FF).  For
    // the '816 (which HOLDS writes during stalls) an ungated write repeats
    // the same register+data for a few cycles -- every VIA write side
    // effect (timer reload, IFR w1c, IER set/clear, handshake clears) is
    // idempotent under repetition, so this is safe for both CPUs.
    wire bus_write  = cs & ~rwn;

    // --- Main write / read-side-effect / timer state ----------------------
    // ifr_next accumulates auto-set bits (timer expiry, CA1 edges, etc.);
    // explicit IFR/IER writes still happen below.
    logic [6:0] ifr_set;
    always_comb begin
        ifr_set = 7'h0;
        if (ca1_active_edge) ifr_set[1] = 1'b1;
        if (ca2_active_edge) ifr_set[0] = 1'b1;
        if (cb1_active_edge) ifr_set[4] = 1'b1;
        if (cb2_active_edge) ifr_set[3] = 1'b1;
    end

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ora_r        <= 8'h00;
            orb_r        <= 8'h00;
            ddra_r       <= 8'h00;
            ddrb_r       <= 8'h00;
            t1_lat_lo_r  <= 8'h00;
            t1_lat_hi_r  <= 8'h00;
            t2_lat_lo_r  <= 8'h00;
            sr_r         <= 8'h00;
            acr_r        <= 8'h00;
            pcr_r        <= 8'h00;
            ifr_r        <= 8'h00;
            ier_r        <= 7'h00;
            t1_cnt_r     <= 16'hFFFF;
            t2_cnt_r     <= 16'hFFFF;
            t1_running_r <= 1'b0;
            t2_running_r <= 1'b0;
        end else begin
            // ---- Timer 1 ---------------------------------------------------
            // Counts every 8 MHz cycle (gated by ce).  When it reaches 0xFFFF
            // after underflow, raise IFR[6].  Free-run (ACR[6]=1) reloads from
            // latch; one-shot (ACR[6]=0) stops.
            if (ce) begin
                if (t1_cnt_r == 16'h0000) begin
                    if (t1_running_r) ifr_r[6] <= 1'b1;
                    if (acr_r[6]) begin
                        // free-run: reload from latch
                        t1_cnt_r <= {t1_lat_hi_r, t1_lat_lo_r};
                    end else begin
                        // one-shot: keep counting down (wraps to FFFF) but no
                        // further IRQ until next manual reload.
                        t1_running_r <= 1'b0;
                        t1_cnt_r     <= 16'hFFFF;
                    end
                end else begin
                    t1_cnt_r <= t1_cnt_r - 16'h0001;
                end

                // ---- Timer 2 (mode 0 only: timed interrupt) -------------------
                if (t2_cnt_r == 16'h0000) begin
                    if (t2_running_r) begin
                        ifr_r[5]     <= 1'b1;
                        t2_running_r <= 1'b0;   // one-shot
                    end
                    t2_cnt_r <= 16'hFFFF;
                end else begin
                    t2_cnt_r <= t2_cnt_r - 16'h0001;
                end
            end

            // ---- Auto-set IFR bits from CA/CB edges -----------------------
            // (OR with anything that fired this cycle from timers above.)
            if (ifr_set[0]) ifr_r[0] <= 1'b1;
            if (ifr_set[1]) ifr_r[1] <= 1'b1;
            if (ifr_set[3]) ifr_r[3] <= 1'b1;
            if (ifr_set[4]) ifr_r[4] <= 1'b1;

            // ---- CPU write --------------------------------------------------
            if (bus_write) begin
                case (addr)
                    4'h0: begin            // ORB
                        orb_r    <= di;
                        ifr_r[3] <= 1'b0;  // clear CB2
                        ifr_r[4] <= 1'b0;  // clear CB1
                    end
                    4'h1: begin            // ORA (handshake)
                        ora_r    <= di;
                        ifr_r[0] <= 1'b0;
                        ifr_r[1] <= 1'b0;
                    end
                    4'h2: ddrb_r <= di;
                    4'h3: ddra_r <= di;
                    4'h4: t1_lat_lo_r <= di;
                    4'h5: begin            // T1C-H : load latch hi, transfer to counter
                        t1_lat_hi_r <= di;
                        t1_cnt_r    <= {di, t1_lat_lo_r};
                        ifr_r[6]    <= 1'b0;
                        t1_running_r <= 1'b1;
                    end
                    4'h6: t1_lat_lo_r <= di;
                    4'h7: begin
                        t1_lat_hi_r <= di;
                        ifr_r[6]    <= 1'b0;
                    end
                    4'h8: t2_lat_lo_r <= di;
                    4'h9: begin            // T2C-H : start timer 2
                        t2_cnt_r     <= {di, t2_lat_lo_r};
                        ifr_r[5]     <= 1'b0;
                        t2_running_r <= 1'b1;
                    end
                    4'hA: begin
                        sr_r     <= di;
                        ifr_r[2] <= 1'b0;
                    end
                    4'hB: acr_r <= di;
                    4'hC: pcr_r <= di;
                    4'hD: begin            // IFR : write-1-to-clear (bit 7 ignored)
                        ifr_r[6:0] <= ifr_r[6:0] & ~di[6:0];
                    end
                    4'hE: begin            // IER : bit7=1 -> set, bit7=0 -> clear
                        if (di[7]) ier_r[6:0] <= ier_r[6:0] |  di[6:0];
                        else       ier_r[6:0] <= ier_r[6:0] & ~di[6:0];
                    end
                    4'hF: ora_r <= di;     // ORA no-handshake
                    default: ;
                endcase
            end

            // ---- CPU read side effects -------------------------------------
            if (bus_read) begin
                case (addr)
                    4'h0: begin            // IRB read clears CB1/CB2 (per WDC)
                        ifr_r[3] <= 1'b0;
                        ifr_r[4] <= 1'b0;
                    end
                    4'h1: begin            // IRA read clears CA1/CA2
                        ifr_r[0] <= 1'b0;
                        ifr_r[1] <= 1'b0;
                    end
                    4'h4: ifr_r[6] <= 1'b0;  // T1C-L
                    4'h8: ifr_r[5] <= 1'b0;  // T2C-L
                    4'hA: ifr_r[2] <= 1'b0;  // SR
                    default: ;
                endcase
            end

            // bit 7 of ifr_r is the OR-tree -- always recompute combinationally.
            ifr_r[7] <= 1'b0;
        end
    end

endmodule
