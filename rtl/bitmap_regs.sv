//============================================================================
// bitmap_regs.sv  -  CPU register block for the SDRAM bitmap layer ($9F60-$6F).
//
// Small, mode-based API (no width/height/pitch) so software can't build an
// invalid framebuffer, and an ID register so a program can feature-detect and
// fall back on real X16 hardware.  All in the cpu_clk domain; framebuffer DATA
// rides the ext_ram_sdram CPU port (x16.sv muxes it), the palette write feeds
// bitmap_engine, and the BLIT drives a copy engine inside ext_ram_sdram.
//
//   $9F60  CTRL   W: [0]=enable, [2:1]=mode(1=640x480x8bpp,2=640x480x4bpp),
//                    [3]=passthru (VERA sprites/opaque pixels OVER the bitmap)
//                 R: {4'b0, passthru, mode, enable}
//   $9F61  ID     R: MAGIC_ID (feature-detect).  W: ignored
//   $9F62  ADDRL  R/W: framebuffer byte pointer [7:0]  (also the BLIT SOURCE)
//   $9F63  ADDRM  R/W: pointer [15:8]
//   $9F64  ADDRH  R/W: {incr[3:0], pointer[19:16]}  (20-bit ptr -> 1 MB, so a
//                    full-screen 8bpp save-under (307 KB image + 307 KB
//                    scratch = 600 KB) fits).  incr = the DATA auto-increment
//                    STRIDE select, same bit position as VERA's ADDRx_H[7:4]
//                    but indexing a SIGNED table (see `step` below), so no
//                    separate DECR bit is needed.
//   $9F65  DATA   R/W: byte <-> SDRAM @ planar(pointer), pointer += stride
//                    (read-back is the GUI save-under path)
//   $9F66  PALADR W: palette index [7:0] (auto-increments after PALHI)
//   $9F67  PALLO  W: {G[3:0], B[3:0]}  (latched)
//   $9F68  PALHI  W: {----, R[3:0]} -> commits {R,G,B} to palette[idx]; idx++
//   $9F69  BDSTL  W: blit destination byte address [7:0]
//   $9F6A  BDSTM  W: [15:8]
//   $9F6B  BDSTH  W: [19:16]
//   $9F6C  BLENL  W: blit length in bytes [7:0]
//   $9F6D  BLENM  W: [15:8]
//   $9F6E  BLENH  W: [19:16]  (20-bit, up to 1 MB per blit)
//
// The framebuffer address space is 20-bit = 1 MB, reserved in SDRAM (word base
// FB_BASE_WORD, planar).  8bpp displays the first 307,200 B; the rest is
// save-under scratch (+ headroom for future double-buffering / modes).
//   $9F6F  BCTRL  W: bit0=1 starts a blit -> copy BLEN bytes from the ADDR
//                    pointer to BDST (SDRAM->SDRAM, for fast save-under to an
//                    off-screen scratch region).  R: bit0 = busy.
//
// Planar map (matches ext_ram_sdram / bitmap_engine): byte i lives at
// word = FB_BASE_WORD + (i>>1), plane = i[0] (=A24).
//============================================================================
module bitmap_regs #(
    parameter [23:0] FB_BASE_WORD = 24'h800000,
    parameter [7:0]  MAGIC_ID     = 8'hB5
)(
    input  wire        clk,
    input  wire        reset_n,
    input  wire        cs,          // $9F60-$9F6F selected
    input  wire        rwn,         // 1 = read
    input  wire        en,          // access-enable (cpu_rdy) -> single commit
    input  wire  [3:0] addr,        // cpu_a[3:0]
    input  wire  [7:0] di,
    output reg   [7:0] do_o,

    input  wire        master_en,   // OSD master enable (status bit)
    output wire        bmp_enable,  // -> bitmap_engine.enable
    output wire  [1:0] bmp_mode,    // -> bitmap_engine.mode
    output wire        bmp_passthru,// -> x16 mux: VERA opaque pixels over bitmap

    // framebuffer DATA access -> x16 routes through the ext_ram_sdram CPU port
    output wire        fb_wr_sel,   // level: this cycle is a $9F65 write (RAW)
    output wire        fb_rd_sel,   // level: this cycle is a $9F65 read  (RAW)
    output wire [24:0] fb_addr,     // planar byte address (both read and write)

    // palette write -> bitmap_engine palette RAM (pal_clk = this clk)
    output reg         pal_we,
    output reg   [7:0] pal_idx,
    output reg  [11:0] pal_data,

    // blit engine (in ext_ram_sdram) -- src = the DATA pointer
    output reg         blit_start,  // TOGGLE: flips on a start write
    output wire [19:0] blit_src,    // = the DATA pointer (ADDR)
    output reg  [19:0] blit_dst,
    output reg  [19:0] blit_len,
    input  wire        blit_done    // TOGGLE: flips when a blit completes
);
    reg        enable_r;
    reg  [1:0] mode_r;
    reg        passthru_r;
    reg [19:0] ptr;          // 20-bit byte pointer (1 MB; also the blit source)
    reg  [3:0] incr_sel;     // $9F64[7:4]: DATA auto-increment stride select
    reg  [7:0] pal_lo;
    reg  [7:0] cur_idx;

    // DATA auto-increment stride.  Like VERA's VRAM port, the stride is a 4-bit
    // select in ADDRx_H[7:4] -- but the table here is SIGNED, so the two useful
    // directions live in one field instead of needing VERA's extra DECR bit.
    // Index 0 = +1 so a plain "set pointer, stream bytes" loop is the default.
    reg signed [11:0] step;
    always @(*) begin
        case (incr_sel)
            4'h0: step =  12'sd1;      // linear streaming (reset default)
            4'h1: step =  12'sd0;      // hold: 4bpp read-modify-write, re-read
            4'h2: step =  12'sd2;
            4'h3: step =  12'sd4;
            4'h4: step =  12'sd8;
            4'h5: step =  12'sd16;
            4'h6: step =  12'sd32;
            4'h7: step =  12'sd64;
            4'h8: step =  12'sd128;
            4'h9: step =  12'sd256;
            4'hA: step =  12'sd320;    // 4bpp row stride: one pixel down
            4'hB: step =  12'sd640;    // 8bpp row stride: one pixel down
            4'hC: step = -12'sd1;      // reverse streaming / overlapping copy
            4'hD: step = -12'sd2;
            4'hE: step = -12'sd320;    // 4bpp row stride: one pixel up
            4'hF: step = -12'sd640;    // 8bpp row stride: one pixel up
        endcase
    end
    // Sign-extended to the pointer width; wraps mod 1 MB like the pointer does.
    wire [19:0] ptr_next = ptr + {{8{step[11]}}, step};

    // blit-busy tracking: pending is raised on a start write and cleared on the
    // done TOGGLE from the sdram domain (an edge is reliably caught, whereas a
    // busy LEVEL could be missed for a very short blit).
    reg  [1:0] done_s;
    reg        done_d;
    reg        blit_pending;

    assign bmp_enable   = enable_r & master_en;
    assign bmp_mode     = mode_r;
    assign bmp_passthru = passthru_r;
    assign fb_wr_sel  = cs & ~rwn & (addr == 4'd5);
    assign fb_rd_sel  = cs &  rwn & (addr == 4'd5);
    assign fb_addr    = { ptr[0], (FB_BASE_WORD + {5'b0, ptr[19:1]}) };
    assign blit_src   = ptr;

    always @(*) begin
        case (addr)
            4'd0:    do_o = {4'b0, passthru_r, mode_r, enable_r};
            4'd1:    do_o = MAGIC_ID;
            4'd2:    do_o = ptr[7:0];               // pointer readback: where a
            4'd3:    do_o = ptr[15:8];              // streaming loop ended up
            4'd4:    do_o = {incr_sel, ptr[19:16]};
            4'hF:    do_o = {7'b0, blit_pending};   // BCTRL read: busy
            default: do_o = 8'h00;
        endcase
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            enable_r <= 1'b0; mode_r <= 2'd0; passthru_r <= 1'b0;
            ptr <= 20'd0; incr_sel <= 4'h0; pal_lo <= 8'h00;
            cur_idx <= 8'h00; pal_we <= 1'b0; pal_idx <= 8'h00; pal_data <= 12'h000;
            blit_start <= 1'b0; blit_dst <= 20'd0; blit_len <= 20'd0;
            done_s <= 2'b0; done_d <= 1'b0; blit_pending <= 1'b0;
        end else begin
            pal_we <= 1'b0;                   // 1-cycle pulse
            done_s <= {done_s[0], blit_done};
            done_d <= done_s[1];
            if (done_s[1] != done_d) blit_pending <= 1'b0;   // blit finished
            if (cs && en && !rwn) begin
                case (addr)
                    4'd0: begin enable_r <= di[0]; mode_r <= di[2:1];
                                passthru_r <= di[3]; end
                    4'd2: ptr[7:0]   <= di;
                    4'd3: ptr[15:8]  <= di;
                    4'd4: begin ptr[19:16] <= di[3:0];       // ADDRH + stride
                                incr_sel   <= di[7:4]; end
                    4'd5: ptr        <= ptr_next;            // DATA write: advance
                    4'd6: cur_idx    <= di;                  // PALADR: set cursor
                    4'd7: pal_lo     <= di;                  // PALLO: latch {G,B}
                    4'd8: begin                              // PALHI {R} -> commit
                              pal_we   <= 1'b1;
                              pal_idx  <= cur_idx;
                              pal_data <= {di[3:0], pal_lo};
                              cur_idx  <= cur_idx + 8'd1;
                          end
                    4'h9: blit_dst[7:0]   <= di;
                    4'hA: blit_dst[15:8]  <= di;
                    4'hB: blit_dst[19:16] <= di[3:0];
                    4'hC: blit_len[7:0]   <= di;
                    4'hD: blit_len[15:8]  <= di;
                    4'hE: blit_len[19:16] <= di[3:0];
                    4'hF: if (di[0]) begin                   // BCTRL: start blit
                              blit_start   <= ~blit_start;
                              blit_pending <= 1'b1;
                          end
                    default: ;
                endcase
            end else if (cs && en && rwn && (addr == 4'd5)) begin
                ptr <= ptr_next;                             // DATA read: advance
            end
        end
    end
endmodule
