`timescale 1ns/1ps
// ============================================================================
// tb_bmpregs.v -- bitmap_regs decode / planar-address / palette unit test.
// ============================================================================
module tb_bmpregs;
    reg clk = 0; always #62.5 clk = ~clk;   // 8 MHz
    reg res_n = 0;

    reg        cs = 0, rwn = 1, en = 1;
    reg  [3:0] addr = 0;
    reg  [7:0] di = 0;
    wire [7:0] do_o;
    reg        master_en = 1;
    wire       bmp_enable;
    wire [1:0] bmp_mode;
    wire       bmp_passthru;
    wire       fb_wr_sel, fb_rd_sel;
    wire [24:0] fb_wr_addr;
    wire       pal_we;
    wire [7:0] pal_idx;
    wire [11:0] pal_data;

    bitmap_regs dut (
        .clk(clk), .reset_n(res_n), .cs(cs), .rwn(rwn), .en(en),
        .addr(addr), .di(di), .do_o(do_o), .master_en(master_en),
        .bmp_enable(bmp_enable), .bmp_mode(bmp_mode), .bmp_passthru(bmp_passthru),
        .fb_wr_sel(fb_wr_sel), .fb_rd_sel(fb_rd_sel), .fb_addr(fb_wr_addr),
        .pal_we(pal_we), .pal_idx(pal_idx), .pal_data(pal_data),
        .blit_start(), .blit_src(), .blit_dst(), .blit_len(), .blit_done(1'b0)
    );

    integer errs = 0;
    task chk(input cond, input [255:0] msg);
        if (!cond) begin errs = errs + 1; $display("[BR  ] FAIL: %0s", msg); end
    endtask

    task wr(input [3:0] a, input [7:0] d);
        begin @(negedge clk); cs<=1; rwn<=0; en<=1; addr<=a; di<=d;
              @(posedge clk); @(negedge clk); cs<=0; en<=0; rwn<=1; end
    endtask
    task rd(input [3:0] a);
        begin @(negedge clk); cs<=1; rwn<=1; en<=1; addr<=a; #1; end
    endtask
    task dat_wr(input [7:0] d);          // a $9F65 DATA write (advances the ptr)
        begin wr(4'd5, d); end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        res_n = 1;
        @(negedge clk);

        // ID + CTRL
        rd(4'd1); chk(do_o === 8'hB5, "ID != B5");
        wr(4'd0, 8'b0000_0101);              // enable=1, mode=2 (bits[2:1]=10)
        chk(bmp_enable === 1'b1, "enable");
        chk(bmp_mode   === 2'd2, "mode");
        rd(4'd0); chk(do_o === 8'b0000_0101, "CTRL readback");

        // master_en gating
        master_en = 0; #1;
        chk(bmp_enable === 1'b0, "master_en gate");
        master_en = 1; #1;

        // pointer + planar framebuffer write.  ptr=5 -> {A24=1, 0x800000+2}
        wr(4'd2, 8'h05); wr(4'd3, 8'h00); wr(4'd4, 8'h00);
        @(negedge clk); cs<=1; rwn<=0; en<=1; addr<=4'd5; di<=8'hAB; #1;
        chk(fb_wr_sel  === 1'b1,          "fb_wr_sel");
        chk(fb_wr_addr === 25'h1800002,   "fb_wr_addr planar (ptr=5)");
        @(posedge clk); @(negedge clk); cs<=0; en<=0; rwn<=1;
        chk(dut.ptr === 19'd6, "ptr auto-increment");

        // next DATA write should target ptr=6 -> {A24=0, 0x800000+3}
        @(negedge clk); cs<=1; rwn<=0; en<=1; addr<=4'd5; di<=8'hCD; #1;
        chk(fb_wr_addr === 25'h0800003, "fb_wr_addr planar (ptr=6)");
        @(posedge clk); @(negedge clk); cs<=0; en<=0; rwn<=1;

        // ---- auto-increment stride ($9F64[7:4], signed table) ----
        // ADDRH carries {incr[3:0], ptr[19:16]}: one store sets both.
        wr(4'd2, 8'h00); wr(4'd3, 8'h10); wr(4'd4, 8'hB0);  // ptr=$01000, incr=B (+640)
        chk(dut.ptr === 20'h01000, "ADDRH ptr field");
        chk(dut.incr_sel === 4'hB,  "ADDRH incr field");
        dat_wr(8'h11);
        chk(dut.ptr === 20'h01000 + 20'd640, "stride +640 (write)");
        dat_wr(8'h22);
        chk(dut.ptr === 20'h01000 + 20'd1280, "stride +640 twice");

        // a DATA READ must advance by the same stride
        rd(4'd5); @(posedge clk); @(negedge clk); cs<=0; en<=0; rwn<=1;
        chk(dut.ptr === 20'h01000 + 20'd1920, "stride +640 (read)");

        // hold (incr=1): pointer must NOT move -- the 4bpp read-modify-write case
        wr(4'd4, 8'h10);                     // incr=1 (0), ptr[19:16]=0
        chk(dut.ptr === 20'h01780, "ADDRH keeps low pointer bits");
        dat_wr(8'h33);
        chk(dut.ptr === 20'h01780, "stride 0 holds the pointer");
        rd(4'd5); @(posedge clk); @(negedge clk); cs<=0; en<=0; rwn<=1;
        chk(dut.ptr === 20'h01780, "stride 0 holds on read too");

        // negative stride (incr=F = -640) walks back up the screen
        wr(4'd4, 8'hF0);
        dat_wr(8'h44);
        chk(dut.ptr === 20'h01780 - 20'd640, "stride -640");

        // -1 and the 1 MB wrap (20-bit two's complement)
        wr(4'd2, 8'h00); wr(4'd3, 8'h00); wr(4'd4, 8'hC0);  // ptr=0, incr=C (-1)
        dat_wr(8'h55);
        chk(dut.ptr === 20'hFFFFF, "stride -1 wraps to the top of the 1 MB space");

        // default stride after a plain ADDRH store is +1 (index 0)
        wr(4'd2, 8'h40); wr(4'd3, 8'h00); wr(4'd4, 8'h00);
        chk(dut.incr_sel === 4'h0, "incr defaults to 0 (=+1)");
        dat_wr(8'h66);
        chk(dut.ptr === 20'h00041, "stride +1 default");

        // pointer readback ($9F62-64) -- VERA-style, so a loop can see where it got
        rd(4'd2); chk(do_o === 8'h41, "ADDRL readback");
        rd(4'd3); chk(do_o === 8'h00, "ADDRM readback");
        wr(4'd4, 8'hA3);                     // incr=A (+320), ptr[19:16]=3
        rd(4'd4); chk(do_o === 8'hA3, "ADDRH readback {incr, ptr[19:16]}");
        dat_wr(8'h77);
        chk(dut.ptr === 20'h30041 + 20'd320, "stride +320");

        // palette write: idx=0x10, {G=3,B=4}, R=7 -> pal_data=0x734, idx++ ->0x11
        wr(4'd6, 8'h10);                     // PALADR
        wr(4'd7, 8'h34);                     // PALLO {G,B}
        @(negedge clk); cs<=1; rwn<=0; en<=1; addr<=4'd8; di<=8'h07; // PALHI {R}
        @(posedge clk); #1;
        chk(pal_we   === 1'b1,     "pal_we pulse");
        chk(pal_idx  === 8'h10,    "pal_idx");
        chk(pal_data === 12'h734,  "pal_data {R,G,B}");
        @(negedge clk); cs<=0; en<=0; rwn<=1;
        chk(dut.cur_idx === 8'h11, "palette cursor auto-increment");

        if (errs == 0) $display("[BR  ] ALL OK");
        else           $display("[BR  ] FAILED (%0d)", errs);
        $finish;
    end
    initial begin #200000; $display("[BR  ] TIMEOUT"); $finish; end
endmodule
