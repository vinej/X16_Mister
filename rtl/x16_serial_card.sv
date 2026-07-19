module x16_serial_card
#(
    parameter integer CLK_HZ          = 8_000_000,
    // X16 serial card UARTs use a 14.7456 MHz crystal with /16 internal
    // prescale, so divisor 1 == 921600 baud and divisor 8 == 115200 baud.
    parameter [15:0] DEFAULT_DIVISOR  = 16'd8
)
(
    input  wire       clk,
    input  wire       reset_n,
    input  wire       cs,
    input  wire       rwn,
    input  wire       enable,
    input  wire [2:0] addr,
    input  wire [7:0] di,
    output reg  [7:0] do_o,

    input  wire       uart_rxd,
    input  wire       uart_cts,
    input  wire       uart_dsr,
    output wire       uart_txd,
    output wire       uart_rts,
    output wire       uart_dtr
);

    localparam integer FIFO_DEPTH = 16;

    reg [7:0] rx_fifo[0:FIFO_DEPTH-1];
    reg [7:0] tx_fifo[0:FIFO_DEPTH-1];
    reg [3:0] rx_wr_ptr, rx_rd_ptr, tx_wr_ptr, tx_rd_ptr;
    reg [4:0] rx_count, tx_count;

    reg [7:0] ier;
    reg [7:0] fcr;
    reg [7:0] lcr;
    reg [7:0] mcr;
    reg [7:0] scr;
    reg [15:0] divisor;

    reg        lsr_overrun;
    reg        lsr_parity;
    reg        lsr_framing;
    reg        lsr_break;

    reg [3:0]  msr_delta;
    reg        cts_prev;
    reg        dsr_prev;
    reg        dcd_prev;

    reg        rx_meta, rx_sync, rx_prev;
    reg        cts_meta, cts_sync;
    reg        dsr_meta, dsr_sync;

    reg        tx_busy;
    reg [15:0] tx_ticks;
    reg [3:0]  tx_bits_left;
    reg [11:0] tx_shift;
    reg        tx_line;

    reg        rx_busy;
    reg [15:0] rx_ticks;
    reg [3:0]  rx_sample_idx;
    reg [7:0]  rx_shift;
    reg [3:0]  rx_data_bits;
    reg [1:0]  rx_stop_bits;
    reg        rx_parity_en;
    reg        rx_parity_even;
    reg        rx_parity_stick;
    reg        rx_calc_parity;

    wire dlab      = lcr[7];
    wire fifo_en   = fcr[0];
    wire [3:0] current_data_bits = 4'd5 + {2'b00, lcr[1:0]};
    wire [1:0] current_stop_bits = ((lcr[2]) && (lcr[1:0] == 2'b00)) ? 2'd2 :
                                   ((lcr[2]) ? 2'd2 : 2'd1);
    wire current_parity_en   = lcr[3];
    wire current_parity_even = lcr[4];
    wire current_parity_stick = lcr[5];

    wire [15:0] effective_divisor = (divisor == 16'd0) ? 16'd1 : divisor;
    wire [15:0] bit_ticks_now = bit_ticks(effective_divisor);

    // MiSTer's internal modem/TCP bridge doesn't reliably present CTS to the
    // guest core, and ROMTERM may refuse to transmit when it sees CTS low.
    // Treat CTS as asserted so the guest can use the built-in modem path
    // without requiring external hardware flow-control wiring.
    wire cts_curr = 1'b1;
    wire dsr_curr = 1'b1;
    wire ri_curr  = 1'b0;
    wire dcd_curr = 1'b1;

    wire [7:0] lsr_value = {
        1'b0,
        (~tx_busy && (tx_count == 5'd0)),
        (tx_count == 5'd0),
        lsr_break,
        lsr_framing,
        lsr_parity,
        lsr_overrun,
        (rx_count != 5'd0)
    };

    wire [7:0] msr_value = {
        dcd_curr,
        ri_curr,
        dsr_curr,
        cts_curr,
        msr_delta
    };

    wire [7:0] iir_value = fifo_en ? 8'hC1 : 8'h01;

    assign uart_txd = tx_line;
    assign uart_rts = mcr[1];
    assign uart_dtr = mcr[0];

    function [15:0] bit_ticks;
        input [15:0] div;
        reg [47:0] scaled;
        begin
            // Match the X16 dual-UART card timing:
            // baud = 14.7456 MHz / (16 * divisor) = 921600 / divisor
            // so clocks/bit = CLK_HZ * divisor / 921600.
            scaled = (div * CLK_HZ) + 48'd460800;
            scaled = scaled / 48'd921600;
            bit_ticks = scaled[15:0];
            if (scaled == 48'd0) bit_ticks = 16'd1;
        end
    endfunction

    function parity_bit;
        input [7:0] data;
        input [3:0] bits;
        input       even_parity;
        input       stick_parity;
        reg         raw;
        integer     i;
        begin
            raw = 1'b0;
            for (i = 0; i < 8; i = i + 1)
                if (i < bits) raw = raw ^ data[i];

            if (stick_parity) parity_bit = ~even_parity;
            else if (even_parity) parity_bit = ~raw;
            else                  parity_bit = raw;
        end
    endfunction

    task push_rx;
        input [7:0] data;
        begin
            if (rx_count != FIFO_DEPTH) begin
                rx_fifo[rx_wr_ptr] <= data;
                rx_wr_ptr <= rx_wr_ptr + 4'd1;
                rx_count  <= rx_count + 5'd1;
            end else begin
                lsr_overrun <= 1'b1;
            end
        end
    endtask

    task clear_rx_fifo;
        integer i;
        begin
            rx_wr_ptr <= 4'd0;
            rx_rd_ptr <= 4'd0;
            rx_count  <= 5'd0;
            for (i = 0; i < FIFO_DEPTH; i = i + 1)
                rx_fifo[i] <= 8'h00;
        end
    endtask

    task clear_tx_fifo;
        integer i;
        begin
            tx_wr_ptr <= 4'd0;
            tx_rd_ptr <= 4'd0;
            tx_count  <= 5'd0;
            for (i = 0; i < FIFO_DEPTH; i = i + 1)
                tx_fifo[i] <= 8'h00;
        end
    endtask

    always @(*) begin
        do_o = 8'hFF;
        case (addr)
            3'd0: do_o = dlab ? divisor[7:0] : ((rx_count != 5'd0) ? rx_fifo[rx_rd_ptr] : 8'h00);
            3'd1: do_o = dlab ? divisor[15:8] : ier;
            3'd2: do_o = iir_value;
            3'd3: do_o = lcr;
            3'd4: do_o = mcr;
            3'd5: do_o = lsr_value;
            3'd6: do_o = msr_value;
            3'd7: do_o = scr;
            default: do_o = 8'hFF;
        endcase
    end

    always @(posedge clk or negedge reset_n) begin
        reg [7:0] tx_byte;
        reg [3:0] frame_bits;
        reg [11:0] frame_shift;
        reg parity_expected;

        if (!reset_n) begin
            rx_wr_ptr      <= 4'd0;
            rx_rd_ptr      <= 4'd0;
            tx_wr_ptr      <= 4'd0;
            tx_rd_ptr      <= 4'd0;
            rx_count       <= 5'd0;
            tx_count       <= 5'd0;
            ier            <= 8'h00;
            fcr            <= 8'h00;
            lcr            <= 8'h03;
            mcr            <= 8'h00;
            scr            <= 8'h00;
            divisor        <= DEFAULT_DIVISOR;
            lsr_overrun    <= 1'b0;
            lsr_parity     <= 1'b0;
            lsr_framing    <= 1'b0;
            lsr_break      <= 1'b0;
            msr_delta      <= 4'h0;
            cts_prev       <= 1'b0;
            dsr_prev       <= 1'b0;
            dcd_prev       <= 1'b0;
            rx_meta        <= 1'b1;
            rx_sync        <= 1'b1;
            rx_prev        <= 1'b1;
            cts_meta       <= 1'b0;
            cts_sync       <= 1'b0;
            dsr_meta       <= 1'b0;
            dsr_sync       <= 1'b0;
            tx_busy        <= 1'b0;
            tx_ticks       <= 16'd0;
            tx_bits_left   <= 4'd0;
            tx_shift       <= 12'hFFF;
            tx_line        <= 1'b1;
            rx_busy        <= 1'b0;
            rx_ticks       <= 16'd0;
            rx_sample_idx  <= 4'd0;
            rx_shift       <= 8'h00;
            rx_data_bits   <= 4'd8;
            rx_stop_bits   <= 2'd1;
            rx_parity_en   <= 1'b0;
            rx_parity_even <= 1'b0;
            rx_parity_stick <= 1'b0;
            rx_calc_parity <= 1'b0;
        end else begin
            rx_meta  <= uart_rxd;
            rx_sync  <= rx_meta;
            rx_prev  <= rx_sync;
            cts_meta <= uart_cts;
            cts_sync <= cts_meta;
            dsr_meta <= uart_dsr;
            dsr_sync <= dsr_meta;

            if (cts_sync != cts_prev) begin
                msr_delta[0] <= 1'b1;
                cts_prev <= cts_sync;
            end
            if (dsr_sync != dsr_prev) begin
                msr_delta[1] <= 1'b1;
                dsr_prev <= dsr_sync;
            end
            if (dcd_curr != dcd_prev) begin
                msr_delta[3] <= 1'b1;
                dcd_prev <= dcd_curr;
            end

            if (cs && enable && !rwn) begin
                case (addr)
                    3'd0: if (dlab) begin
                              divisor[7:0] <= di;
                          end else if (tx_count != FIFO_DEPTH) begin
                              tx_fifo[tx_wr_ptr] <= di;
                              tx_wr_ptr <= tx_wr_ptr + 4'd1;
                              tx_count  <= tx_count + 5'd1;
                          end
                    3'd1: if (dlab) divisor[15:8] <= di;
                          else      ier <= di;
                    3'd2: begin
                              fcr <= {2'b00, di[5:0]};
                              if (di[1]) begin
                                  rx_wr_ptr <= 4'd0;
                                  rx_rd_ptr <= 4'd0;
                                  rx_count  <= 5'd0;
                              end
                              if (di[2]) begin
                                  tx_wr_ptr <= 4'd0;
                                  tx_rd_ptr <= 4'd0;
                                  tx_count  <= 5'd0;
                              end
                          end
                    3'd3: lcr <= di;
                    3'd4: mcr <= di;
                    3'd7: scr <= di;
                    default: ;
                endcase
            end

            if (cs && enable && rwn) begin
                case (addr)
                    3'd0: if (!dlab && (rx_count != 5'd0)) begin
                              rx_rd_ptr <= rx_rd_ptr + 4'd1;
                              rx_count  <= rx_count - 5'd1;
                              lsr_overrun <= 1'b0;
                              lsr_parity  <= 1'b0;
                              lsr_framing <= 1'b0;
                              lsr_break   <= 1'b0;
                          end
                    3'd5: begin
                              lsr_overrun <= 1'b0;
                              lsr_parity  <= 1'b0;
                              lsr_framing <= 1'b0;
                              lsr_break   <= 1'b0;
                          end
                    3'd6: msr_delta <= 4'h0;
                    default: ;
                endcase
            end

            if (!tx_busy && (tx_count != 5'd0)) begin
                tx_byte    = tx_fifo[tx_rd_ptr];
                frame_shift = 12'hFFF;
                frame_shift[0] = 1'b0;
                frame_bits = 4'd1 + current_data_bits + current_stop_bits;
                frame_shift[8:1] = tx_byte;
                if (current_parity_en) begin
                    parity_expected = parity_bit(tx_byte, current_data_bits,
                                                 current_parity_even,
                                                 current_parity_stick);
                    frame_shift[1 + current_data_bits] = parity_expected;
                    frame_bits = frame_bits + 4'd1;
                end

                tx_shift     <= frame_shift;
                tx_bits_left <= frame_bits;
                tx_ticks     <= bit_ticks_now;
                tx_line      <= 1'b0;
                tx_busy      <= 1'b1;
                tx_rd_ptr    <= tx_rd_ptr + 4'd1;
                tx_count     <= tx_count - 5'd1;
            end else if (tx_busy) begin
                if (tx_ticks != 16'd0) begin
                    tx_ticks <= tx_ticks - 16'd1;
                end else begin
                    tx_shift <= {1'b1, tx_shift[11:1]};
                    tx_bits_left <= tx_bits_left - 4'd1;
                    tx_ticks <= bit_ticks_now;
                    tx_line <= tx_shift[1];

                    if (tx_bits_left == 4'd1) begin
                        tx_busy <= 1'b0;
                        tx_line <= 1'b1;
                    end
                end
            end

            if (!rx_busy) begin
                if (rx_prev && !rx_sync) begin
                    rx_busy         <= 1'b1;
                    rx_ticks        <= bit_ticks_now + (bit_ticks_now >> 1);
                    rx_sample_idx   <= 4'd0;
                    rx_shift        <= 8'h00;
                    rx_data_bits    <= current_data_bits;
                    rx_stop_bits    <= current_stop_bits;
                    rx_parity_en    <= current_parity_en;
                    rx_parity_even  <= current_parity_even;
                    rx_parity_stick <= current_parity_stick;
                    rx_calc_parity  <= 1'b0;
                end
            end else if (rx_ticks != 16'd0) begin
                rx_ticks <= rx_ticks - 16'd1;
            end else begin
                if (rx_sample_idx < rx_data_bits) begin
                    rx_shift[rx_sample_idx] <= rx_sync;
                    rx_calc_parity <= rx_calc_parity ^ rx_sync;
                end else if (rx_sample_idx < (rx_data_bits + rx_parity_en)) begin
                    parity_expected = rx_parity_stick ? ~rx_parity_even :
                                      (rx_parity_even ? ~rx_calc_parity : rx_calc_parity);
                    if (rx_sync != parity_expected) lsr_parity <= 1'b1;
                end else if (!rx_sync) begin
                    lsr_framing <= 1'b1;
                end

                if (rx_sample_idx == (rx_data_bits + rx_parity_en + rx_stop_bits - 4'd1)) begin
                    if (rx_count != FIFO_DEPTH) begin
                        rx_fifo[rx_wr_ptr] <= rx_shift;
                        rx_wr_ptr <= rx_wr_ptr + 4'd1;
                        rx_count  <= rx_count + 5'd1;
                    end else begin
                        lsr_overrun <= 1'b1;
                    end
                    rx_busy <= 1'b0;
                end else begin
                    rx_sample_idx <= rx_sample_idx + 4'd1;
                    rx_ticks <= bit_ticks_now;
                end
            end
        end
    end

endmodule
