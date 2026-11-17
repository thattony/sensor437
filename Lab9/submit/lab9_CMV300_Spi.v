`timescale 1ns / 1ps
//============================================================================
// CMV300_Spi.v - THIS IS A FILE YOU WRITE (2 of 3). SPI master for the CMV300
// register interface.
//
// WHAT IT MUST DO (CMV300.pdf section 3.9 Figures 7-9):
//   * One frame = 16 bits, MSB first: control bit (1 = write, 0 = read),
//     7 address bits, 8 data bits.
//   * The SENSOR samples SPI_IN on the RISING edge of SPI_CLK and launches
//     SPI_OUT on the FALLING edge - so YOUR master changes SPI_IN on falling
//     edges and samples SPI_OUT on rising edges.
//   * SPI_EN high frames the transfer: raise it (with the control bit already
//     on SPI_IN) half a period before the first rising edge, keep it high one
//     period after the last one. SPI_CLK idles low.
//   * SPI_CLK <= 10 MHz (HALF = 5 gives 100.8 / 10 = 10.08 MHz - fine).
//   * burst = 1..3: that many EXTRA frames back to back with SPI_EN held high
//     and addresses addr, addr+1, ... (Figure 8; the temperature registers
//     78/79 are read this way). Read byte i of the burst lands in
//     rd_data[8i+7:8i]. A burst write repeats the same data byte.
//   * done = one-clock pulse when the transfer completes; busy while it runs.
//
// The PORT LIST IS FROZEN (INTERFACE.md).
//
// HOW IT IS BUILT: three states (IDLE, SHIFT, FINISH) plus a clock divider,
// a bit counter, a frame counter and two shift registers. SPI_OUT is sampled
// on the RISING edge of SPI_CLK: the sensor launches each bit on the falling
// edge, so by the next rising edge it has been stable for half a period.
//============================================================================
module CMV300_Spi #(
    parameter integer HALF = 5          // okClk cycles per half SPI_CLK period
)(
    input  wire        clk,             // okClk 100.8 MHz
    input  wire        rst,
    input  wire        go,              // one-clock pulse (ignore while busy)
    input  wire        wr,              // 1 = write, 0 = read
    input  wire [6:0]  addr,
    input  wire [7:0]  wr_data,
    input  wire [1:0]  burst,           // number of frames - 1
    output reg         busy,
    output reg         done,            // one-clock pulse at the end of a transfer
    output reg  [31:0] rd_data,         // byte i of a read burst in [8i+7:8i]
    output reg         SPI_CLK,
    output reg         SPI_EN,
    output reg         SPI_IN,          // MOSI
    input  wire        SPI_OUT          // MISO
);
    // ---- States ------------------------------------------------------------
    localparam [1:0] S_IDLE   = 2'd0,   // bus inactive, waiting for go
                     S_SHIFT  = 2'd1,   // clocking frames out/in, SPI_EN high
                     S_FINISH = 2'd2;   // SPI_CLK low, SPI_EN held one more period

    reg [1:0]  state;

    // ---- Datapath registers ------------------------------------------------
    reg [7:0]  div_cnt;     // okClk cycles inside the current half period
    reg [3:0]  bit_cnt;     // bit position inside the current frame (0..15)
    reg [1:0]  frames_left; // extra frames still to send after this one
    reg [1:0]  frame_idx;   // which byte of rd_data this frame fills
    reg [15:0] sh_out;      // {wr, addr, wr_data}; MSB drives SPI_IN
    reg [7:0]  sh_in;       // bits sampled from SPI_OUT
    reg        wr_r;        // inputs latched at go
    reg [6:0]  addr_r;      // address of the CURRENT frame (increments in a burst)
    reg [7:0]  wr_data_r;

    wire half_tick = (div_cnt == HALF - 1);        // end of a half period
    wire full_tick = (div_cnt == 2 * HALF - 1);    // end of a full period (S_FINISH)

    initial begin
        state = S_IDLE; busy = 1'b0; done = 1'b0; rd_data = 32'd0;
        SPI_CLK = 1'b0; SPI_EN = 1'b0; SPI_IN = 1'b0;
        div_cnt = 8'd0; bit_cnt = 4'd0; frames_left = 2'd0; frame_idx = 2'd0;
        sh_out = 16'd0; sh_in = 8'd0; wr_r = 1'b0; addr_r = 7'd0; wr_data_r = 8'd0;
    end

    always @(posedge clk) begin
        done <= 1'b0;                               // default: one-clock pulse

        if (rst) begin
            state   <= S_IDLE;
            busy    <= 1'b0;
            SPI_CLK <= 1'b0;
            SPI_EN  <= 1'b0;
            SPI_IN  <= 1'b0;
            div_cnt <= 8'd0;
        end else begin
            case (state)
                // ------------------------------------------------------------
                S_IDLE: begin
                    SPI_CLK <= 1'b0;
                    SPI_EN  <= 1'b0;
                    busy    <= 1'b0;
                    SPI_IN  <= 1'b0;
                    if (go) begin
                        // latch the request so the host may change its wires mid-transfer
                        wr_r        <= wr;
                        addr_r      <= addr;
                        wr_data_r   <= wr_data;
                        frames_left <= burst;
                        frame_idx   <= 2'd0;
                        bit_cnt     <= 4'd0;
                        sh_out      <= {wr, addr, wr_data};
                        SPI_IN      <= wr;          // control bit is on the pin when SPI_EN rises
                        rd_data     <= 32'd0;
                        div_cnt     <= 8'd0;
                        SPI_EN      <= 1'b1;        // first rising edge comes HALF clocks later
                        busy        <= 1'b1;
                        state       <= S_SHIFT;
                    end
                end

                // ------------------------------------------------------------
                S_SHIFT: begin
                    if (!half_tick) begin
                        div_cnt <= div_cnt + 8'd1;
                    end else begin
                        div_cnt <= 8'd0;
                        if (!SPI_CLK) begin
                            // RISING edge: the sensor samples SPI_IN, we sample SPI_OUT
                            SPI_CLK <= 1'b1;
                            sh_in   <= {sh_in[6:0], SPI_OUT};   // the last 8 samples of a frame are the data
                        end else begin
                            // FALLING edge: change SPI_IN, advance the bit counter
                            SPI_CLK <= 1'b0;
                            if (bit_cnt != 4'd15) begin
                                bit_cnt <= bit_cnt + 4'd1;
                                sh_out  <= {sh_out[14:0], 1'b0};
                                SPI_IN  <= sh_out[14];          // the bit AFTER the one just clocked
                            end else begin
                                // end of a 16-bit frame
                                if (!wr_r) rd_data[{frame_idx, 3'b000} +: 8] <= sh_in;
                                if (frames_left != 2'd0) begin
                                    // burst: next frame, SPI_EN stays high, address + 1
                                    frames_left <= frames_left - 2'd1;
                                    frame_idx   <= frame_idx + 2'd1;
                                    addr_r      <= addr_r + 7'd1;
                                    bit_cnt     <= 4'd0;
                                    sh_out      <= {wr_r, addr_r + 7'd1, wr_data_r};
                                    SPI_IN      <= wr_r;
                                end else begin
                                    SPI_IN <= 1'b0;
                                    state  <= S_FINISH;
                                end
                            end
                        end
                    end
                end

                // ------------------------------------------------------------
                S_FINISH: begin
                    // SPI_CLK is low; hold SPI_EN high for one full period
                    if (!full_tick) begin
                        div_cnt <= div_cnt + 8'd1;
                    end else begin
                        div_cnt <= 8'd0;
                        SPI_EN  <= 1'b0;
                        SPI_IN  <= 1'b0;
                        busy    <= 1'b0;
                        done    <= 1'b1;                // one clock: the default above clears it
                        state   <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
