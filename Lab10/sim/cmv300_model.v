`timescale 1ns / 1ps
//============================================================================
// cmv300_model.v - behavioural model of the CMV300 for simulation
//   (CMV300.pdf: SPI §3.9 p.15-16, register defaults §6 p.36-38, parallel
//   output timing §4.2.1 Figures 26-27 p.23, LVDS output §4.1 p.18-22,
//   reset §3.8 p.15).
//
//   * 128 registers with the datasheet power-on defaults; a falling edge on
//     SYS_RES_N restores them. Prints every SPI write and read.
//   * SPI slave: samples SPI_IN on the rising edge of SPI_CLK while SPI_EN is
//     high, launches SPI_OUT on the falling edge, parses 16-bit frames
//     {rw, addr[6:0], data[7:0]} back to back while SPI_EN stays high (burst).
//     SPI_OUT is 0 while SPI_EN is low.
//   * Video: a rising edge on FRAME_REQ (sensor out of reset, CLK_IN running)
//     produces Number_frames (regs 55/56) frames of Number_lines (regs 1/2)
//     rows. A FRAME_REQ during a running sequence is served right after it.
//     - parallel (enable_lvds = 0): LVAL high for 324 pixels + 1 INVALID clock
//       (DVAL low) + 324 pixels, LVAL low for one clock between rows, an idle
//       gap between frames. Data changes on the rising edge of CLK_OUT (the
//       receiver samples on the falling edge). CLK_OUT = CLK_IN when reg 69 = 9
//       and the sensor is out of reset, else 0. Pixel value = (col + 3*row) mod 1024.
//     - LVDS (enable_lvds = 1, 4-output mode): every CLK_IN period one 12-bit
//       word per lane, LSB first, 12 bits per period on a DDR clock whose high
//       phase carries bit 0 (Fig. 15); lanes 1-4 lag the clock by 250 / 930 /
//       450 / 1050 ps, the control lane by 100 ps (Fig. 24). Bursts of 324
//       words carry two rows (Fig. 19: lane 1 = row 2b pixels 0-323, lane 2 =
//       row 2b pixels 324-647, lanes 3/4 = row 2b+1), one overhead word
//       between bursts (DVAL = LVAL = 0, training pattern on the data lanes),
//       FVAL high for the frame; idle: control word 0x080, training pattern
//       (regs 61/62) on the data lanes (Fig. 22/25). The 12-bit word is
//       ((col + 3*row) mod 1024) << 2 in 10-bit mode (reg 68 = 1, Fig. 14:
//       the two LSBs are zero) and the same | (row mod 4) in 12-bit mode.
//   * Temperature registers 78/79 hold 5220 (0x1464) to test the burst read.
//============================================================================
module cmv300_model (
    input  wire       clk_in,
    input  wire       sys_res_n,
    input  wire       enable_lvds,
    input  wire       frame_req,
    input  wire       spi_clk,
    input  wire       spi_en,
    input  wire       spi_in,
    output reg        spi_out,
    output wire       clk_out,
    output reg  [9:0] d,
    output reg        lval,
    output reg        dval,
    // LVDS outputs (§4.1)
    output reg        lv_clk_p, lv_clk_n,
    output reg  [3:0] lv_d_p, lv_d_n,
    output reg        lv_ctrl_p, lv_ctrl_n
);
    reg [7:0] regs [0:127];
    integer i;

    task set_defaults;
        begin
            for (i = 0; i < 128; i = i + 1) regs[i] = 8'd0;
            regs[1]  = 232; regs[2]  = 1;        // Number_lines = 488
            regs[39] = 1;
            regs[42] = 232; regs[43] = 1;        // Exp_time = 488
            regs[45] = 232; regs[46] = 1;
            regs[54] = 1;   regs[55] = 1;        // Nr_slopes, Number_frames = 1
            regs[58] = 4;   regs[61] = 85;  regs[63] = 12;  regs[64] = 2;
            regs[77] = 128; regs[78] = 8'h64; regs[79] = 8'h14;   // temperature 0x1464 = 5220
            regs[81] = 126; regs[82] = 128; regs[83] = 155; regs[85] = 3;
            regs[99] = 255; regs[100] = 96;
            regs[101] = 136; regs[102] = 136; regs[103] = 136;
            regs[104] = 96;  regs[105] = 96;  regs[106] = 64;
            regs[107] = 96;  regs[108] = 96;  regs[109] = 96;  regs[110] = 96;
            regs[111] = 64;  regs[112] = 64;  regs[113] = 64;  regs[114] = 64;
            regs[115] = 96;  regs[116] = 96;  regs[117] = 96;  regs[127] = 255;
        end
    endtask

    initial begin
        set_defaults;
        spi_out = 1'b0; d = 10'd0; lval = 1'b0; dval = 1'b0;
        lv_clk_p = 1'b0; lv_clk_n = 1'b1; lv_d_p = 4'd0; lv_d_n = 4'hF; lv_ctrl_p = 1'b0; lv_ctrl_n = 1'b1;
    end
    always @(negedge sys_res_n) begin
        set_defaults;
        $display("%0t  CMV300 model: SYS_RES_N fell - registers back to defaults", $time);
    end

    //------------------------------------------------------------------------
    // SPI slave
    //------------------------------------------------------------------------
    reg [4:0]  bitc = 5'd0;          // 0..15 inside a frame
    reg [15:0] sh   = 16'd0;
    reg        rw   = 1'b0;
    reg [6:0]  addr = 7'd0;
    reg [7:0]  rdv  = 8'd0;

    always @(negedge spi_en) begin bitc <= 5'd0; spi_out <= 1'b0; end

    always @(posedge spi_clk) if (spi_en) begin
        sh <= {sh[14:0], spi_in};
        if (bitc == 5'd7) begin              // control bit + 7 address bits received
            rw   <= sh[6];
            addr <= {sh[5:0], spi_in};
            rdv  <= regs[{sh[5:0], spi_in}];
            if (!sh[6]) $display("%0t  CMV300 model: SPI read  reg %0d -> %0d", $time, {sh[5:0], spi_in}, regs[{sh[5:0], spi_in}]);
        end
        if (bitc == 5'd15) begin
            if (rw) begin
                regs[addr] = {sh[6:0], spi_in};
                $display("%0t  CMV300 model: SPI write reg %0d <= %0d", $time, addr, {sh[6:0], spi_in});
            end
            bitc <= 5'd0;
        end else bitc <= bitc + 5'd1;
    end

    // data bits launched on the falling edge after the address (bits 8..15 of a read frame)
    always @(negedge spi_clk) if (spi_en) begin
        if (!rw && bitc >= 5'd8 && bitc <= 5'd15) spi_out <= rdv[5'd15 - bitc];
        else spi_out <= 1'b0;
    end

    //------------------------------------------------------------------------
    // Video
    //------------------------------------------------------------------------
    assign clk_out = (regs[69] == 8'd9 && sys_res_n) ? clk_in : 1'b0;

    reg busy = 1'b0, req_pending = 1'b0;
    integer f, r, c, nframes, nlines;

    // LVDS words for the next CLK_IN period (0..3 = Out1..Out4, 4 = control); nonblocking updates
    // from the video process, read by the serialiser at the following clock edge
    reg [11:0] lv_word [0:4];
    wire [11:0] train = {regs[62][3:0], regs[61]};
    function [11:0] px12(input integer row, input integer col);
        reg [9:0] v;
        begin
            v = (col + 3*row) & 10'h3FF;
            px12 = (regs[68] == 8'd1) ? {v, 2'b00} : {v, row[1:0]};
        end
    endfunction
    initial begin lv_word[0] = 12'h055; lv_word[1] = 12'h055; lv_word[2] = 12'h055; lv_word[3] = 12'h055; lv_word[4] = 12'h080; end
    task lv_idle; begin lv_word[0] <= train; lv_word[1] <= train; lv_word[2] <= train; lv_word[3] <= train; lv_word[4] <= 12'h080; end endtask

    // a request is remembered while a sequence is running and served right after it (the real
    // sensor accepts a FRAME_REQ as soon as it is idle; ignoring one would stall a paced ring)
    always @(posedge frame_req) if (sys_res_n) req_pending = 1'b1;
    always @(negedge sys_res_n) req_pending = 1'b0;

    always begin
        wait (req_pending && !busy);
        req_pending = 1'b0;
        busy = 1'b1;
        nframes = {regs[56], regs[55]};
        nlines  = {regs[2], regs[1]};
        $display("%0t  CMV300 model: FRAME_REQ -> %0d frame(s) of %0d rows (%0s)", $time, nframes, nlines, enable_lvds ? "LVDS" : "parallel");
        for (f = 0; f < nframes; f = f + 1) begin
            repeat (300) @(posedge clk_in);                 // exposure + FOT (shortened)
            if (enable_lvds) begin
                for (r = 0; r < nlines; r = r + 2) begin
                    // the real sensor raises FVAL on the same word as the first pixel of the frame (measured on
                    // hardware; Fig. 22 draws a leading overhead word) - so the first burst has no OH word here
                    if (r != 0) begin @(posedge clk_in); lv_idle; lv_word[4] <= 12'h084; end
                    for (c = 0; c < 324; c = c + 1) begin
                        @(posedge clk_in);
                        lv_word[4] <= 12'h087;                                     // DVAL, LVAL, FVAL
                        lv_word[0] <= px12(r, c);     lv_word[1] <= px12(r, 324 + c);
                        lv_word[2] <= px12(r + 1, c); lv_word[3] <= px12(r + 1, 324 + c);
                    end
                end
                @(posedge clk_in); lv_idle;                                        // FVAL low
            end else begin
                for (r = 0; r < nlines; r = r + 1) begin
                    @(posedge clk_in); lval <= 1'b1; dval <= 1'b0;
                    for (c = 0; c < 648; c = c + 1) begin
                        if (c == 324) begin @(posedge clk_in); dval <= 1'b0; d <= 10'h3FF; end   // INVALID slot
                        @(posedge clk_in); dval <= 1'b1; d <= (c + 3*r) & 10'h3FF;
                    end
                    @(posedge clk_in); dval <= 1'b0; lval <= 1'b0; d <= 10'd0;
                end
            end
            repeat (200) @(posedge clk_in);                 // gap before the next frame
        end
        busy = 1'b0;
    end

    //------------------------------------------------------------------------
    // LVDS serialiser: 12 bits per CLK_IN period, LSB first, DDR clock with bit 0 in its high phase
    //------------------------------------------------------------------------
    // one bit lasts a twelfth of the measured CLK_IN period: 3.333 ns at 25 MHz (300 Mbit/s),
    // 2.083 ns at 40 MHz (480 Mbit/s) - the model follows whatever clock the FPGA supplies
    real       t_last = 0.0, t1 = 40.0 / 12.0;
    reg        lvclk = 1'b0;
    reg [4:0]  ser = 5'd0;
    reg [11:0] cur [0:4];
    integer b, k;
    always @(posedge clk_in) begin
        if (t_last != 0.0 && ($realtime - t_last) < 100.0) t1 = ($realtime - t_last) / 12.0;
        t_last = $realtime;
    end
    always @(posedge clk_in) if (enable_lvds && sys_res_n) begin
        for (k = 0; k < 5; k = k + 1) cur[k] = lv_word[k];
        for (b = 0; b < 12; b = b + 1) begin
            lvclk = ~b[0];
            ser   = {cur[4][b], cur[3][b], cur[2][b], cur[1][b], cur[0][b]};
            #(t1);
        end
    end
    // pad drivers with the lane skews of Fig. 24 (control lane taken as 100 ps)
    always @(lvclk)  begin lv_clk_p <= lvclk; lv_clk_n <= ~lvclk; end
    always @(ser[0]) begin lv_d_p[0] <= #0.25 ser[0]; lv_d_n[0] <= #0.25 ~ser[0]; end
    always @(ser[1]) begin lv_d_p[1] <= #0.93 ser[1]; lv_d_n[1] <= #0.93 ~ser[1]; end
    always @(ser[2]) begin lv_d_p[2] <= #0.45 ser[2]; lv_d_n[2] <= #0.45 ~ser[2]; end
    always @(ser[3]) begin lv_d_p[3] <= #1.05 ser[3]; lv_d_n[3] <= #1.05 ~ser[3]; end
    always @(ser[4]) begin lv_ctrl_p <= #0.10 ser[4]; lv_ctrl_n <= #0.10 ~ser[4]; end
endmodule
