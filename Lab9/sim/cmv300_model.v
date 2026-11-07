`timescale 1ns / 1ps
//============================================================================
// cmv300_model.v - behavioural CMV300 for the Lab 9 simulation
//   (CMV300.pdf: SPI section 3.9, register defaults section 6, parallel video
//    section 4.2.1 Figures 26-27, start-up section 3.7 Figures 4-5).
//
//   * 128 registers with the datasheet power-on defaults; a falling edge on
//     SYS_RES_N restores them. Every SPI access is printed.
//   * START-UP ENFORCEMENT: the model counts a violation (and prints
//     "MODEL VIOLATION") when SYS_RES_N rises less than 1 us after CLK_IN
//     started, or when SPI_EN rises while the sensor is in reset or less
//     than 1 us after the reset release. It keeps working - silently working
//     hardware with a wrong sequence is exactly the bug this catches.
//   * CLK_OUT: none until reg 69 = 9 (v2.11). PLL band (reg 83): 251 ->
//     CLK_OUT = CLK_IN; anything else at 25 MHz -> about 36.7 MHz (the
//     out-of-band lock this board really produces).
//   * Video, on CLK_OUT: FRAME_REQ rising edge (edge-latched; a level held
//     high re-triggers, i.e. continuous frames) produces Number_frames
//     (regs 55/56; 0 = none) frames of Number_lines (regs 1/2) rows. A row =
//     324 pixels, one INVALID slot (DVAL low), 324 pixels; LVAL low for one
//     CLK_OUT between rows. TWO TIMING REGIMES, exactly like the silicon:
//       - Number_frames = 1 from idle: exposure, then FOT, then readout,
//         STRICTLY IN SEQUENCE: at the defaults 6.55 ms of silence after
//         FRAME_REQ, then 12.69 ms of readout - 19.24 ms end to end.
//       - Number_frames > 1 (and a held FRAME_REQ level): the next exposure
//         OVERLAPS the current readout; frame period = FOT + max(readout,
//         exposure) = 12.88 ms at the default exposure.
//     Exposure = regs 42/43/44 x 325 CLK_IN periods (section 5.1); FOT =
//     reg 58 x 1220 CLK_OUT periods (4,880 at the default 4 - the datasheet's
//     ">= 4,875 clocks"). Data changes on the rising edge (the receiver
//     samples on the falling edge). Pixel = (col + 3 x row) mod 1024 - a
//     ramp, so the OR/AND masks of a full frame are 0x3FF / 0x000.
//   * Temperature registers 78/79 hold 0x1464 = 5220 (burst-read test).
//============================================================================
module cmv300_model (
    input  wire       clk_in,
    input  wire       sys_res_n,
    input  wire       enable_lvds,           // must stay 0 in this lab
    input  wire       frame_req,
    input  wire       spi_clk,
    input  wire       spi_en,
    input  wire       spi_in,
    output reg        spi_out,
    output reg        clk_out,
    output reg  [9:0] d,
    output reg        lval,
    output reg        dval,
    output reg [15:0] violations             // start-up / SPI-timing violations counted
);
    reg [7:0] regs [0:127];
    integer i;

    task set_defaults;
        begin
            for (i = 0; i < 128; i = i + 1) regs[i] = 8'd0;
            regs[1]  = 232; regs[2]  = 1;        // Number_lines = 488
            regs[39] = 1;
            regs[42] = 232; regs[43] = 1;        // Exp_time = 488 (~6.34 ms at 25 MHz)
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
        spi_out = 1'b0; clk_out = 1'b0; d = 10'd0; lval = 1'b0; dval = 1'b0; violations = 16'd0;
    end
    always @(negedge sys_res_n) begin
        set_defaults;
        $display("%0t  CMV300 model: SYS_RES_N fell - registers back to defaults", $time);
    end

    //------------------------------------------------------------------------
    // CLK_IN observation: period measurement + "when did the clock start"
    //------------------------------------------------------------------------
    real t_edge = -1.0, t_clk_start = -1.0, t_period = 40.0, t_res_rise = -1.0;
    always @(posedge clk_in) begin
        if (t_edge < 0.0 || ($realtime - t_edge) > 10000.0) begin
            t_clk_start = $realtime;               // the clock (re)started after >10 us of silence
            $display("%0t  CMV300 model: CLK_IN started", $time);
        end else t_period = $realtime - t_edge;
        t_edge = $realtime;
    end
    wire clk_running = (t_edge >= 0.0) && (($realtime - t_edge) < 10000.0);

    //------------------------------------------------------------------------
    // Start-up enforcement (section 3.7: >= 1 us clock before reset release,
    // >= 1 us after it before SPI)
    //------------------------------------------------------------------------
    always @(posedge sys_res_n) begin
        t_res_rise = $realtime;
        if (t_clk_start < 0.0 || ($realtime - t_clk_start) < 1000.0) begin
            violations = violations + 1;
            $display("%0t  CMV300 model: MODEL VIOLATION - SYS_RES_N released %0.0f ns after CLK_IN started (>= 1000 required)",
                     $time, (t_clk_start < 0.0) ? 0.0 : ($realtime - t_clk_start));
        end
    end
    always @(posedge spi_en) begin
        if (!sys_res_n) begin
            violations = violations + 1;
            $display("%0t  CMV300 model: MODEL VIOLATION - SPI access while the sensor is in reset", $time);
        end else if (t_res_rise < 0.0 || ($realtime - t_res_rise) < 1000.0) begin
            violations = violations + 1;
            $display("%0t  CMV300 model: MODEL VIOLATION - SPI access %0.0f ns after the reset release (>= 1000 required)",
                     $time, ($realtime - t_res_rise));
        end
    end

    //------------------------------------------------------------------------
    // SPI slave (16-bit frames, back to back while SPI_EN is high)
    //------------------------------------------------------------------------
    reg [4:0]  bitc = 5'd0;
    reg [15:0] sh   = 16'd0;
    reg        rw   = 1'b0;
    reg [6:0]  addr = 7'd0;
    reg [7:0]  rdv  = 8'd0;

    always @(negedge spi_en) begin bitc <= 5'd0; spi_out <= 1'b0; end

    always @(posedge spi_clk) if (spi_en) begin
        sh <= {sh[14:0], spi_in};
        if (bitc == 5'd7) begin
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

    always @(negedge spi_clk) if (spi_en) begin
        if (!rw && bitc >= 5'd8 && bitc <= 5'd15) spi_out <= rdv[5'd15 - bitc];
        else spi_out <= 1'b0;
    end

    //------------------------------------------------------------------------
    // CLK_OUT generator: none until reg 69 = 9; PLL band from reg 83
    //------------------------------------------------------------------------
    wire clk_out_on = (regs[69] == 8'd9) && sys_res_n && clk_running;
    real t_half;
    always begin
        if (!clk_out_on) begin clk_out = 1'b0; @(posedge clk_in); end
        else begin
            // band 251 locks CLK_OUT = CLK_IN; any other band at 25 MHz locks off-band at ~36.7 MHz
            t_half = (regs[83] == 8'd251) ? t_period / 2.0 : (t_period / 2.0) * (25.0 / 36.7);
            clk_out = ~clk_out;
            #(t_half);
        end
    end

    //------------------------------------------------------------------------
    // Video sequencer (edge-latched FRAME_REQ; a held level re-triggers)
    //------------------------------------------------------------------------
    reg busy = 1'b0, req_pending = 1'b0;
    integer f, r, c, nframes, nlines;
    real t_exp_ns, t_readout_ns, t_fot_ns, t_spare_ns, t_f0;

    always @(posedge frame_req) if (sys_res_n) req_pending = 1'b1;
    always @(negedge sys_res_n) req_pending = 1'b0;

    always begin
        wait ((req_pending || (frame_req && sys_res_n)) && !busy && clk_out_on);
        req_pending = 1'b0;
        busy = 1'b1;
        nframes = {regs[56], regs[55]};
        nlines  = {regs[2], regs[1]};
        $display("%0t  CMV300 model: FRAME_REQ -> %0d frame(s) of %0d rows", $time, nframes, nlines);
        for (f = 0; f < nframes; f = f + 1) begin
            t_exp_ns     = ({regs[44], regs[43], regs[42]} * 325.0) * t_period;
            t_readout_ns = (nlines * 650.0) * (t_period * ((regs[83] == 8'd251) ? 1.0 : 25.0/36.7));
            t_fot_ns     = (regs[58] * 1220.0) * (t_period * ((regs[83] == 8'd251) ? 1.0 : 25.0/36.7));
            if (f == 0)
                // first frame from idle: exposure, then FOT, then readout - strictly in sequence
                t_spare_ns = t_exp_ns + t_fot_ns;
            else begin
                // pipelined: the next exposure overlapped the readout we just produced;
                // start-to-start spacing = FOT + max(readout, exposure)
                t_spare_ns = t_fot_ns + (((t_exp_ns > t_readout_ns) ? t_exp_ns : t_readout_ns) - t_readout_ns);
            end
            #(t_spare_ns);
            for (r = 0; r < nlines; r = r + 1) begin
                @(posedge clk_out); lval <= 1'b1; dval <= 1'b0;
                for (c = 0; c < 648; c = c + 1) begin
                    if (c == 324) begin @(posedge clk_out); dval <= 1'b0; d <= 10'h3FF; end   // INVALID slot
                    @(posedge clk_out); dval <= 1'b1; d <= (c + 3*r) & 10'h3FF;               // ramp
                end
                @(posedge clk_out); dval <= 1'b0; lval <= 1'b0; d <= 10'd0;                   // one low clock between rows
            end
        end
        busy = 1'b0;
        @(posedge clk_in);       // guarantees time advances per sequence: a held FRAME_REQ
                                 // with Number_frames = 0 must not livelock the simulator
    end
endmodule
