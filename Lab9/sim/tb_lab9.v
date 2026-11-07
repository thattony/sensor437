`timescale 1ns / 1ps
//============================================================================
// tb_lab9.v - Testbench for YOUR CMV300_Ctrl.v, CMV300_Spi.v and
// CMV300_Counters.v, against the given CMV300_Meters.v and the behavioural
// sensor (sim/cmv300_model.v).
//
// Run:  python build.py --sim     (log: sim/sim.log, ends with RESULT: ...)
//
// Scenarios (every check prints scenario, check number, time, expected and
// actual on ONE line - paste that line when asking for help):
//   1  power-up sequence timing (+ a deliberate violation that must be caught)
//   2  SPI read of the datasheet defaults; Number_lines reconstructed as 488
//   3  write 100 to reg 1, read back, reset, read 232
//   4  CLK_OUT meter reads 0 before reg 69 = 9
//   5  CLK_OUT meter ~36,700 with the default PLL band (reg 83 = 155)
//   6  CLK_OUT meter 25,000 +/- 16 with reg 83 = 251
//   7  ONE frame from idle (Number_frames = 1): 488 LVAL edges, 648 DVAL
//      cycles/row, FRAME_REQ->first LVAL 6.55 ms +/- 1%, readout 12.69 ms +/- 1%
//   8  Number_frames = 4: frame-to-frame period 12.88 ms +/- 1%, and
//      measurably SHORTER than scenario 7's 19.24 ms end-to-end
//   9  exposure sweep 1/6/12/20/40 ms, pipelined: period = FOT +
//      max(readout, exposure) - flat, then a knee at Exp_time ~= 976
//  10  the pixel-counter trap: 648 valid pixels per row; counting 649 means
//      the mid-row INVALID slot was included
//  11  Number_frames = 0 produces no frame
//  12  FRAME_REQ held as a LEVEL free-runs the sensor; your pulse generator
//      must turn a held frame_req_in into ONE pulse
//  13  OR/AND masks of a ramp frame = 0x3FF / 0x000
//============================================================================
module tb_lab9;

    // ---- clocks --------------------------------------------------------------
    reg clk = 1'b0;    always #4.96 clk   = ~clk;    // 100.8 MHz okClk
    reg clk25 = 1'b0;  always #20   clk25 = ~clk25;  // 25 MHz "MMCM" output
    reg rst = 1'b0;
    reg mmcm_locked = 1'b1;

    // ---- student modules -------------------------------------------------------
    reg        cam_enable = 1'b0, cam_reset = 1'b0, frame_req_in = 1'b0, t_exp_in = 1'b0;
    reg        clear = 1'b0;
    wire       clk_en, sys_res_n, frame_req_ctrl, t_exp1, sensor_ready;
    wire [3:0] ctrl_state;

    CMV300_Ctrl u_ctrl (
        .clk(clk), .cam_enable(cam_enable), .cam_reset(cam_reset), .clk_locked(mmcm_locked),
        .frame_req_in(frame_req_in), .t_exp_in(t_exp_in),
        .clk_en(clk_en), .sys_res_n(sys_res_n), .frame_req(frame_req_ctrl), .t_exp1(t_exp1),
        .sensor_ready(sensor_ready), .state(ctrl_state)
    );

    reg        spi_go = 1'b0, spi_wr = 1'b0;
    reg [6:0]  spi_addr = 7'd0;
    reg [7:0]  spi_wdat = 8'd0;
    reg [1:0]  spi_burst = 2'd0;
    wire       spi_busy, spi_done;
    wire [31:0] spi_rdat;
    wire       spi_clk_w, spi_en_w, spi_in_w, spi_out_w;

    CMV300_Spi #(.HALF(5)) u_spi (
        .clk(clk), .rst(rst), .go(spi_go), .wr(spi_wr), .addr(spi_addr), .wr_data(spi_wdat),
        .burst(spi_burst), .busy(spi_busy), .done(spi_done), .rd_data(spi_rdat),
        .SPI_CLK(spi_clk_w), .SPI_EN(spi_en_w), .SPI_IN(spi_in_w), .SPI_OUT(spi_out_w)
    );

    // ---- the sensor (force paths bypass the student modules on purpose) --------
    reg  force_spi_en = 1'b0, force_req_level = 1'b0;
    wire clk_in       = clk25 & clk_en;
    wire model_spi_en = spi_en_w | force_spi_en;
    wire model_req    = frame_req_ctrl | force_req_level;
    wire clk_out, lval, dval;
    wire [9:0]  d;
    wire [15:0] violations;

    cmv300_model u_sensor (
        .clk_in(clk_in), .sys_res_n(sys_res_n), .enable_lvds(1'b0), .frame_req(model_req),
        .spi_clk(spi_clk_w), .spi_en(model_spi_en), .spi_in(spi_in_w), .spi_out(spi_out_w),
        .clk_out(clk_out), .d(d), .lval(lval), .dval(dval), .violations(violations)
    );

    // ---- the given meters + YOUR counters ---------------------------------------
    wire [15:0] frame_count, last_row_pixels, last_frame_rows;
    wire [31:0] lval_edges, clk_out_meas;
    wire [23:0] t_req_us, t_read_us, t_per_us;
    wire [9:0]  or_mask, and_mask;
    wire        frame_done_pulse;
    CMV300_Meters #(.MEAS_WINDOW(100800), .GAP_US(50)) u_meters (
        .clk_cmv(clk_out), .clk(clk), .rst(rst), .clear(clear),
        .frame_req(model_req), .lval(lval),
        .frame_count(frame_count),
        .t_req_to_lval_us(t_req_us), .t_readout_us(t_read_us), .t_period_us(t_per_us),
        .frame_done_pulse(frame_done_pulse), .clk_out_meas(clk_out_meas)
    );
    CMV300_Counters #(.GAP(64)) u_counters (
        .clk_cmv(clk_out), .d(d), .lval(lval), .dval(dval),
        .clk(clk), .rst(rst), .clear(clear),
        .lval_edges(lval_edges), .last_row_pixels(last_row_pixels), .last_frame_rows(last_frame_rows),
        .or_mask(or_mask), .and_mask(and_mask)
    );

    integer n_frames_done = 0;
    always @(posedge clk) if (frame_done_pulse) n_frames_done = n_frames_done + 1;

    integer n_req_pulses = 0;
    realtime t_req_rise = 0, req_width_ns = 0;
    always @(posedge frame_req_ctrl) begin n_req_pulses = n_req_pulses + 1; t_req_rise = $realtime; end
    always @(negedge frame_req_ctrl) req_width_ns = $realtime - t_req_rise;

    // ---- check machinery ---------------------------------------------------------
    integer fails = 0, checks = 0;
    task chk(input integer scen, input integer num, input reg cond,
             input [63:0] expv, input [63:0] actv, input [8*56-1:0] name);
        begin
            checks = checks + 1;
            if (cond) $display("SCEN %0d CHECK %0d @%t PASS %0s (expected %0d, actual %0d)", scen, num, $realtime, name, expv, actv);
            else begin
                fails = fails + 1;
                $display("SCEN %0d CHECK %0d @%t FAIL %0s: expected %0d actual %0d", scen, num, $realtime, name, expv, actv);
            end
        end
    endtask
    // a value within +/- pct percent
    function in_pct(input [63:0] act, input [63:0] expv, input integer pct);
        in_pct = (act >= expv - expv*pct/100) && (act <= expv + expv*pct/100);
    endfunction

    reg spi_dead = 1'b0;
    task spi_xfer(input wr_i, input [6:0] a, input [7:0] v, input [1:0] b);
        integer t;
        begin
            spi_wr = wr_i; spi_addr = a; spi_wdat = v; spi_burst = b;
            @(posedge clk); spi_go <= 1'b1; @(posedge clk); spi_go <= 1'b0;
            t = 0;
            while (!spi_done && t < 200000) begin @(posedge clk); t = t + 1; end
            if (t >= 200000) begin
                spi_dead = 1'b1;
                $display("SCEN - @%t: SPI TIMEOUT (busy=%b) - is CMV300_Spi.v still the stub?", $realtime, spi_busy);
            end
            @(posedge clk);
        end
    endtask
    task wr_reg(input [6:0] a, input [7:0] v); spi_xfer(1'b1, a, v, 2'd0); endtask
    task rd_reg(input [6:0] a); spi_xfer(1'b0, a, 8'd0, 2'd0); endtask
    function [7:0] last_rd; input dummy; last_rd = spi_rdat[7:0]; endfunction
    task pulse_req; begin @(posedge clk); frame_req_in <= 1'b1; @(posedge clk); frame_req_in <= 1'b0; end endtask

    task wait_frames(input integer n, input integer timeout_ms);
        integer t0;
        begin
            t0 = n_frames_done;
            fork : wf
                wait (n_frames_done >= t0 + n);
                begin repeat (timeout_ms) #1_000_000; $display("SCEN - @%t: TIMEOUT waiting for %0d frame(s)", $realtime, n); end
            join_any
            disable wf;
            #200_000;                              // let the okClk-side counters settle
        end
    endtask

    // expected numbers at 25 MHz, band 251 (in us)
    localparam integer EXP_DEF_US  = 6344;         // 488 x 325 x 40 ns
    localparam integer FOT_US      = 195;          // 4 x 1220 x 40 ns
    localparam integer READOUT_US  = 12707;        // 488 x 651 x 40 ns (row = 650 high + 1 low)
    function [63:0] period_pipe_us(input [63:0] exp_us);
        period_pipe_us = FOT_US + ((exp_us > 12688) ? exp_us : 12688) + (READOUT_US - 12688);
    endfunction

    integer v0, exp_ticks, s7_total_us, p1, p12;
    real e_ms;
    integer sweep_i;
    real exp_list [0:4];
    integer p_meas [0:4];

    initial begin
        if ($test$plusargs("vcd")) begin $dumpfile("tb_lab9.vcd"); $dumpvars(0, tb_lab9); end
        $timeformat(-9, 1, " ns", 14);
        exp_list[0] = 1.0; exp_list[1] = 6.0; exp_list[2] = 12.0; exp_list[3] = 20.0; exp_list[4] = 40.0;
        #200;

        // ---- 1: power-up ------------------------------------------------------
        $display("---- scenario 1: power-up sequence ----");
        cam_enable = 1'b1;
        fork : pu
            wait (sensor_ready);
            begin #2_000_000; $display("SCEN 1 @%t: TIMEOUT waiting for sensor_ready - is CMV300_Ctrl.v still the stub?", $realtime); end
        join_any
        disable pu;
        chk(1, 1, sensor_ready === 1'b1, 1, sensor_ready, "sensor_ready after cam_enable");
        chk(1, 2, sys_res_n === 1'b1, 1, sys_res_n, "SYS_RES_N released");
        chk(1, 3, clk_en === 1'b1, 1, clk_en, "CLK_IN running");
        chk(1, 4, violations == 0, 0, violations, "no start-up violation");
        chk(1, 5, ctrl_state == 4'd3, 3, ctrl_state, "state = READY (3)");
        v0 = violations;
        cam_enable = 1'b0; #100_000;
        force_spi_en = 1'b1; #500; force_spi_en = 1'b0;
        chk(1, 6, violations == v0 + 1, v0 + 1, violations, "model catches SPI during reset");
        cam_enable = 1'b1;
        fork : pu2 wait (sensor_ready); #2_000_000; join_any
        disable pu2;
        #10_000;

        // fast-fail: with a stub sequencer or SPI master nothing below can work -
        // report the remaining scenarios as named failures WITHOUT waiting out the
        // frame timeouts (a stub run must finish in seconds, not hours)
        if (!sensor_ready) begin
            $display("TB: sequencer never became ready - fast-failing the remaining scenarios");
            chk(2, 1, 1'b0, 232, 0, "SKIPPED (no ready sequencer): reg 1 default");
            chk(7, 1, 1'b0, 488, 0, "SKIPPED (no ready sequencer): 488 LVAL edges");
            chk(8, 1, 1'b0, 12883, 0, "SKIPPED (no ready sequencer): pipelined period");
            chk(10, 1, 1'b0, 648, 0, "SKIPPED (no ready sequencer): pixels per row");
            chk(13, 1, 1'b0, 1023, 0, "SKIPPED (no ready sequencer): OR mask");
            finish_up;
        end

        // ---- 2: defaults ------------------------------------------------------
        $display("---- scenario 2: SPI read of the datasheet defaults ----");
        rd_reg(7'd1);  chk(2, 1, last_rd(0) == 232, 232, last_rd(0), "reg 1 default");
        rd_reg(7'd2);  chk(2, 2, last_rd(0) == 1,   1,   last_rd(0), "reg 2 default");
        rd_reg(7'd42); chk(2, 3, last_rd(0) == 232, 232, last_rd(0), "reg 42 default");
        rd_reg(7'd61); chk(2, 4, last_rd(0) == 85,  85,  last_rd(0), "reg 61 default");
        rd_reg(7'd63); chk(2, 5, last_rd(0) == 12,  12,  last_rd(0), "reg 63 default");
        rd_reg(7'd77); chk(2, 6, last_rd(0) == 128, 128, last_rd(0), "reg 77 default");
        rd_reg(7'd81); chk(2, 7, last_rd(0) == 126, 126, last_rd(0), "reg 81 default");
        rd_reg(7'd83); chk(2, 8, last_rd(0) == 155, 155, last_rd(0), "reg 83 default");
        rd_reg(7'd85); chk(2, 9, last_rd(0) == 3,   3,   last_rd(0), "reg 85 default");
        rd_reg(7'd99); chk(2, 10, last_rd(0) == 255, 255, last_rd(0), "reg 99 default");
        spi_xfer(1'b0, 7'd1, 8'd0, 2'd1);
        chk(2, 11, (spi_rdat[7:0] + 256*spi_rdat[15:8]) == 488, 488, spi_rdat[7:0] + 256*spi_rdat[15:8], "Number_lines from regs 1/2");
        spi_xfer(1'b0, 7'd78, 8'd0, 2'd1);
        chk(2, 12, spi_rdat[15:0] == 16'h1464, 16'h1464, spi_rdat[15:0], "temperature burst regs 78/79");

        if (spi_dead) begin
            $display("TB: the SPI master never completed a transfer - fast-failing the remaining scenarios");
            chk(5, 1, 1'b0, 36700, 0, "SKIPPED (no SPI): CLK_OUT off-band");
            chk(7, 1, 1'b0, 488, 0, "SKIPPED (no SPI): 488 LVAL edges");
            chk(8, 1, 1'b0, 12883, 0, "SKIPPED (no SPI): pipelined period");
            chk(10, 1, 1'b0, 648, 0, "SKIPPED (no SPI): pixels per row");
            chk(13, 1, 1'b0, 1023, 0, "SKIPPED (no SPI): OR mask");
            finish_up;
        end

        // ---- 3: write / read-back / reset --------------------------------------
        $display("---- scenario 3: write, read back, reset ----");
        wr_reg(7'd1, 8'd100);
        rd_reg(7'd1); chk(3, 1, last_rd(0) == 100, 100, last_rd(0), "read-back of reg 1 = 100");
        @(posedge clk); cam_reset <= 1'b1; @(posedge clk); cam_reset <= 1'b0;
        #10_000; wait (sensor_ready); #10_000;
        rd_reg(7'd1); chk(3, 2, last_rd(0) == 232, 232, last_rd(0), "reg 1 back to 232 after reset");

        // ---- 4/5/6: CLK_OUT meter -----------------------------------------------
        $display("---- scenario 4: CLK_OUT gated by reg 69 ----");
        #2_200_000;
        chk(4, 1, clk_out_meas == 0, 0, clk_out_meas, "CLK_OUT meter 0 before reg 69 = 9");
        $display("---- scenario 5: reg 69 = 9, default band (reg 83 = 155) ----");
        wr_reg(7'd69, 8'd9);
        #2_500_000;
        chk(5, 1, clk_out_meas > 36200 && clk_out_meas < 37200, 36700, clk_out_meas, "CLK_OUT/ms off-band ~36,700");
        $display("---- scenario 6: reg 83 = 251 ----");
        wr_reg(7'd83, 8'd251);
        #2_500_000;
        chk(6, 1, clk_out_meas >= 24984 && clk_out_meas <= 25016, 25000, clk_out_meas, "CLK_OUT/ms = 25,000 +/- 16");

        // ---- 7: ONE frame from idle: the sequential regime ------------------------
        $display("---- scenario 7: single frame (Number_frames = 1) ----");
        @(posedge clk); clear <= 1'b1; @(posedge clk); clear <= 1'b0; #1000;
        pulse_req;
        wait_frames(1, 40);
        chk(7, 1, lval_edges == 488, 488, lval_edges, "488 LVAL edges after one FRAME_REQ");
        chk(7, 2, last_row_pixels == 648, 648, last_row_pixels, "648 DVAL cycles per row");
        chk(7, 3, last_frame_rows == 488, 488, last_frame_rows, "488 rows in the frame");
        chk(7, 4, in_pct(t_req_us, EXP_DEF_US + FOT_US, 1), EXP_DEF_US + FOT_US, t_req_us,
            "FRAME_REQ->first LVAL = exposure + FOT (6.55 ms +/- 1%)");
        chk(7, 5, in_pct(t_read_us, READOUT_US, 1), READOUT_US, t_read_us, "readout 12.69 ms +/- 1%");
        s7_total_us = t_req_us + t_read_us;
        $display("SCEN 7: end-to-end = %0d us (~19.24 ms: the SEQUENTIAL regime)", s7_total_us);

        // ---- 8: pipelined regime ---------------------------------------------------
        $display("---- scenario 8: Number_frames = 4, pipelined ----");
        wr_reg(7'd55, 8'd4);
        pulse_req;
        wait_frames(4, 90);
        chk(8, 1, in_pct(t_per_us, period_pipe_us(EXP_DEF_US), 1), period_pipe_us(EXP_DEF_US), t_per_us,
            "frame-to-frame period 12.88 ms +/- 1%");
        chk(8, 2, t_per_us + 24'd1000 < s7_total_us, s7_total_us, t_per_us,
            "pipelined period measurably SHORTER than the 19.24 ms sequential end-to-end");

        // ---- 9: exposure sweep, pipelined ------------------------------------------
        $display("---- scenario 9: exposure sweep (period = FOT + max(readout, exposure)) ----");
        wr_reg(7'd55, 8'd2);
        for (sweep_i = 0; sweep_i < 5; sweep_i = sweep_i + 1) begin
            e_ms = exp_list[sweep_i];
            exp_ticks = e_ms * 1e-3 * 25.0e6 / 325.0;
            wr_reg(7'd42, exp_ticks[7:0]); wr_reg(7'd43, exp_ticks[15:8]); wr_reg(7'd44, exp_ticks[23:16]);
            pulse_req;
            wait_frames(2, 150);
            p_meas[sweep_i] = t_per_us;
            chk(9, sweep_i + 1, in_pct(t_per_us, period_pipe_us(e_ms * 1000.0), 1),
                period_pipe_us(e_ms * 1000.0), t_per_us, "period at this exposure");
        end
        chk(9, 6, in_pct(p_meas[2], p_meas[0], 1), p_meas[0], p_meas[2], "flat below the knee (12 ms ~= 1 ms)");
        chk(9, 7, p_meas[3] > p_meas[2] + 5000, p_meas[2] + 7300, p_meas[3], "the knee: 20 ms exposure lengthens the period (Exp_time ~= 976)");
        wr_reg(7'd42, 8'd232); wr_reg(7'd43, 8'd1); wr_reg(7'd44, 8'd0);

        // ---- 10: the pixel-counter trap ---------------------------------------------
        $display("---- scenario 10: the mid-row INVALID slot ----");
        checks = checks + 1;
        if (last_row_pixels == 648)
            $display("SCEN 10 CHECK 1 @%t PASS 648 valid pixels per row (expected 648, actual %0d)", $realtime, last_row_pixels);
        else begin
            fails = fails + 1;
            $display("SCEN 10 CHECK 1 @%t FAIL pixels per row: expected 648 actual %0d%s", $realtime, last_row_pixels,
                     (last_row_pixels == 649) ?
                     " - you counted the mid-row INVALID slot: DVAL drops for ONE CLK_OUT cycle after pixel 324 (Figure 26); gate the count with DVAL, not with LVAL alone" :
                     (last_row_pixels == 324) ? " - the count restarted at the mid-row INVALID slot (DVAL low for one cycle after pixel 324); accumulate across it" : "");
        end

        // ---- 11: Number_frames = 0 ---------------------------------------------------
        $display("---- scenario 11: Number_frames = 0 ----");
        wr_reg(7'd55, 8'd0);
        v0 = n_frames_done;
        pulse_req;
        #30_000_000;
        chk(11, 1, n_frames_done == v0, v0, n_frames_done, "no frame with Number_frames = 0");
        wr_reg(7'd55, 8'd1);

        // ---- 12: a LEVEL free-runs; the pulse generator prevents it -------------------
        $display("---- scenario 12: FRAME_REQ level vs pulse generator ----");
        v0 = n_frames_done;
        force_req_level = 1'b1;
        #45_000_000;
        force_req_level = 1'b0;
        chk(12, 1, n_frames_done >= v0 + 2, v0 + 2, n_frames_done, "level at the pin -> continuous frames");
        wait (u_sensor.busy == 0); #1_000_000;
        v0 = n_frames_done; sweep_i = n_req_pulses;
        frame_req_in = 1'b1;
        #25_000_000;
        frame_req_in = 1'b0;
        chk(12, 2, n_req_pulses == sweep_i + 1, sweep_i + 1, n_req_pulses, "held frame_req_in -> exactly ONE pulse");
        chk(12, 3, req_width_ns >= 160.0, 160, req_width_ns, "pulse >= 4 CLK_IN periods (160 ns)");
        chk(12, 4, n_frames_done == v0 + 1, v0 + 1, n_frames_done, "exactly one frame from the held level");
        wait (u_sensor.busy == 0); #1_000_000;

        // ---- 13: OR/AND masks ----------------------------------------------------------
        $display("---- scenario 13: OR/AND masks of the ramp ----");
        chk(13, 1, or_mask == 10'h3FF, 10'h3FF, or_mask, "OR mask of the ramp = 0x3FF");
        chk(13, 2, and_mask == 10'h000, 0, and_mask, "AND mask of the ramp = 0x000");

        finish_up;
    end

    task finish_up;
        begin
            #100;
            $display("TB: %0d checks, %0d failure(s)", checks, fails);
            if (fails == 0) $display("RESULT: ALL PASS");
            else            $display("RESULT: %0d FAILED", fails);
            $finish;
        end
    endtask
endmodule
