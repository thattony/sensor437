`timescale 1ns / 1ps
//============================================================================
// tb_lab8_core.v - self-checking testbench for lab8_core (lab8_motor.v).
// Scaled-down timing so it runs in seconds: PERIOD = 1000 clocks, 1 % = 10
// clocks, 1 "ms" = 100 clocks, I2C state every 4 clocks. An LSM303 model
// (i2c_slave_model, ST auto-increment rule) sits on bus 1 with ZYXDA = 1.
// Prints PASS/FAIL per check and "ALL CHECKS PASSED" at the end.
//============================================================================
module tb_lab8_core;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg  [31:0] wi00 = 0, wi01 = 0, wi02 = 0, wi03 = 0, wi04 = 0, wi05 = 0, wi06 = 0, ti40 = 0;
    wire [31:0] wo20, wo21, wo22, wo23, wo24, wo25, wo26, wo27, wo28, wo29, wo2a, wo2b, wo2c, wo2d, wo2e, to60;
    reg         po_read = 1'b0;
    wire [31:0] po_data;
    wire        s0l, d0l, s1l, d1l, en1, dir1;
    wire [3:0]  dbg;

    wire scl1, sda1;
    pullup (scl1); pullup (sda1);
    assign scl1 = s1l ? 1'b0 : 1'bz;
    assign sda1 = d1l ? 1'b0 : 1'bz;

    i2c_slave_model #(.ADDR(7'h19), .NAME("LSM303_A"), .SUB7_AUTOINC(1)) u_acc (.scl(scl1), .sda(sda1));

    lab8_core #(
        .PERIOD(1000), .CLK_PER_PCT(10), .GUARD_SAME(20), .GUARD_DIR(50),
        .MS_DIV(100), .TS_DIV(63), .POLL_GAP(50), .I2C_TICK(4), .FIFO_AW(6)
    ) dut (
        .clk(clk), .wi00(wi00), .wi01(wi01), .wi02(wi02), .wi03(wi03), .wi04(wi04), .wi05(wi05), .wi06(wi06),
        .ti40(ti40), .button_n(4'hF),
        .wo20(wo20), .wo21(wo21), .wo22(wo22), .wo23(wo23), .wo24(wo24), .wo25(wo25), .wo26(wo26),
        .wo27(wo27), .wo28(wo28), .wo29(wo29), .wo2a(wo2a), .wo2b(wo2b), .wo2c(wo2c), .wo2d(wo2d), .wo2e(wo2e),
        .to60(to60), .po_read(po_read), .po_data(po_data),
        .i2c0_scl_low(s0l), .i2c0_sda_low(d0l), .i2c1_scl_low(s1l), .i2c1_sda_low(d1l),
        .i2c0_scl_in(1'b1), .i2c0_sda_in(1'b1), .i2c1_scl_in(scl1), .i2c1_sda_in(sda1),
        .en1(en1), .dir1(dir1), .dbg(dbg)
    );

    integer errors = 0;
    task check(input cond, input [8*72-1:0] what);
        begin
            if (cond) $display("PASS  %0s", what);
            else begin $display("FAIL  %0s", what); errors = errors + 1; end
        end
    endtask

    task trig(input integer bit_n);
        begin
            @(negedge clk) ti40 = (32'd1 << bit_n);
            @(negedge clk) ti40 = 32'd0;
        end
    endtask

    // ---- monitors: DIR1 may change only while EN1 is low; measure pulses
    reg en1_d = 0, dir1_d = 0;
    integer rises = 0, high_len = 0, last_high = 0, min_high = 1 << 30, max_high = 0;
    integer dir_violations = 0;
    always @(posedge clk) begin
        en1_d  <= en1;
        dir1_d <= dir1;
        if (dir1 !== dir1_d && (en1 || en1_d)) dir_violations = dir_violations + 1;
        if (en1 && !en1_d) begin rises = rises + 1; high_len = 1; end
        else if (en1) high_len = high_len + 1;
        if (!en1 && en1_d) begin
            last_high = high_len;
            if (high_len < min_high) min_high = high_len;
            if (high_len > max_high) max_high = high_len;
        end
    end
    task reset_pulse_stats;
        begin rises = 0; min_high = 1 << 30; max_high = 0; end
    endtask

    // ---- pipe read of one word (data valid the clock after po_read)
    reg [31:0] w [0:3];
    task read_sample;
        integer k;
        begin
            for (k = 0; k < 4; k = k + 1) begin
                @(negedge clk) po_read = 1'b1;
                @(negedge clk) po_read = 1'b0;
                w[k] = po_data;
            end
        end
    endtask

    integer i, t0, good_markers, n_read;
    initial begin
        // LSM303 registers: STATUS = ZYXDA, X = +1 mg, Y = +2 mg, Z = +1000 mg
        u_acc.mem[8'h27] = 8'h0F;
        u_acc.mem[8'h28] = 8'h10; u_acc.mem[8'h29] = 8'h00;
        u_acc.mem[8'h2A] = 8'h20; u_acc.mem[8'h2B] = 8'h00;
        u_acc.mem[8'h2C] = 8'h80; u_acc.mem[8'h2D] = 8'h3E;

        repeat (5) @(negedge clk);
        trig(1);                                           // reset
        repeat (5) @(negedge clk);

        //------------------------------------------------------------ 1 sampler
        wi06 = 32'h0000_0001;                              // sampler on, watchdog off
        wait (wo2b >= 3);
        repeat (4) @(negedge clk);                         // 4th word of the sample lands
        check(wo29 >= 12, "sampler pushed >= 3 samples into the FIFO");
        read_sample;
        check(w[2][31:28] == 4'hA, "sample word 2 carries the 0xA marker");
        check(w[1][15:0] == 16'h0010 && w[1][31:16] == 16'h0020 && w[2][15:0] == 16'h3E80,
              "sample X/Y/Z = 0x0010 / 0x0020 / 0x3E80 (1, 2, 1000 mg)");
        check(w[2][23:16] == 8'h0F, "sample carries STATUS_REG_A = 0x0F");
        t0 = w[0];
        read_sample;
        check(w[0] > t0, "timestamps increase");
        check(wo2e[15:0] == 0, "no NACKs from the accelerometer");

        // ZYXDA = 0 -> no new samples
        u_acc.mem[8'h27] = 8'h00;
        repeat (3000) @(negedge clk);
        i = wo2b;
        repeat (6000) @(negedge clk);
        check(wo2b == i, "STATUS without ZYXDA pushes nothing");
        u_acc.mem[8'h27] = 8'h0F;

        //------------------------------------------------------------ 2 exactly N pulses
        trig(5);                                           // FIFO reset
        reset_pulse_stats;
        wi04 = 3; wi05 = {16'd0, 8'd30, 8'd1};             // N = 3, 30 % duty, DIR = 1
        i = wo2b;
        trig(2);
        wait (wo26[0] == 1'b1);
        wait (wo26[0] == 1'b0);
        repeat (2000) @(negedge clk);
        check(rises == 3, "N = 3 -> exactly 3 EN1 pulses");
        check(min_high == 300 && max_high == 300, "each pulse is 30 % of the period (300 clocks)");
        check(wo27 == 3 && wo28 == 3, "pulses sent = requested = 3");
        check(en1 == 1'b0, "EN1 held low after the last pulse");
        check(dir1 == 1'b1, "DIR1 = requested direction");

        // the sampler kept running during the move
        check(wo2b >= i + 2, "samples kept arriving while the motor pulsed");

        //------------------------------------------------------------ 3 direction change + duty 100 %
        reset_pulse_stats;
        wi04 = 2; wi05 = {16'd0, 8'd100, 8'd0};
        trig(2);
        wait (wo26[0] == 1'b1);
        wait (wo26[0] == 1'b0);
        repeat (100) @(negedge clk);
        check(rises == 1 && max_high == 2000, "100 % duty, N = 2 -> EN1 high for 2 periods");
        check(dir1 == 1'b0, "DIR1 changed to 0");
        check(dir_violations == 0, "DIR1 never changed while EN1 was high");

        //------------------------------------------------------------ 4 stall trip
        reset_pulse_stats;
        wi04 = 100; wi05 = {16'd0, 8'd50, 8'd1};
        trig(2);
        wait (wo27 == 4);
        repeat (200) @(negedge clk);
        trig(3);                                           // STALL TRIP from the PC
        @(negedge clk);
        check(en1 == 1'b0 && wo26[0] == 1'b0, "trip -> EN1 low, not running (safe state)");
        check(wo26[3] == 1'b1 && wo26[7:4] == 4'd1, "fault latched, code 1 (stall)");
        check(wo2d == 4, "fault log: 4 pulses sent at the trip");
        check(wo2c != 0, "fault log: timestamp recorded");
        reset_pulse_stats;
        trig(2);                                           // GO while faulted
        repeat (3000) @(negedge clk);
        check(rises == 0 && wo26[10] == 1'b1, "GO refused while the fault is latched");
        trig(4);                                           // clear fault
        @(negedge clk);
        check(wo26[3] == 1'b0, "clear fault");
        wi04 = 1;
        trig(2);
        wait (wo26[0] == 1'b1);
        wait (wo26[0] == 1'b0);
        repeat (10) @(negedge clk);
        check(rises == 1, "after clear, a move runs again");

        //------------------------------------------------------------ 5 watchdog
        wi06 = {16'd5, 16'h0003};                          // sampler + watchdog, 5 "ms" = 500 clocks
        wi04 = 1000; wi05 = {16'd0, 8'd50, 8'd1};
        trig(2);
        wait (wo26[0] == 1'b1);
        for (i = 0; i < 6; i = i + 1) begin                // kick every 300 clocks: keeps running
            repeat (300) @(negedge clk);
            trig(6);
        end
        check(wo26[0] == 1'b1, "watchdog kicked -> motor keeps running");
        repeat (700) @(negedge clk);                       // stop kicking
        check(wo26[0] == 1'b0 && en1 == 1'b0 && wo26[7:4] == 4'd2, "no kick for 5 ms -> watchdog stop, code 2");
        trig(4);
        wi06 = 32'h0000_0001;

        //------------------------------------------------------------ 6 STOP (no fault)
        wi04 = 1000;
        trig(2);
        wait (wo27 == 2);
        trig(7);
        @(negedge clk);
        check(en1 == 1'b0 && wo26[0] == 1'b0 && wo26[3] == 1'b0, "STOP -> EN1 low, no fault");

        //------------------------------------------------------------ 7 FIFO overflow keeps whole samples
        trig(5);
        wait (wo26[9] == 1'b1);
        check(wo29 <= 64 && (wo29 % 4) == 0, "FIFO full: count <= 64 and whole samples only");
        n_read = wo29 / 4;
        good_markers = 0;
        for (i = 0; i < n_read; i = i + 1) begin
            read_sample;
            if (w[2][31:28] == 4'hA) good_markers = good_markers + 1;
        end
        check(good_markers == n_read, "every sample read back after overflow is aligned");

        if (errors == 0) $display("ALL CHECKS PASSED");
        else             $display("%0d CHECK(S) FAILED", errors);
        $finish;
    end

    initial begin
        #200_000_000;
        $display("FAIL  timeout");
        $finish;
    end
endmodule
