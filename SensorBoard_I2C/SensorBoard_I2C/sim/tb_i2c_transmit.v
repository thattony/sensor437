`timescale 1ns / 1ps
//============================================================================
// tb_i2c_transmit.v - Testbench for hdl/I2C_Transmit.v (the file you edit).
//
// Run:  ./build.sh --sim        (xsim, log in sim/sim.log, waveform sim/tb.vcd)
// or open the Vivado project in build/vivado and "Run Simulation".
//
// The state machine is simulated on its own (the top level only adds pads,
// the bus selector and the PC endpoints): 100.8 MHz clock, reset, a byte,
// a start pulse, wait for done (with a timeout), print and check the results.
// One I2C bus with a pull-up carries two behavioural slaves from
// sim/i2c_slave_model.v: an "ADT7420" at 0x48 (ID 0x0B = 0xCB, temperature
// 0x00/0x01 = 0x0C 0xA0) and an "LSM303 accel" at 0x19. The slaves print
// START / STOP / address / byte events, so a protocol error is visible in
// the log without a scope.
//
// Frames sent:
//   1. 0x90  ADT7420 write address       -> ACK  (ACK_bit = 0)
//   2. 0x92  address 0x49, nobody there  -> NACK (ACK_bit = 1)
//   3. 0x32  LSM303 accel write address  -> ACK
//   4. 0x91  ADT7420 READ address        -> ACK, but look at the log: the
//      slave starts sending data bit 7 (= 0) right after the ACK and holds
//      SDA low, so the master's STOP never happens ("no STOP" line). That is
//      why a read transaction must go on to clock the byte(s) out and NACK
//      the last one (ADT7420.pdf Figure 17, p.19) - your job.
//   3b. (M2 steps 1-3) param3[0] = 1, param2 = 0x0B: S 0x90 A 0x0B A Sr 0x91
//       A data(0xCB) NACK P - a bus monitor checks bytes, ACK/NACKs and
//       START/STOP positions straight off the wires; rx_data must be 0xCB.
//   3c. same read of register 0x00 (0x0C, MSB = 0 - must not hang); leaves the
//       slave pointer at 0x00 so frame 4 is unchanged.
//   3d. (Gate 4) same read mode, absent address 0x49 (0x92): error = 1,
//       State 0, busy 0, lines released.
// Preload other registers / add slaves at other addresses to match the
// sensor you are implementing, and extend the checks as the FSM grows.
//============================================================================
module tb_i2c_transmit;

    // ---- clock / reset ---------------------------------------------------
    reg clk = 1'b0;
    always #4.96 clk = ~clk;              // 100.8 MHz
    reg rst = 1'b1;

    // ---- host side --------------------------------------------------------
    reg         start = 1'b0;
    reg  [7:0]  tx_byte = 8'h00;
    reg  [31:0] param2 = 32'd0, param3 = 32'd0;
    wire        ACK_bit, busy, done, error_bit;
    wire [31:0] rx_data;
    wire [7:0]  State;

    // ---- one I2C bus with pull-ups and two slaves --------------------------
    wire scl, sda;
    pullup (scl); pullup (sda);
    wire scl_low, sda_low;
    assign scl = scl_low ? 1'b0 : 1'bz;
    assign sda = sda_low ? 1'b0 : 1'bz;

    i2c_slave_model #(.ADDR(7'h48), .NAME("ADT7420"))  u_adt7420 (.scl(scl), .sda(sda));
    i2c_slave_model #(.ADDR(7'h19), .NAME("LSM303_A")) u_lsm303a (.scl(scl), .sda(sda));

    I2C_Transmit #(.TICK_DIVIDE(252)) dut (
        .clk(clk), .rst(rst), .start(start),
        .tx_byte(tx_byte), .param2(param2), .param3(param3),
        .scl_low(scl_low), .sda_low(sda_low), .scl_in(scl), .sda_in(sda),
        .ACK_bit(ACK_bit), .rx_data(rx_data),
        .busy(busy), .done(done), .error_bit(error_bit), .State(State)
    );

    // ---- bookkeeping: cycle counter (like result2), STOP counter -----------
    integer cycles = 0;
    always @(posedge clk) if (busy) cycles = cycles + 1;
    integer stops = 0;
    always @(posedge sda) if (scl === 1'b1) stops = stops + 1;
    integer starts = 0;
    always @(negedge sda) if (scl === 1'b1) starts = starts + 1;

    // ---- bus monitor: decodes every 9-bit slot straight off the wires -------
    // (independent of the slave model) -> bus_byte[k], bus_ack[k] (0 = ACK)
    reg [8:0] mon_sh = 9'd0;
    integer   mon_n  = 0;
    integer   bus_cnt = 0;
    reg [7:0] bus_byte [0:7];
    reg       bus_ack  [0:7];
    always @(negedge sda) if (scl === 1'b1) mon_n = 0;           // START / repeated START

    // START / STOP positions inside a transaction, counted in bytes already on the bus
    integer start_n = 0, stop_n = 0, first_stop_at = -1;
    integer start_at [0:3];
    always @(negedge sda) if (scl === 1'b1) begin
        $display("%0t  BUS: %0s (after %0d bytes)", $time, start_n == 0 ? "START" : "repeated START", bus_cnt);
        if (start_n < 4) start_at[start_n] = bus_cnt;
        start_n = start_n + 1;
    end
    always @(posedge sda) if (scl === 1'b1) begin
        $display("%0t  BUS: STOP (after %0d bytes)", $time, bus_cnt);
        if (stop_n == 0) first_stop_at = bus_cnt;
        stop_n = stop_n + 1;
    end
    always @(posedge scl) begin
        mon_sh = {mon_sh[7:0], (sda === 1'b0) ? 1'b0 : 1'b1};
        mon_n  = mon_n + 1;
        if (mon_n == 9) begin
            $display("%0t  BUS: byte 0x%02X, 9th clock %0s", $time, mon_sh[8:1], mon_sh[0] ? "NACK" : "ACK");
            if (bus_cnt < 8) begin bus_byte[bus_cnt] = mon_sh[8:1]; bus_ack[bus_cnt] = mon_sh[0]; end
            bus_cnt = bus_cnt + 1;
            mon_n   = 0;
        end
    end

    integer fails = 0;
    task check(input [8*64-1:0] name, input cond);
        begin
            if (cond) $display("%0t  TB: PASS %0s", $time, name);
            else begin $display("%0t  TB: FAIL %0s", $time, name); fails = fails + 1; end
        end
    endtask

    // ---- one frame: set the byte, pulse start, wait for done ---------------
    task run_frame(input [7:0] b, input integer timeout_us);
        integer t;
        begin
            tx_byte = b; cycles = 0; bus_cnt = 0; start_n = 0; stop_n = 0; first_stop_at = -1;
            @(posedge clk); start <= 1'b1; @(posedge clk); start <= 1'b0;
            @(negedge clk);                      // the FSM has now cleared done / raised busy
            t = 0;
            while (!done && t < timeout_us * 1000) begin #1 t = t + 1; end
            @(posedge clk); @(posedge clk);
            if (!done) $display("%0t  TB: TIMEOUT after %0d us (State %0d, busy %b)", $time, timeout_us, State, busy);
            else $display("%0t  TB: done. byte 0x%02X -> ACK_bit=%0d error_bit=%b  (%0d cycles = %0d us)",
                          $time, b, ACK_bit, error_bit, cycles, cycles / 101);
        end
    endtask

    integer s0, s1;
    initial begin
        $dumpfile("tb.vcd");
        $dumpvars(0, tb_i2c_transmit);

        // preload slave registers
        u_adt7420.mem[8'h00] = 8'h0C;   // temperature MSB (25.25 C)
        u_adt7420.mem[8'h01] = 8'hA0;   // temperature LSB
        u_adt7420.mem[8'h0B] = 8'hCB;   // ID
        u_lsm303a.mem[8'h20] = 8'h07;   // CTRL_REG1_A power-on default
        u_lsm303a.mem[8'h28] = 8'h00;   // OUT_X_L_A

        repeat (5) @(posedge clk); rst <= 1'b0;
        repeat (5) @(posedge clk);
        check("idle after reset: busy=0 done=0 State=0, lines released", !busy && !done && State == 8'd0 && !scl_low && !sda_low);

        $display("---- frame 1: 0x90 (ADT7420 write address) ----");
        s0 = stops;
        run_frame(8'h90, 500);
        check("frame 1: ADT7420 ACKed (ACK_bit = 0)", done && ACK_bit == 1'b0);
        check("frame 1: error_bit = 0", error_bit == 1'b0);
        check("frame 1: STOP seen", stops == s0 + 1);
        check("frame 1: transaction time 100..110 us (41 ticks x 2.5 us)", cycles >= 10000 && cycles <= 11100);
        check("frame 1: busy = 0, State back to 0, lines released", !busy && State == 8'd0 && !scl_low && !sda_low);

        $display("---- frame 2: 0x92 (address 0x49, no device) ----");
        run_frame(8'h92, 500);
        check("frame 2: NACK (ACK_bit = 1)", done && ACK_bit == 1'b1);
        check("frame 2: error_bit still 0 (a NACK is not an FSM fault)", error_bit == 1'b0);

        $display("---- frame 3: 0x32 (LSM303 accel write address) ----");
        run_frame(8'h32, 500);
        check("frame 3: LSM303 accel ACKed", done && ACK_bit == 1'b0);

        // ---- M2 steps 1-3: complete register read (param3[0] = 1, param2 = pointer)
        //      S 0x90 A 0x0B A Sr 0x91 A data NACK P
        $display("---- frame 3b: read register 0x0B (M2 steps 1-3) ----");
        param2 = 32'h0000_000B; param3 = 32'h0000_0001;
        run_frame(8'h90, 900);
        check("3b: exactly 4 bytes on the bus", bus_cnt == 4);
        check("3b: bus byte 1 = 0x90, ACKed", bus_byte[0] == 8'h90 && bus_ack[0] == 1'b0);
        check("3b: bus byte 2 = 0x0B, ACKed", bus_byte[1] == 8'h0B && bus_ack[1] == 1'b0);
        check("3b: FSM sampled pointer ACK (ACK_ptr = 0)", dut.ACK_ptr == 1'b0);
        check("3b: 2 STARTs: START before byte 1, repeated START after byte 2",
              start_n == 2 && start_at[0] == 0 && start_at[1] == 2);
        check("3b: bus byte 3 = 0x91, ACKed; slave took it as a READ (rw = 1)",
              bus_byte[2] == 8'h91 && bus_ack[2] == 1'b0 && u_adt7420.rw == 1'b1);
        check("3b: FSM sampled read-address ACK (ACK_rd = 0), ACK_bit = 0", dut.ACK_rd == 1'b0 && ACK_bit == 1'b0);
        check("3b: data byte on the bus = 0xCB", bus_byte[3] == 8'hCB);
        check("3b: master NACKed the data byte (9th clock high)", bus_ack[3] == 1'b1);
        check("3b: rx_data[7:0] (-> result1[7:0]) = 0xCB", rx_data == 32'h0000_00CB);
        check("3b: slave released SDA after the NACK (not transmitting, not pulling SDA)",
              u_adt7420.txmode == 1'b0 && u_adt7420.sda_low == 1'b0);
        check("3b: exactly 1 STOP, after byte 4 (none between 0x0B and 0x91)", stop_n == 1 && first_stop_at == 4);
        check("3b: error_bit = 0, busy = 0, State back to 0, lines released, SDA wire high",
              error_bit == 1'b0 && !busy && State == 8'd0 && !scl_low && !sda_low && sda === 1'b1);
        check("3b: time 380..395 us (153 ticks x 2.5 us)", cycles >= 38000 && cycles <= 39800);

        // register 0x00 = 0x0C: data bit 7 = 0, the case that hangs the bus without
        // the master NACK (frame 4). Also leaves the slave pointer at 0x00 for frame 4.
        $display("---- frame 3c: read register 0x00 (data MSB = 0, must not hang) ----");
        param2 = 32'h0000_0000;
        run_frame(8'h90, 900);
        check("3c: rx_data = 0x0C, data byte NACKed, 1 STOP after byte 4",
              rx_data == 32'h0000_000C && bus_byte[3] == 8'h0C && bus_ack[3] == 1'b1 && stop_n == 1 && first_stop_at == 4);
        check("3c: State back to 0, SDA wire high (no hang), slave pointer = 0x00",
              State == 8'd0 && !busy && sda === 1'b1 && u_adt7420.ptr == 8'h00);

        // Gate 4: absent device 0x49 in complete-read mode. "error" here is the
        // top level's status bit: error = error_bit | (done & ACK_bit).
        $display("---- frame 3d: read register 0x0B from absent device 0x49 (Gate 4) ----");
        param2 = 32'h0000_000B;
        run_frame(8'h92, 900);
        check("3d: first byte 0x92 NACKed on the bus", bus_byte[0] == 8'h92 && bus_ack[0] == 1'b1);
        check("3d: top-level error = 1 (ACK_bit = 1), error_bit = 0", (error_bit | (done & ACK_bit)) == 1'b1 && error_bit == 1'b0);
        check("3d: done = 1, busy = 0, State = 0", done && !busy && State == 8'd0);
        check("3d: SCL/SDA released, both wires high, STOP seen",
              !scl_low && !sda_low && scl === 1'b1 && sda === 1'b1 && stop_n == 1);
        param2 = 32'd0; param3 = 32'd0;          // back to the shipped single frame

        $display("---- frame 4: 0x91 (ADT7420 READ address) - demonstration ----");
        s0 = stops;
        run_frame(8'h91, 500);
        check("frame 4: ADT7420 ACKed the read address", done && ACK_bit == 1'b0);
        check("frame 4: param3[0] = 0 path reads nothing (rx_data = 0)", rx_data == 32'd0);
        if (stops == s0)
            $display("%0t  TB: NOTE frame 4: no STOP - the slave holds SDA low (data bit 7 = 0) after the ACK; SDA wire = %b. A read must clock the byte out and NACK it (Figure 17).", $time, sda);
        else
            $display("%0t  TB: NOTE frame 4: STOP seen (slave released SDA)", $time);

        repeat (10) @(posedge clk);
        $display("%0t  TB: finished, %0d failure(s)  ->  %0s", $time, fails, fails == 0 ? "RESULT: ALL PASS" : "RESULT: FAIL");
        $finish;
    end
endmodule
