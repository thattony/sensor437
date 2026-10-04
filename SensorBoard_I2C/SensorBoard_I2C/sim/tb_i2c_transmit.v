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
            tx_byte = b; cycles = 0;
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

    integer s0;
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

        $display("---- frame 4: 0x91 (ADT7420 READ address) - demonstration ----");
        s0 = stops;
        run_frame(8'h91, 500);
        check("frame 4: ADT7420 ACKed the read address", done && ACK_bit == 1'b0);
        if (stops == s0)
            $display("%0t  TB: NOTE frame 4: no STOP - the slave holds SDA low (data bit 7 = 0) after the ACK; SDA wire = %b. A read must clock the byte out and NACK it (Figure 17).", $time, sda);
        else
            $display("%0t  TB: NOTE frame 4: STOP seen (slave released SDA)", $time);

        repeat (10) @(posedge clk);
        $display("%0t  TB: finished, %0d failure(s)  ->  %0s", $time, fails, fails == 0 ? "RESULT: ALL PASS" : "RESULT: FAIL");
        $finish;
    end
endmodule
