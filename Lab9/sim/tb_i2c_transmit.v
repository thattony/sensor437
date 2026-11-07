`timescale 1ns / 1ps
//============================================================================
// tb_i2c_transmit.v - Testbench for hdl/I2C_Transmit.v (the file you edit).
//
// Run:  ./build.sh --sim        (xsim, log in sim/sim.log, waveform sim/tb.vcd)
// or open the Vivado project in build/vivado and "Run Simulation".
//
// The state machine is simulated on its own (the top level only adds pads,
// the bus selector and the PC endpoints). One I2C bus with a pull-up carries
// two behavioural slaves from sim/i2c_slave_model.v: an "ADT7420" at 0x48
// (ID 0x0B = 0xCB, temperature 0x00/0x01 = 0x0C 0xA0) and an "LSM303 accel"
// at 0x19. The slaves print START / STOP / address / byte events, so a
// protocol error is visible in the log without a scope.
//
// Transactions (param2 = {write, data, reg}, param3 = bytes to read):
//   1. read  ADT7420 reg 0x0B, 1 byte      -> 0xCB, all three frames ACKed, STOP seen
//   2. read  ADT7420 reg 0x00, 2 bytes     -> 0x0CA0 (temperature MSB:LSB), master NACKs the last byte
//   3. write ADT7420 reg 0x03 <= 0x80      -> slave prints "write mem[0x03] <= 0x80"
//   4. read  ADT7420 reg 0x03, 1 byte      -> 0x80 (read-back of the write)
//   5. read  address 0x49 (nobody)         -> NACK on frame 1, FSM aborts to STOP, rx_data = 0
//   6. read  ADT7420 reg 0x0B, 0 bytes     -> treated as 1 byte (robustness)
//   7. read  LSM303 accel 0x19 reg 0x20    -> 0x07 (second slave on the same bus)
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
    wire [3:0]  ACK_frames;
    wire        busy, done, error_bit;
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
        .ACK_frames(ACK_frames), .rx_data(rx_data),
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

    // ---- one transaction -------------------------------------------------
    task run_txn(input [7:0] sad_w, input write, input [7:0] reg_addr, input [7:0] wdata, input [7:0] nbytes, input integer timeout_us);
        integer t;
        begin
            tx_byte = sad_w; param2 = {15'd0, write, wdata, reg_addr}; param3 = {24'd0, nbytes}; cycles = 0;
            @(posedge clk); start <= 1'b1; @(posedge clk); start <= 1'b0;
            @(negedge clk);                      // the FSM has now cleared done / raised busy
            t = 0;
            while (!done && t < timeout_us * 1000) begin #1 t = t + 1; end
            @(posedge clk); @(posedge clk);
            if (!done) $display("%0t  TB: TIMEOUT after %0d us (State %0d, busy %b)", $time, timeout_us, State, busy);
            else $display("%0t  TB: done. %s SAD 0x%02X reg 0x%02X -> ACK_frames=%b rx_data=0x%08X error_bit=%b  (%0d cycles = %0d us)",
                          $time, write ? "WRITE" : "READ ", sad_w, reg_addr, ACK_frames, rx_data, error_bit, cycles, cycles / 101);
        end
    endtask

    integer s0;
    initial begin
        if ($test$plusargs("vcd")) begin        // xsim tb_i2c -R -testplusarg vcd
            $dumpfile("tb.vcd");
            $dumpvars(0, tb_i2c_transmit);
        end

        // preload slave registers
        u_adt7420.mem[8'h00] = 8'h0C;   // temperature MSB (25.25 C)
        u_adt7420.mem[8'h01] = 8'hA0;   // temperature LSB
        u_adt7420.mem[8'h03] = 8'h00;   // configuration
        u_adt7420.mem[8'h0B] = 8'hCB;   // ID
        u_lsm303a.mem[8'h20] = 8'h07;   // CTRL_REG1_A power-on default
        u_lsm303a.mem[8'h28] = 8'h00;   // OUT_X_L_A

        repeat (5) @(posedge clk); rst <= 1'b0;
        repeat (5) @(posedge clk);
        check("idle after reset: busy=0 done=0 State=0, lines released", !busy && !done && State == 8'd0 && !scl_low && !sda_low);

        $display("---- 1: read ADT7420 ID register 0x0B (1 byte) ----");
        s0 = stops;
        run_txn(8'h90, 1'b0, 8'h0B, 8'h00, 8'd1, 2000);
        check("1: ID = 0xCB in rx_data[31:24]", done && rx_data[31:24] == 8'hCB);
        check("1: all three frames ACKed", ACK_frames == 4'b0000);
        check("1: STOP seen, lines released, State 0", stops == s0 + 1 && !scl_low && !sda_low && State == 8'd0);
        check("1: transaction time 350..420 us", cycles >= 35000 && cycles <= 42500);

        $display("---- 2: read ADT7420 temperature 0x00/0x01 (2 bytes) ----");
        run_txn(8'h90, 1'b0, 8'h00, 8'h00, 8'd2, 2000);
        check("2: rx_data[31:16] = 0x0CA0", done && rx_data[31:16] == 16'h0CA0);
        check("2: no error", ACK_frames == 4'b0000 && error_bit == 1'b0);

        $display("---- 3: write ADT7420 configuration 0x03 <= 0x80 ----");
        s0 = stops;
        run_txn(8'h90, 1'b1, 8'h03, 8'h80, 8'd0, 2000);
        check("3: slave memory updated", u_adt7420.mem[8'h03] == 8'h80);
        check("3: three frames ACKed, STOP seen", ACK_frames == 4'b0000 && stops == s0 + 1);

        $display("---- 4: read back configuration 0x03 ----");
        run_txn(8'h90, 1'b0, 8'h03, 8'h00, 8'd1, 2000);
        check("4: read-back = 0x80", rx_data[31:24] == 8'h80);

        $display("---- 5: read from address 0x49 (nobody there) ----");
        s0 = stops;
        run_txn(8'h92, 1'b0, 8'h0B, 8'h00, 8'd1, 2000);
        check("5: frame 1 NACKed (ACK_frames[0] = 1)", done && ACK_frames[0] == 1'b1);
        check("5: aborted to STOP after frame 1 (~100 us), rx_data = 0", stops == s0 + 1 && rx_data == 32'd0 && cycles <= 12000);
        check("5: lines released, State 0", !scl_low && !sda_low && State == 8'd0);

        $display("---- 6: read with byte count 0 (must behave as 1) ----");
        run_txn(8'h90, 1'b0, 8'h0B, 8'h00, 8'd0, 2000);
        check("6: byte count 0 -> one byte, ID = 0xCB", done && rx_data[31:24] == 8'hCB && ACK_frames == 4'b0000);

        $display("---- 7: read LSM303 accel CTRL_REG1_A (0x20) on the same bus ----");
        run_txn(8'h32, 1'b0, 8'h20, 8'h00, 8'd1, 2000);
        check("7: LSM303 answered 0x07", done && rx_data[31:24] == 8'h07 && ACK_frames == 4'b0000);

        repeat (10) @(posedge clk);
        $display("%0t  TB: finished, %0d failure(s)  ->  %0s", $time, fails, fails == 0 ? "RESULT: ALL PASS" : "RESULT: FAIL");
        $finish;
    end
endmodule
