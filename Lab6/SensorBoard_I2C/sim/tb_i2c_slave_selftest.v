`timescale 1ns / 1ps
//============================================================================
// tb_i2c_slave_selftest.v - Self-test of sim/i2c_slave_model.v with a
// bit-banged behavioural I2C master. Also a readable reference for the exact
// order of events in an I2C register read/write (100 kHz, quarter-bit = 2.5 us).
//
// Run (inside a Vivado shell, from sim/):
//   xvlog i2c_slave_model.v tb_i2c_slave_selftest.v && xelab -top tb_i2c_slave_selftest -snapshot st && xsim st -R
//============================================================================
module tb_i2c_slave_selftest;
    localparam Q = 2500;                      // quarter bit period (ns) -> 100 kHz SCL

    reg scl_low = 0, sda_low = 0;
    wire scl, sda;
    pullup (scl); pullup (sda);
    assign scl = scl_low ? 1'b0 : 1'bz;
    assign sda = sda_low ? 1'b0 : 1'bz;

    i2c_slave_model #(.ADDR(7'h48), .NAME("ADT7420")) u_slave (.scl(scl), .sda(sda));

    integer errors = 0;
    task expect(input [7:0] got, input [7:0] want, input [8*24:1] what);
        begin
            if (got !== want) begin errors = errors + 1; $display("  FAIL %0s: got 0x%02X want 0x%02X", what, got, want); end
            else $display("  ok   %0s = 0x%02X", what, got);
        end
    endtask

    // ---- master primitives (SCL low when entering, unless noted) ----------
    task i2c_start;                          // from idle (SCL/SDA high) or repeated (SCL low)
        begin
            sda_low = 0; #Q; scl_low = 0; #Q;   // both released -> high
            sda_low = 1; #Q;                    // SDA falls while SCL high = START
            scl_low = 1; #Q;
        end
    endtask
    task i2c_stop;
        begin
            sda_low = 1; #Q; scl_low = 0; #Q; sda_low = 0; #Q;   // SDA rises while SCL high = STOP
        end
    endtask
    task i2c_write_byte(input [7:0] b, output ack);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                sda_low = ~b[i]; #Q; scl_low = 0; #(2*Q); scl_low = 1; #Q;
            end
            sda_low = 0; #Q; scl_low = 0; #Q; ack = (sda === 1'b0); #Q; scl_low = 1; #Q;  // ACK slot
        end
    endtask
    task i2c_read_byte(input send_ack, output [7:0] b);
        integer i;
        begin
            sda_low = 0;
            for (i = 7; i >= 0; i = i - 1) begin
                #Q; scl_low = 0; #Q; b[i] = sda; #Q; scl_low = 1; #Q;
            end
            sda_low = send_ack; #Q; scl_low = 0; #(2*Q); scl_low = 1; #Q; sda_low = 0;   // master ACK/NACK
        end
    endtask

    reg ack; reg [7:0] d0, d1, d2;
    initial begin
        u_slave.mem[8'h00] = 8'h0C; u_slave.mem[8'h01] = 8'hA0; u_slave.mem[8'h0B] = 8'hCB;
        #(4*Q);

        $display("--- 1. write pointer 0x0B, repeated START, read 1 byte (ID)");
        i2c_start; i2c_write_byte(8'h90, ack); expect({7'd0, ack}, 8'h01, "addr+W ACK");
        i2c_write_byte(8'h0B, ack);            expect({7'd0, ack}, 8'h01, "pointer ACK");
        i2c_start; i2c_write_byte(8'h91, ack); expect({7'd0, ack}, 8'h01, "addr+R ACK");
        i2c_read_byte(1'b0, d0);               expect(d0, 8'hCB, "ID reg");
        i2c_stop;

        $display("--- 2. read 2 bytes from 0x00 with auto-increment (ACK then NACK)");
        i2c_start; i2c_write_byte(8'h90, ack); i2c_write_byte(8'h00, ack);
        i2c_start; i2c_write_byte(8'h91, ack);
        i2c_read_byte(1'b1, d0); i2c_read_byte(1'b0, d1);
        expect(d0, 8'h0C, "temp MSB"); expect(d1, 8'hA0, "temp LSB");
        i2c_stop;

        $display("--- 3. register write 0x03 <= 0x80, then read back");
        i2c_start; i2c_write_byte(8'h90, ack); i2c_write_byte(8'h03, ack); i2c_write_byte(8'h80, ack);
        expect({7'd0, ack}, 8'h01, "data ACK"); i2c_stop;
        i2c_start; i2c_write_byte(8'h90, ack); i2c_write_byte(8'h03, ack);
        i2c_start; i2c_write_byte(8'h91, ack); i2c_read_byte(1'b0, d2); expect(d2, 8'h80, "cfg readback"); i2c_stop;

        $display("--- 4. wrong address 0x49 must NACK and the bus must read 0xFF");
        i2c_start; i2c_write_byte(8'h92, ack); expect({7'd0, ack}, 8'h00, "0x49 NACK");
        i2c_read_byte(1'b0, d0); expect(d0, 8'hFF, "idle bus"); i2c_stop;

        $display("--- 5. after the NACK the slave must still answer");
        i2c_start; i2c_write_byte(8'h90, ack); i2c_write_byte(8'h0B, ack);
        i2c_start; i2c_write_byte(8'h91, ack); i2c_read_byte(1'b0, d0); expect(d0, 8'hCB, "ID again"); i2c_stop;

        #(4*Q);
        if (errors == 0) $display("SELFTEST PASS"); else $display("SELFTEST FAIL (%0d errors)", errors);
        $finish;
    end
endmodule
