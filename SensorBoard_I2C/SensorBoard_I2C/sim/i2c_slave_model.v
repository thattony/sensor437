`timescale 1ns / 1ps
//============================================================================
// i2c_slave_model.v - Behavioural I2C slave for simulation only.
//
// A generic register-file slave: 256 x 8-bit memory, 7-bit address ADDR,
// pointer set by the first byte after a write address, auto-increment on
// every data byte (this is how ADT7420, HTS221*, LPS35HW, LSM303DLHC* and
// AD7156 behave; *ST parts need bit 7 of the sub-address set for
// auto-increment - this model always auto-increments).
//
// Preload registers from the testbench:  u_slave.mem[8'h0B] = 8'hCB;
// Prints START/STOP/address/byte events with $display so a wrong FSM is easy
// to spot in the log. Timing is not checked - use the datasheet for that.
//============================================================================
module i2c_slave_model #(
    parameter [6:0] ADDR = 7'h48,
    parameter       NAME = "slave"
)(
    input  wire scl,
    inout  wire sda
);
    reg [7:0] mem [0:255];
    reg [7:0] ptr = 8'h00;

    reg       sda_low = 1'b0;          // 1 = we pull SDA low
    assign sda = sda_low ? 1'b0 : 1'bz;

    reg       started = 1'b0;          // between START and STOP
    reg       active  = 1'b0;          // address matched
    reg       txmode  = 1'b0;          // we are transmitting (master reads)
    reg       rw      = 1'b0;
    reg       ptr_phase = 1'b0;        // next received byte is the register pointer
    reg [7:0] rxshift = 8'h00;
    reg [7:0] txshift = 8'h00;
    reg       master_ack = 1'b0;
    integer   bitcnt = 0;              // 0..7 data bits, 8 = waiting for ACK clock, 9 = in ACK clock

    integer i;
    initial for (i = 0; i < 256; i = i + 1) mem[i] = 8'h00;

    // START: SDA falls while SCL is high (also a repeated START)
    always @(negedge sda) if (scl === 1'b1) begin
        started   <= 1'b1;
        active    <= 1'b0;
        txmode    <= 1'b0;
        bitcnt    <= 0;
        sda_low   <= 1'b0;
        $display("%0t  %s: START", $time, NAME);
    end

    // STOP: SDA rises while SCL is high
    always @(posedge sda) if (scl === 1'b1 && started) begin
        started <= 1'b0;
        active  <= 1'b0;
        txmode  <= 1'b0;
        sda_low <= 1'b0;
        $display("%0t  %s: STOP", $time, NAME);
    end

    // Rising edge of SCL: sample
    always @(posedge scl) if (started) begin
        if (bitcnt < 8) begin
            if (!txmode) rxshift <= {rxshift[6:0], sda};
            bitcnt <= bitcnt + 1;
        end else if (bitcnt == 8) begin
            if (txmode) master_ack <= (sda === 1'b0);
            bitcnt <= 9;
        end
    end

    // Falling edge of SCL: drive
    always @(negedge scl) if (started) begin
        if (bitcnt == 8) begin
            // 8 bits done: ACK slot begins
            if (txmode) begin
                sda_low <= 1'b0;                          // release for the master's ACK/NACK
            end else if (!active) begin
                if (rxshift[7:1] == ADDR) begin
                    active    <= 1'b1;
                    rw        <= rxshift[0];
                    ptr_phase <= ~rxshift[0];
                    sda_low   <= 1'b1;                    // ACK our address
                    $display("%0t  %s: address 0x%02X matched, %s", $time, NAME, rxshift[7:1], rxshift[0] ? "READ" : "WRITE");
                end else begin
                    sda_low <= 1'b0;                      // not for us: NACK (stay released)
                    $display("%0t  %s: address 0x%02X ignored (mine is 0x%02X)", $time, NAME, rxshift[7:1], ADDR);
                end
            end else if (ptr_phase) begin
                ptr       <= rxshift;
                ptr_phase <= 1'b0;
                sda_low   <= 1'b1;
                $display("%0t  %s: register pointer <= 0x%02X", $time, NAME, rxshift);
            end else begin
                mem[ptr]  <= rxshift;
                ptr       <= ptr + 1'b1;
                sda_low   <= 1'b1;
                $display("%0t  %s: write mem[0x%02X] <= 0x%02X", $time, NAME, ptr, rxshift);
            end
        end else if (bitcnt == 9) begin
            // ACK slot ends
            bitcnt <= 0;
            if (txmode) begin
                if (master_ack) begin
                    ptr     <= ptr + 1'b1;
                    txshift <= mem[ptr + 1'b1];
                    sda_low <= ~mem[ptr + 1'b1][7];
                    $display("%0t  %s: master ACK, sending mem[0x%02X] = 0x%02X", $time, NAME, ptr + 1'b1, mem[ptr + 1'b1]);
                end else begin
                    txmode  <= 1'b0;
                    active  <= 1'b0;
                    sda_low <= 1'b0;
                    $display("%0t  %s: master NACK, releasing bus", $time, NAME);
                end
            end else begin
                sda_low <= 1'b0;                          // our ACK is over
                if (active && rw) begin                   // read address just ACKed: start sending
                    txmode  <= 1'b1;
                    txshift <= mem[ptr];
                    sda_low <= ~mem[ptr][7];
                    $display("%0t  %s: sending mem[0x%02X] = 0x%02X", $time, NAME, ptr, mem[ptr]);
                end
            end
        end else if (txmode && bitcnt >= 1 && bitcnt <= 7) begin
            sda_low <= ~txshift[7 - bitcnt];              // next data bit
        end
    end
endmodule
