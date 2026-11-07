`timescale 1ns / 1ps
//============================================================================
// I2C_Transmit.v - THIS IS THE FILE YOU BRING: paste your completed I2C
// master from the I2C lab here. It ships as a stub; the port list is FROZEN
// and identical to that lab's (hdl/SensorBoard_Top.v instantiates it as
// u_i2c and owns the pins, the bus selector and the PC endpoints).
//
// ONE TRANSACTION (ADT7420.pdf Figures 14, 16, 17 p.18-19; same for every
// I2C sensor on the board):
//
//   START
//   frame 1  SAD+W     (param1[7:0], e.g. 0x90)                     slave ACK
//   frame 2  register  (param2[7:0])                                slave ACK
//   -- write (param2[16] = 1) ---------------------------------------------
//   frame 3  data      (param2[15:8])                               slave ACK
//   -- read  (param2[16] = 0) ---------------------------------------------
//   REPEATED START
//   frame 3  SAD+R     ({param1[7:1], 1}, e.g. 0x91)                slave ACK
//   receive byte 1 -> master ACK, ..., receive last byte -> master NACK
//   (param3[7:0] bytes, 1..4; 0 behaves as 1, more than 4 as 4)
//   -----------------------------------------------------------------------
//   STOP
//
// Received bytes land in rx_data, first byte in [31:24] (WireOut 0x21).
// ACK_frames records each frame's ACK as sampled in its 9th clock (0 = ACK,
// 1 = NACK; bit 0 = frame 1); a NACK on any frame aborts straight to STOP,
// so a wrong address never hangs the bus. SCL = 100 kHz (one FSM state per
// tick of TICK_DIVIDE clocks). Open drain: scl_low/sda_low = 1 pulls the
// line low, 0 releases it - the FPGA never drives a line high; read the
// real wire on sda_in. State is your state register (WireOut 0x23 [15:8],
// 0 must mean idle).
//
// sim/tb_i2c_transmit.v (7 transactions, one RESULT line) must pass again
// once your machine is in - run `python build.py --sim`. On hardware,
// python/temperature.py is the acceptance test.
//============================================================================
module I2C_Transmit #(
    parameter integer TICK_DIVIDE = 252   // clocks per state: 100.8 MHz / 252 = 2.5 us -> 100 kHz SCL
)(
    input  wire        clk,         // 100.8 MHz
    input  wire        rst,         // synchronous, active high
    input  wire        start,       // one-clock pulse: run one transaction (ignored while busy)
    input  wire [7:0]  tx_byte,     // param1[7:0] = SAD+W = {7-bit slave address, 0}, e.g. 0x90
    input  wire [31:0] param2,      // [7:0] register address, [15:8] data byte to write, [16] 1 = write / 0 = read
    input  wire [31:0] param3,      // [7:0] number of bytes to read (1..4; 0 -> 1, >4 -> 4)
    output wire        scl_low,     // 1 = pull SCL low, 0 = release   (open drain)
    output wire        sda_low,     // 1 = pull SDA low, 0 = release
    input  wire        scl_in,      // real level on the SCL wire (for clock stretching)
    input  wire        sda_in,      // real level on the SDA wire (the slave's ACK / data)
    output reg  [3:0]  ACK_frames,  // bit n = ACK of frame n+1 as sampled in its 9th clock: 0 = ACK, 1 = NACK
    output reg  [31:0] rx_data,     // bytes received, first byte in [31:24]  (WireOut 0x21)
    output reg         busy,        // 1 while the transaction is on the bus
    output reg         done,        // 1 when rx_data / ACK_frames are valid (until the next start/rst)
    output reg         error_bit,   // 1 = FSM error (fell into the default arm)
    output reg  [7:0]  State        // your state register (WireOut 0x23 [15:8], 0 = idle)
);
    // ---- STUB: replace everything below with your I2C master ----------------
    assign scl_low = 1'b0;              // safe: both lines released
    assign sda_low = 1'b0;
    initial begin
        ACK_frames = 4'd0; rx_data = 32'd0; busy = 1'b0; done = 1'b0; error_bit = 1'b0; State = 8'd0;
    end
    always @(posedge clk) begin
        ACK_frames <= 4'd0;
        rx_data    <= 32'd0;
        busy       <= 1'b0;
        done       <= 1'b0;
        error_bit  <= 1'b0;
        State      <= 8'd0;
    end
endmodule
