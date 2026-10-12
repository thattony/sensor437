`timescale 1ns / 1ps

module I2C_Transmit(
    output [7:0] led,
    input  sys_clkn,
    input  sys_clkp,
    output ADT7420_A0,
    output ADT7420_A1,
    output I2C_SCL_0,
    inout  I2C_SDA_0,
    output reg FSM_Clk_reg,
    output reg ILA_Clk_reg,
    output reg ACK_bit,
    output reg SCL,
    output reg SDA,
    output reg [7:0] State,
    output wire [31:0] PC_control,
    output reg  ACK_marker,      // scope trigger (PMOD), high just before the read-address ACK
    input  wire [4:0]  okUH,
    output wire [2:0]  okHU,
    inout  wire [31:0] okUHU,
    inout  wire        okAA
    );

    //Instantiate the ClockGenerator module, where three signals are generate:
    //High speed CLK signal, Low speed FSM_Clk signal
    wire [23:0] ClkDivThreshold = 30;
    wire FSM_Clk, ILA_Clk;
    ClockGenerator ClockGenerator1 (  .sys_clkn(sys_clkn),
                                      .sys_clkp(sys_clkp),
                                      .ClkDivThreshold(ClkDivThreshold),
                                      .FSM_Clk(FSM_Clk),
                                      .ILA_Clk(ILA_Clk) );

    // Slave address 0x48 with the R/W bit set to 1 (read)
    reg [7:0] SingleByteData_w = 8'b1001_0000;    // write
    reg [7:0] SingleByteData_r = 8'b1001_0001;    // read
    
    // Register pointer, set in STATE_INIT before every transaction
    reg [7:0] Target = 8'h03;    // temperature = 8'h00; configuration = 8'h03; ID = 8'h0B

    // ADT7420 configuration register (0x03), bit 7 = resolution: 0 = 13-bit, 1 = 16-bit.
    // Bits 6:0 stay at their power-on default 0 (continuous conversion, fault queue 1,
    // INT/CT active low, interrupt mode), so the whole byte is 8'h80.
    localparam CONFIG_16BIT = 8'h80;
    reg [7:0] WriteData = CONFIG_16BIT;

    // Transaction type, set in STATE_INIT
    reg WriteOp  = 1'b1;    // 1: pointer + one data byte, then STOP (no read)
    reg TwoBytes = 1'b0;    // read: 1 = MSB + LSB (temperature), 0 = one byte

    reg [7:0] ReceivedData_msb, ReceivedData_lsb;

    // Results published to the host. Latched only when a transaction finishes,
    // so Python never sees a half-received byte.
    reg [15:0] TempData   = 16'h0000;
    reg [7:0]  ConfigData = 8'h00;
    reg [7:0]  IDData     = 8'h00;
    reg [15:0] ReadCount  = 16'd0;    // +1 per completed temperature read

    // Power-up sequence: 0 write config, 1 read config back, 2 read ID
    reg [1:0] InitStep = 2'd0;
    reg init_done      = 1'b0;
    reg init_ack_error = 1'b0;    // a slave ACK was missing during the power-up sequence
    reg temp_ack_error = 1'b0;    // a slave ACK was missing during the last temperature read
    reg ack_error      = 1'b0;    // a slave ACK was missing during the current transaction

    // PC_control[0] comes from the okClk domain: synchronise it, then act on the
    // rising edge only, so one host pulse gives exactly one temperature read.
    reg [1:0] go_sync = 2'b00;
    reg go_prev    = 1'b0;
    reg go_pending = 1'b0;

    reg error_bit = 1'b1;

    localparam STATE_INIT = 8'd0;

    assign led[7] = ACK_bit;
    assign led[6] = error_bit;
    assign ADT7420_A0 = 1'b0;   // has to be zero
    assign ADT7420_A1 = 1'b0;   // has to be zero

    assign I2C_SCL_0 = SCL;
    // SDA is the value we DRIVE. Assigning 1'bz releases the line so the
    // pull-up can take it high and another device can pull it low.
    assign I2C_SDA_0 = SDA;
    // sda_in is what is ACTUALLY on the wire. Always read this, never SDA:
    // while we are released, SDA holds 1'bz and tells you nothing.
    wire sda_in = I2C_SDA_0;

    initial  begin
        SCL = 1'b1;
        SDA = 1'b1;
        ACK_bit = 1'b1;
        State = 8'd0;
        ACK_marker = 1'b0;
    end

    // Scope marker for the SDA edge capture of the read-address ACK. Registered, so it is
    // glitch-free and lines up with State: it samples State 105..109 and is therefore
    // high while State = 106..110. A state's assignments take effect on the clock edge
    // that leaves it, so on the pins: marker rises, one FSM clock later SDA is released
    // for the last address bit (0x91, bit 0 = 1), SCL pulses high for that bit, SCL falls
    // and the sensor pulls SDA low (ACK), then the marker falls as the ACK clock starts.
    always @(posedge FSM_Clk) begin
        ACK_marker <= (State >= 8'd105) && (State <= 8'd109);
    end

    always @(*) begin
        FSM_Clk_reg = FSM_Clk;
        ILA_Clk_reg = ILA_Clk;
    end

    always @(posedge FSM_Clk) begin
        go_sync <= {go_sync[0], PC_control[0]};
        go_prev <= go_sync[1];
        if (go_sync[1] && !go_prev) go_pending <= 1'b1;    // remember the rising edge

        case (State)

// ***************************************************
// Idle. After reset, run the power-up sequence once:
//   write config (16-bit) -> read config back -> read ID
// Then wait for a rising edge on PC_control bit 0 and
// do exactly one temperature read per edge.
// ***************************************************
            STATE_INIT : begin
                SCL <= 1'b1; SDA <= 1'b1; ack_error <= 1'b0;
                if (!init_done) begin
                    case (InitStep)
                        2'd0    : begin  WriteOp <= 1'b1; TwoBytes <= 1'b0; Target <= 8'h03; end   // write config
                        2'd1    : begin  WriteOp <= 1'b0; TwoBytes <= 1'b0; Target <= 8'h03; end   // read config
                        default : begin  WriteOp <= 1'b0; TwoBytes <= 1'b0; Target <= 8'h0B; end   // read ID
                    endcase
                    State <= 8'd1;
                end
                else if (go_pending) begin
                    go_pending <= 1'b0;
                    WriteOp <= 1'b0; TwoBytes <= 1'b1; Target <= 8'h00;    // read temperature
                    State <= 8'd1;
                end
                else State <= STATE_INIT;
            end

// ***************************************************
// Start sequence: SDA falls while SCL is high
// ***************************************************
            8'd1  :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b0; end
            8'd2  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b0; end

// ***************************************************
// Transmit the WRITE BYTE to a sensor
// ***************************************************
            8'd3  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_w[7]; end   // transmit bit 7
            8'd4  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd5  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd6  :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd7  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_w[6]; end   // transmit bit 6
            8'd8  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd9  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd10 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd11 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_w[5]; end   // transmit bit 5
            8'd12 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd13 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd14 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd15 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_w[4]; end   // transmit bit 4
            8'd16 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd17 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd18 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd19 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_w[3]; end   // transmit bit 3
            8'd20 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd21 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd22 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd23 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_w[2]; end   // transmit bit 2
            8'd24 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd25 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd26 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd27 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_w[1]; end   // transmit bit 1
            8'd28 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd29 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd30 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd31 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_w[0]; end   // transmit bit 0
            8'd32 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd33 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd34 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Release SDA and read the ACK the sensor drives
// ***************************************************
            8'd35 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // let go of the line
            8'd36 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd37 :  begin  SCL <= 1'b1; State <= State + 1'b1; ACK_bit <= sda_in; if (sda_in) ack_error <= 1'b1; end   // read the real wire
            8'd38 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Transmit the target reg address
// ***************************************************
            8'd39 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= Target[7]; end   // transmit bit 7
            8'd40 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd41 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd42 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd43 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= Target[6]; end   // transmit bit 6
            8'd44 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd45 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd46 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd47 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= Target[5]; end   // transmit bit 5
            8'd48 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd49 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd50 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd51 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= Target[4]; end   // transmit bit 4
            8'd52 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd53 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd54 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd55 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= Target[3]; end   // transmit bit 3
            8'd56 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd57 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd58 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd59 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= Target[2]; end   // transmit bit 2
            8'd60 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd61 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd62 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd63 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= Target[1]; end   // transmit bit 1
            8'd64 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd65 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd66 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd67 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= Target[0]; end   // transmit bit 0
            8'd68 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd69 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd70 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Release SDA and read the ACK the sensor drives
// ***************************************************
            8'd71 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // let go of the line
            8'd72 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd73 :  begin  SCL <= 1'b1; State <= State + 1'b1; ACK_bit <= sda_in; if (sda_in) ack_error <= 1'b1; end   // read the real wire
            8'd74 :  begin  SCL <= 1'b0; State <= WriteOp ? 8'd190 : State + 1'b1; end   // write: go send the data byte

// ***************************************************
// Repeated Start: SDA falls while SCL is high
// ***************************************************
            
            8'd75 :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b1; end
            8'd76 :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b0; end
            8'd77 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Transmit the READ BYTE to a sensor
// ***************************************************
            8'd78  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_r[7]; end   // transmit bit 7
            8'd79  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd80  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd81  :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd82  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_r[6]; end   // transmit bit 6
            8'd83  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd84  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd85  :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd86  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_r[5]; end   // transmit bit 5
            8'd87  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd88  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd89  :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd90  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_r[4]; end   // transmit bit 4
            8'd91  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd92  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd93  :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd94  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_r[3]; end   // transmit bit 3
            8'd95  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd96  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd97  :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd98  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_r[2]; end   // transmit bit 2
            8'd99  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd100 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd101 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd102 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData_r[1]; end   // transmit bit 1
            8'd103 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd104 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd105 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            // bit 0 of 0x91 is a 1: send it open-drain (release, do not drive high). The pull-up
            // alone charges SDA (the slow RC rise), and the sensor's ACK pull-down after state
            // 109 no longer fights an FPGA output driving high.
            8'd106 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // transmit bit 0 (= 1, released)
            8'd107 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd108 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd109 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Release SDA and read the ACK the sensor drives
// ***************************************************
            8'd110 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // let go of the line
            8'd111 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd112 :  begin  SCL <= 1'b1; State <= State + 1'b1; ACK_bit <= sda_in; if (sda_in) ack_error <= 1'b1; end   // read the real wire
            8'd113 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            
// ***************************************************
// Receive from the sensor
// ***************************************************
            8'd114 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 7
            8'd115 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd116 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_msb[7] <= sda_in; end
            8'd117 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd118 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 6
            8'd119 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd120 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_msb[6] <= sda_in; end
            8'd121 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd122 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 5
            8'd123 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd124 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_msb[5] <= sda_in; end
            8'd125 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd126 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 4
            8'd127 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd128 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_msb[4] <= sda_in; end
            8'd129 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd130 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 3
            8'd131 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd132 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_msb[3] <= sda_in; end
            8'd133 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd134 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 2
            8'd135 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd136 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_msb[2] <= sda_in; end
            8'd137 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd138 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 1
            8'd139 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd140 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_msb[1] <= sda_in; end
            8'd141 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd142 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 0
            8'd143 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd144 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_msb[0] <= sda_in; end
            8'd145 :  begin  SCL <= 1'b0; State <= TwoBytes ? State + 1'b1 : 8'd182; end   // one byte: go straight to NACK

// ***************************************************
// Transmit ACK to sensor after receiving MSB
// ***************************************************
            8'd146 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b0; end   // master drives SDA low
            8'd147 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd148 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd149 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Receive LSB from the sensor
// ***************************************************
            8'd150 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 7
            8'd151 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd152 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_lsb[7] <= sda_in; end
            8'd153 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd154 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 6
            8'd155 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd156 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_lsb[6] <= sda_in; end
            8'd157 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd158 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 5
            8'd159 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd160 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_lsb[5] <= sda_in; end
            8'd161 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd162 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 4
            8'd163 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd164 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_lsb[4] <= sda_in; end
            8'd165 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd166 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 3
            8'd167 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd168 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_lsb[3] <= sda_in; end
            8'd169 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd170 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 2
            8'd171 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd172 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_lsb[2] <= sda_in; end
            8'd173 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd174 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 1
            8'd175 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd176 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_lsb[1] <= sda_in; end
            8'd177 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd178 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // receive bit 0
            8'd179 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd180 :  begin  SCL <= 1'b1; State <= State + 1'b1; ReceivedData_lsb[0] <= sda_in; end
            8'd181 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Transmit NACK to sensor (final byte of every read)
// ***************************************************
            8'd182 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // released = NACK
            8'd183 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd184 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd185 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Stop sequence: SDA rises while SCL is high
// ***************************************************
            8'd186 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b0; end
            8'd187 :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b0; end
            8'd188 :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b1; end
            8'd189 :  begin  SCL <= 1'b1; SDA <= 1'b1; State <= STATE_INIT;
                             // transaction finished: publish the result
                             if (!init_done) begin
                                 if (InitStep == 2'd1) ConfigData <= ReceivedData_msb;
                                 if (InitStep == 2'd2) begin  IDData <= ReceivedData_msb; init_done <= 1'b1; end
                                 init_ack_error <= init_ack_error | ack_error;
                                 InitStep <= InitStep + 1'b1;
                             end
                             else begin
                                 TempData <= {ReceivedData_msb, ReceivedData_lsb};
                                 temp_ack_error <= ack_error;
                                 ReadCount <= ReadCount + 1'b1;
                             end
                         end

// ***************************************************
// Transmit the DATA BYTE (configuration write only)
// ***************************************************
            8'd190 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[7]; end   // transmit bit 7
            8'd191 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd192 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd193 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd194 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[6]; end   // transmit bit 6
            8'd195 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd196 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd197 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd198 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[5]; end   // transmit bit 5
            8'd199 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd200 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd201 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd202 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[4]; end   // transmit bit 4
            8'd203 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd204 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd205 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd206 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[3]; end   // transmit bit 3
            8'd207 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd208 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd209 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd210 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[2]; end   // transmit bit 2
            8'd211 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd212 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd213 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd214 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[1]; end   // transmit bit 1
            8'd215 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd216 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd217 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd218 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[0]; end   // transmit bit 0
            8'd219 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd220 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd221 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Release SDA and read the ACK the sensor drives, then STOP
// ***************************************************
            8'd222 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // let go of the line
            8'd223 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd224 :  begin  SCL <= 1'b1; State <= State + 1'b1; ACK_bit <= sda_in; if (sda_in) ack_error <= 1'b1; end   // read the real wire
            8'd225 :  begin  SCL <= 1'b0; State <= 8'd186; end   // -> STOP

            //If the FSM ends up here, there was an error in the FSM code.
            //LED[6] will turn on (active low) in that case.
            default : begin  error_bit <= 0; end

        endcase
    end


    // OK Interface
    localparam integer endPt_count = 6;
    wire [112:0]    okHE;  //These are FrontPanel wires needed to IO communication
    wire [64:0]     okEH;  //These are FrontPanel wires needed to IO communication
    wire [endPt_count*65-1:0] okEHx;
    wire            okClk;

    //This is the OK host that allows data to be sent or recived
    okHost hostIF (
        .okUH(okUH),
        .okHU(okHU),
        .okUHU(okUHU),
        .okClk(okClk),
        .okAA(okAA),
        .okHE(okHE),
        .okEH(okEH)
    );

    okWireOR #(.N(endPt_count)) wireOR (
        .okEH(okEH),
        .okEHx(okEHx)
    );

    // 0x20/0x21: last temperature word (MSB, LSB), latched at the end of the read
    okWireOut wire20 (
        .okHE(okHE),
        .okEH(okEHx[0*65 +: 65]),
        .ep_addr(8'h20),
        .ep_datain(TempData[15:8])
    );

    okWireOut wire21 (
        .okHE(okHE),
        .okEH(okEHx[1*65 +: 65]),
        .ep_addr(8'h21),
        .ep_datain(TempData[7:0])
    );

    // 0x22: configuration register read back after the power-up write (expect 0x80)
    okWireOut wire22 (
        .okHE(okHE),
        .okEH(okEHx[2*65 +: 65]),
        .ep_addr(8'h22),
        .ep_datain(ConfigData)
    );

    // 0x23: ID register read during power-up (expect 0xCB)
    okWireOut wire23 (
        .okHE(okHE),
        .okEH(okEHx[3*65 +: 65]),
        .ep_addr(8'h23),
        .ep_datain(IDData)
    );

    // 0x24: status. bit 0 init_done, bit 1 init_ack_error, bit 2 temp_ack_error
    okWireOut wire24 (
        .okHE(okHE),
        .okEH(okEHx[4*65 +: 65]),
        .ep_addr(8'h24),
        .ep_datain({29'd0, temp_ack_error, init_ack_error, init_done})
    );

    // 0x25: number of completed temperature reads. Python waits for this to step.
    okWireOut wire25 (
        .okHE(okHE),
        .okEH(okEHx[5*65 +: 65]),
        .ep_addr(8'h25),
        .ep_datain(ReadCount)
    );

    //  PC_control is a wire that contains data sent from the PC to FPGA.
    //  The data is communicated via memory location 0x00
    okWireIn wire10 (   
        .okHE(okHE),
        .ep_addr(8'h00),
        .ep_dataout(PC_control)
    );

endmodule
