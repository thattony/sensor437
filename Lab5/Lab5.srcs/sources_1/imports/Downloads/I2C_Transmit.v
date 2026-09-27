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
    input  wire [4:0]  okUH,
    output wire [2:0]  okHU,
    inout  wire [31:0] okUHU,
    inout  wire        okAA
    );

    //Instantiate the ClockGenerator module, where three signals are generate:
    //High speed CLK signal, Low speed FSM_Clk signal
    wire [23:0] ClkDivThreshold = 100;
    wire FSM_Clk, ILA_Clk;
    ClockGenerator ClockGenerator1 (  .sys_clkn(sys_clkn),
                                      .sys_clkp(sys_clkp),
                                      .ClkDivThreshold(ClkDivThreshold),
                                      .FSM_Clk(FSM_Clk),
                                      .ILA_Clk(ILA_Clk) );

    // Slave address 0x48 with the R/W bit set to 1 (read)
    reg [7:0] SingleByteData = 8'b1001_0001;    // 8'b10010001 
    reg error_bit = 1'b1;

    localparam STATE_INIT = 8'd0;

    assign led[7] = ACK_bit;
    assign led[6] = error_bit;
    assign ADT7420_A0 = 1'b0;
    assign ADT7420_A1 = 1'b0;

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
    end

    always @(*) begin
        FSM_Clk_reg = FSM_Clk;
        ILA_Clk_reg = ILA_Clk;
    end

    always @(posedge FSM_Clk) begin
        case (State)

// ***************************************************
// Wait here until the PC sets PC_control bit 0
// ***************************************************
            STATE_INIT : begin  if (PC_control[0] == 1'b1) State <= 8'd1;
                                else begin  SCL <= 1'b1; SDA <= 1'b1; State <= 8'd0; end   end

// ***************************************************
// Start sequence: SDA falls while SCL is high
// ***************************************************
            8'd1  :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b0; end
            8'd2  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b0; end

// ***************************************************
// Transmit the 1st BYTE to a sensor
// ***************************************************
            8'd3  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[7]; end   // transmit bit 7
            8'd4  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd5  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd6  :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd7  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[6]; end   // transmit bit 6
            8'd8  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd9  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd10 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd11 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[5]; end   // transmit bit 5
            8'd12 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd13 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd14 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd15 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[4]; end   // transmit bit 4
            8'd16 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd17 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd18 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd19 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[3]; end   // transmit bit 3
            8'd20 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd21 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd22 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd23 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[2]; end   // transmit bit 2
            8'd24 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd25 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd26 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd27 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[1]; end   // transmit bit 1
            8'd28 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd29 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd30 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd31 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[0]; end   // transmit bit 0
            8'd32 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd33 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd34 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Release SDA and read the ACK the sensor drives
// ***************************************************
            8'd35 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'bz; end   // let go of the line
            8'd36 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd37 :  begin  SCL <= 1'b1; State <= State + 1'b1; ACK_bit <= sda_in; end   // read the real wire
            8'd38 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Stop sequence: SDA rises while SCL is high
// ***************************************************
            8'd39 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b0; end
            8'd40 :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b0; end
            8'd41 :  begin  SCL <= 1'b1; SDA <= 1'b1; end   // halts here. Uncomment the line below to allow repeat runs.
         // 8'd41 :  begin  SCL <= 1'b1; SDA <= 1'b1; State <= STATE_INIT; end

            //If the FSM ends up here, there was an error in the FSM code.
            //LED[6] will turn on (active low) in that case.
            default : begin  error_bit <= 0; end

        endcase
    end


    // OK Interface
    wire [112:0]    okHE;  //These are FrontPanel wires needed to IO communication
    wire [64:0]     okEH;  //These are FrontPanel wires needed to IO communication
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

    //  PC_control is a wire that contains data sent from the PC to FPGA.
    //  The data is communicated via memory location 0x00
    okWireIn wire10 (   .okHE(okHE),
                        .ep_addr(8'h00),
                        .ep_dataout(PC_control));

endmodule
