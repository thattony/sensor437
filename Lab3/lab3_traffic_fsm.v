`timescale 1ns / 1ps

// lab3_traffic_fsm.v -- the state machine. No okHost, no FrontPanel, plain ports.
module lab3_traffic_fsm #(
    parameter TICK = 100000000        // 200 MHz cycles in one tick
) (
    input        clk,
    input        ped_request,
    output reg [7:0] lights               // 1 = lit, before the active-low inversion
);
    // your divider, your state machine, your light assignments
    
    reg [2:0] state = 0;
    reg [26:0] tick_count = 0;
    reg [1:0] sec_count = 0;
    reg ped_flag = 1'b0;

    localparam NS_GREEN    = 3'd0;
    localparam NS_YELLOW   = 3'd1;
    localparam EW_GREEN   = 3'd2;
    localparam EW_YELLOW = 3'd3;
    localparam PED_GREEN = 3'd4;
    
    wire tick_done;
    assign tick_done = (tick_count == TICK - 1);
    
    always @(posedge clk) begin
        // track the tick count
         if (tick_done) tick_count <= 27'd0;
         else tick_count <= tick_count + 1'b1;
         
         // set ped flag
        if (state == PED_GREEN)  ped_flag <= 1'b0;
        else if (ped_request) ped_flag <= 1'b1;
    end

    always @(posedge clk) begin
        case (state)
    
            NS_GREEN : begin
                // G1,R2,R3 = 1
                lights <= 8'b01001100;
    
                if (tick_done) begin
                    if (sec_count == 2'd1) begin
                        state <= NS_YELLOW;
                        sec_count <= 2'd0;
                    end
                    else
                        sec_count <= sec_count + 1'b1;
                end
            end
    
            NS_YELLOW : begin
                // Y1,R2,R3 = 1
                lights <= 8'b01001010;
    
                if (tick_done) begin
                    if (ped_flag || ped_request)
                        state <= PED_GREEN;
                    else
                        state <= EW_GREEN;
    
                    sec_count <= 2'd0;
                end
            end
    
            EW_GREEN : begin
                // G2,R1,R3 = 1
                lights <= 8'b01100001;
    
                if (tick_done) begin
                    if (sec_count == 2'd1) begin
                        state <= EW_YELLOW;
                        sec_count <= 2'd0;
                    end
                    else
                        sec_count <= sec_count + 1'b1;
                end
            end
    
            EW_YELLOW : begin
                // Y2,R1,R3 = 1
                lights <= 8'b01010001;
    
                if (tick_done) begin
                    if (ped_flag || ped_request)
                        state <= PED_GREEN;
                    else
                        state <= NS_GREEN;
    
                    sec_count <= 2'd0;
                end
            end
    
            PED_GREEN : begin
                // G3,R1,R2 = 1
                lights <= 8'b10001001;
    
                if (tick_done) begin
                    if (sec_count == 2'd1) begin
                        state <= NS_GREEN;
                        sec_count <= 2'd0;
                    end
                    else
                        sec_count <= sec_count + 1'b1;
                end
            end
    
            default : begin
                state <= NS_GREEN;
                sec_count <= 2'd0;
            end
    
        endcase
    end
endmodule