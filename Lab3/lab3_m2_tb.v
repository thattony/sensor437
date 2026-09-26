`timescale 1ns / 1ps

module lab3_m2_TestBench();

    reg clk = 0;
    reg ped_request = 0;
    wire [7:0] lights;

    wire NS_RED = lights[0];
    wire NS_GREEN = lights[2];
    wire EW_RED = lights[3];
    wire EW_GREEN = lights[5];
    wire PED_GREEN = lights[7];

    reg vehicle_green_pass = 1;
    reg pedestrian_pass = 1;

    lab3_traffic_fsm #(
        .TICK(10)
    ) dut (
        .clk(clk),
        .ped_request(ped_request),
        .lights(lights)
    );

    // 200 MHz clock -> period = 5 ns
    always begin
        #2.5 clk = ~clk;
    end

    always @(posedge clk) begin

        // Check 1:
        // NS green and EW green must never be on together
        if (NS_GREEN && EW_GREEN)
            vehicle_green_pass <= 0;

        // Check 2:
        // Pedestrian green only when both vehicle lights are red
        if (PED_GREEN && !(NS_RED && EW_RED))
            pedestrian_pass <= 0;

    end

    initial begin

        // Start with no pedestrian request
        ped_request = 0;

        // Show one complete normal traffic cycle
        #350;

        // Send pedestrian request mid-green
        ped_request = 1;
        #10;
        ped_request = 0;

        // Allow enough time to finish pedestrian sequence
        #500;

        // Print results
        if (vehicle_green_pass)
            $display("PASS: vehicle greens are never on together");
        else
            $display("FAIL: vehicle greens were on together");

        if (pedestrian_pass)
            $display("PASS: pedestrian green only occurs while both vehicles are red");
        else
            $display("FAIL: pedestrian green occurred without both vehicles red");

        // Intentionally create an unsafe output to test the checker
        $display("Starting intentional FAIL test...");
        force dut.lights = 8'b00100100;
        #10;
        release dut.lights;
        #100;

        if (vehicle_green_pass)
            $display("ERROR: checker did not detect intentional failure");
        else
            $display("EXPECTED FAIL: vehicle greens were on together");

        $finish;
    end

endmodule