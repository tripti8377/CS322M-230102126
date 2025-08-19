`timescale 1ns/1ps

module tb_traffic_light;
    reg clk, rst, tick;
    wire ns_g, ns_y, ns_r, ew_g, ew_y, ew_r;

    // Instantiate DUT
    traffic_light dut(
        .clk(clk), .rst(rst), .tick(tick),
        .ns_g(ns_g), .ns_y(ns_y), .ns_r(ns_r),
        .ew_g(ew_g), .ew_y(ew_y), .ew_r(ew_r)
    );

    // Clock generation
    initial clk = 0;
    always #5 clk = ~clk;  // 100MHz clock (10ns period)

    // Test sequence
    initial begin
        $dumpfile("dump.vcd"); 
        $dumpvars(0, tb_traffic_light);

        rst = 1; tick = 0;
        #20 rst = 0;

        // Generate 1Hz tick pulses
        forever begin
            #100 tick = 1;   // 1 cycle tick high
            #10  tick = 0;   // back low
        end
    end

    // Stop simulation after 40s
    initial begin
        #400000 $finish;
    end
endmodule
