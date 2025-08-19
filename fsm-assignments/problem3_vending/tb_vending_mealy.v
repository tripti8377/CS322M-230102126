`timescale 1ns/1ps
module tb_vending_mealy;
    reg clk, rst;
    reg [1:0] coin;
    wire dispense, chg5;

    // DUT instantiation
    vending_mealy dut (
        .clk(clk), .rst(rst), .coin(coin),
        .dispense(dispense), .chg5(chg5)
    );

    // Clock generation (10ns period)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Stimulus
    initial begin
        $dumpfile("waves/dump.vcd");
        $dumpvars(0, tb_vending_mealy);

        // Reset
        rst = 1; coin = 2'b00;
        #12;
        rst = 0;

        // Case 1: 10 + 10 = vend
        coin = 2'b10; #10; coin = 2'b00; #10;
        coin = 2'b10; #10; coin = 2'b00; #10;

        // Case 2: 5 + 5 + 10 = vend
        coin = 2'b01; #10; coin = 2'b00; #10;
        coin = 2'b01; #10; coin = 2'b00; #10;
        coin = 2'b10; #10; coin = 2'b00; #10;

        // Case 3: 10 + 10 + 5 = vend + chg5
        coin = 2'b10; #10; coin = 2'b00; #10;
        coin = 2'b10; #10; coin = 2'b00; #10;
        coin = 2'b01; #10; coin = 2'b00; #10;

        #50 $finish;
    end
endmodule
