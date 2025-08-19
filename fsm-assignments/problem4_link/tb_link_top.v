`timescale 1ns/1ps
module tb_link_top;

    reg clk, rst;
    wire done;

    link_top dut(.clk(clk), .rst(rst), .done(done));

    // Clock generation
    initial clk = 0;
    always #5 clk = ~clk;  // 100 MHz clock

    // Reset + Stimulus
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_link_top);

        rst = 1;
        #15 rst = 0;

        wait(done);
        #20;
        $finish;
    end

endmodule
