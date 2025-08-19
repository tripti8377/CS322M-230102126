// problem1_seqdet/tb_seq_detect_mealy.v
// Testbench for Mealy sequence detector (pattern 1101 with overlap).
`timescale 1ns/1ps

module tb_seq_detect_mealy;
    reg  clk, rst, din;
    wire y;

    // DUT
    seq_detect_mealy dut(.clk(clk), .rst(rst), .din(din), .y(y));

    // Clock: 10 ns period (100 MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Test sequence
    integer i;
    integer n;          // <<-- moved here
    reg [0:99] stream;  // up to 100 bits if needed; index grows to the right

    task drive_stream(input integer nbits);
        begin
            for (i = 0; i < nbits; i = i + 1) begin
                din = stream[i];
                @(posedge clk);
                // Log cycle, input, and y pulse
                $display("t=%0t ns  cycle=%0d  din=%0d  y=%0d", $time, i, din, y);
            end
        end
    endtask

    initial begin
        // Wave dump
        $dumpfile("waves/dump.vcd");
        $dumpvars(0, tb_seq_detect_mealy);

        // Reset
        rst = 1;
        din = 0;
        repeat (2) @(posedge clk);
        rst = 0;

        // STREAM-1: demonstrates overlaps; bits: 1101101101
        // Expected pulse indices (0-based): 3, 6, 9
        stream[0] = 1;
        stream[1] = 1;
        stream[2] = 0;
        stream[3] = 1;  // pulse
        stream[4] = 1;
        stream[5] = 0;
        stream[6] = 1;  // pulse
        stream[7] = 1;
        stream[8] = 0;
        stream[9] = 1;  // pulse
        drive_stream(10);

        // Small reset between streams
        rst = 1; @(posedge clk); rst = 0;

        // STREAM-2: random bits with some hits
        // Sequence chosen: 1 0 1 1 0 1 1 0 0 1 1 0 1
        // Expected pulse indices (0-based): 6, 12
        n = 13;
        stream[0]  = 1;
        stream[1]  = 0;
        stream[2]  = 1;
        stream[3]  = 1;
        stream[4]  = 0;
        stream[5]  = 1;
        stream[6]  = 1;   // pulse
        stream[7]  = 0;
        stream[8]  = 0;
        stream[9]  = 1;
        stream[10] = 1;
        stream[11] = 0;
        stream[12] = 1;   // pulse
        drive_stream(n);

        // Finish
        @(posedge clk);
        $finish;
    end
endmodule
