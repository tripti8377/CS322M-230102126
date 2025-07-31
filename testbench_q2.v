`timescale 1ns / 1ps

module tb_question_2;

  reg [3:0] A, B;
  wire equal;

  
  comparator_4bit uut (
    .A(A),
    .B(B),
    .equal(equal)
  );

  initial begin
    $display("  A     B   | Equal");
    $display("---------------");

    A = 4'b0000; B = 4'b0000; #10;
    $display("%b %b |   %b", A, B, equal);

    A = 4'b1010; B = 4'b1010; #10;
    $display("%b %b |   %b", A, B, equal);

    A = 4'b1111; B = 4'b0000; #10;
    $display("%b %b |   %b", A, B, equal);

    A = 4'b1001; B = 4'b1000; #10;
    $display("%b %b |   %b", A, B, equal);

    A = 4'b1100; B = 4'b1100; #10;
    $display("%b %b |   %b", A, B, equal);

    A = 4'b0110; B = 4'b1110; #10;
    $display("%b %b |   %b", A, B, equal);

    $finish;
  end

endmodule

