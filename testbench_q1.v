`timescale 1ns / 1ps

module tb_question_1;

  reg A, B;
  wire o1, o2, o3;

  
  comparator_1bit uut (.A(A), .B(B), .o1(o1), .o2(o2), .o3(o3));

  initial begin
    $display("A B | o1(A>B) o2(A==B) o3(A<B)");
    $display("-----------------------------");

    A = 0; B = 0; #10;
    $display("%b %b |    %b       %b       %b", A, B, o1, o2, o3);

    A = 0; B = 1; #10;
    $display("%b %b |    %b       %b       %b", A, B, o1, o2, o3);

    A = 1; B = 0; #10;
    $display("%b %b |    %b       %b       %b", A, B, o1, o2, o3);

    A = 1; B = 1; #10;
    $display("%b %b |    %b       %b       %b", A, B, o1, o2, o3);

    $finish;
  end

endmodule

