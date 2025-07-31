module comparator_4bit (
  input [3:0] A,
  input [3:0] B,
  output equal
);
  wire [3:0] x;

  // Use Xnor for each bit
  Xnor x0 (.A(A[0]), .B(B[0]), .Z(x[0]));
  Xnor x1 (.A(A[1]), .B(B[1]), .Z(x[1]));
  Xnor x2 (.A(A[2]), .B(B[2]), .Z(x[2]));
  Xnor x3 (.A(A[3]), .B(B[3]), .Z(x[3]));

  // Final: x[0] & x[1] & x[2] & x[3]
  And a01 (.A(x[0]), .B(x[1]), .Z(w01));
  And a23 (.A(x[2]), .B(x[3]), .Z(w23));
  And final (.A(w01), .B(w23), .Z(equal));

endmodule