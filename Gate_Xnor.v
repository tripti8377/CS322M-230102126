module Xnor(input A, B, output Z);
  assign Z = ~(A ^ B);
endmodule