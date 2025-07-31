module comparator_1bit (
  input A,
  input B,
  output o1,  // A > B
  output o2,  // A == B
  output o3   // A < B
);
  wire notB, notA, A_grt_B, A_lst_B, A_eql_B;

  // A > B: A & ~B
  Not invB(.A(B), .Z(notB));
  And gt(.A(A), .B(notB), .Z(o1));

  // A == B: ~(A ^ B)
  Xnor eq(.A(A), .B(B), .Z(o2));

  // A < B: ~A & B
  Not invA(.A(A), .Z(notA));
  And lt(.A(notA), .B(B), .Z(o3));

endmodule