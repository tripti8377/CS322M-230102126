// hazard_unit.sv
// Detect load-use hazard (EX stage has a load whose Rd matches ID's Rs1/Rs2).
module hazard_unit(input  logic MemReadE,
                   input  logic [4:0] RdE,
                   input  logic [4:0] Rs1D, Rs2D,
                   output logic stallF, stallD, flushE);

  always_comb begin
    stallF = 0; stallD = 0; flushE = 0;
    if (MemReadE && ( (RdE == Rs1D) || (RdE == Rs2D) )) begin
      stallF = 1; stallD = 1; flushE = 1;
    end
  end
endmodule
