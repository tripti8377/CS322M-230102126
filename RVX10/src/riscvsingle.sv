// riscvsingle.sv

// RISC-V single-cycle processor
// From Section 7.6 of Digital Design & Computer Architecture
// 27 April 2020
// David_Harris@hmc.edu 
// Sarah.Harris@unlv.edu

// run 210
// Expect simulator to print "Simulation succeeded"
// when the value 25 (0x19) is written to address 100 (0x64)

// Single-cycle implementation of RISC-V (RV32I)
// User-level Instruction Set Architecture V2.2 (May 7, 2017)
// Implements a subset of the base integer instructions:
//    lw, sw
//    add, sub, and, or, slt, 
//    addi, andi, ori, slti
//    beq
//    jal
// Exceptions, traps, and interrupts not implemented
// little-endian memory

// 31 32-bit registers x1-x31, x0 hardwired to 0
// R-Type instructions
//   add, sub, and, or, slt
//   INSTR rd, rs1, rs2
//   Instr[31:25] = funct7 (funct7b5 & opb5 = 1 for sub, 0 for others)
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// I-Type Instructions
//   lw, I-type ALU (addi, andi, ori, slti)
//   lw:         INSTR rd, imm(rs1)
//   I-type ALU: INSTR rd, rs1, imm (12-bit signed)
//   Instr[31:20] = imm[11:0]
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// S-Type Instruction
//   sw rs2, imm(rs1) (store rs2 into address specified by rs1 + immm)
//   Instr[31:25] = imm[11:5] (offset[11:5])
//   Instr[24:20] = rs2 (src)
//   Instr[19:15] = rs1 (base)
//   Instr[14:12] = funct3
//   Instr[11:7]  = imm[4:0]  (offset[4:0])
//   Instr[6:0]   = opcode
// B-Type Instruction
//   beq rs1, rs2, imm (PCTarget = PC + (signed imm x 2))
//   Instr[31:25] = imm[12], imm[10:5]
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = imm[4:1], imm[11]
//   Instr[6:0]   = opcode
// J-Type Instruction
//   jal rd, imm  (signed imm is multiplied by 2 and added to PC, rd = PC+4)
//   Instr[31:12] = imm[20], imm[10:1], imm[11], imm[19:12]
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode

//   Instruction  opcode    funct3    funct7
//   add          0110011   000       0000000
//   sub          0110011   000       0100000
//   and          0110011   111       0000000
//   or           0110011   110       0000000
//   slt          0110011   010       0000000
//   addi         0010011   000       immediate
//   andi         0010011   111       immediate
//   ori          0010011   110       immediate
//   slti         0010011   010       immediate
//   beq          1100011   000       immediate
//   lw	          0000011   010       immediate
//   sw           0100011   010       immediate
//   jal          1101111   immediate immediate

// riscvsingle.sv


module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);

  logic [31:0] PC, Instr, ReadData;
  
  // instantiate processor and memories
  riscvsingle rvsingle(clk, reset, PC, Instr, MemWrite, DataAdr, 
                       WriteData, ReadData);
  imem imem(PC, Instr);
  dmem dmem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

module riscvsingle(input  logic        clk, reset,
                   output logic [31:0] PC,
                   input  logic [31:0] Instr,
                   output logic        MemWrite,
                   output logic [31:0] ALUResult, WriteData,
                   input  logic [31:0] ReadData);

  logic       ALUSrc, RegWrite, Jump, Zero;
  logic [1:0] ResultSrc, ImmSrc;
  logic [4:0] ALUControl; // <-- widened to 5 bits

  controller c(Instr[6:0], Instr[14:12], Instr[30], Instr[31:25], Zero,
               ResultSrc, MemWrite, PCSrc,
               ALUSrc, RegWrite, Jump,
               ImmSrc, ALUControl);
  datapath dp(clk, reset, ResultSrc, PCSrc,
              ALUSrc, RegWrite,
              ImmSrc, ALUControl,
              Zero, PC, Instr,
              ALUResult, WriteData, ReadData);
endmodule

module controller(input  logic [6:0] op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic [6:0] funct7,    // added full funct7 field
                  input  logic       Zero,
                  output logic [1:0] ResultSrc,
                  output logic       MemWrite,
                  output logic       PCSrc, ALUSrc,
                  output logic       RegWrite, Jump,
                  output logic [1:0] ImmSrc,
                  output logic [4:0] ALUControl); // widened to 5 bits

  logic [1:0] ALUOp;
  logic       Branch;

  maindec md(op, ResultSrc, MemWrite, Branch,
             ALUSrc, RegWrite, Jump, ImmSrc, ALUOp);
  aludec  ad(op[5], funct3, funct7b5, ALUOp, funct7, ALUControl);

  assign PCSrc = Branch & Zero | Jump;
endmodule

module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrc,
               output logic       MemWrite,
               output logic       Branch, ALUSrc,
               output logic       RegWrite, Jump,
               output logic [1:0] ImmSrc,
               output logic [1:0] ALUOp);

  logic [10:0] controls;

  assign {RegWrite, ImmSrc, ALUSrc, MemWrite,
          ResultSrc, Branch, ALUOp, Jump} = controls;

  always_comb
    case(op)
    // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
      7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
      7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw
      7'b0110011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type 
      7'b0001011: controls = 11'b1_xx_0_0_00_0_10_0; // RVX10 custom R-type
      7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
      7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
      default:    controls = 11'bx_xx_x_x_xx_x_xx_x; // non-implemented instruction
    endcase
endmodule

module aludec(
    input  logic       opb5,
    input  logic [2:0] funct3,
    input  logic       funct7b5, 
    input  logic [1:0] ALUOp,
    input  logic [6:0] funct7,      // needed for RVX10 decode
    output logic [4:0] ALUControl
);

  logic RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb begin
    case (ALUOp)
      2'b00: ALUControl = 5'b00000; // add for lw/sw
      2'b01: ALUControl = 5'b00001; // subtract for beq
      default: begin
        case (funct3)
          // --- Standard RV32I ---
          3'b000: ALUControl = (RtypeSub ? 5'b00001 : 5'b00000); // sub : add
          3'b010: ALUControl = 5'b00010; // slt
          3'b110: ALUControl = 5'b00011; // or
          3'b111: ALUControl = 5'b00100; // and

          // --- RVX10 custom ops ---
          default: begin
            case (funct7)
              7'b0000000: case (funct3)
                3'b000: ALUControl = 5'b01000; // ANDN
                3'b001: ALUControl = 5'b01001; // ORN
                3'b010: ALUControl = 5'b01010; // XNOR
                default: ALUControl = 5'bxxxxx;
              endcase

              7'b0000001: case (funct3)
                3'b000: ALUControl = 5'b01011; // MIN
                3'b001: ALUControl = 5'b01100; // MAX
                3'b010: ALUControl = 5'b01101; // MINU
                3'b011: ALUControl = 5'b01110; // MAXU
                default: ALUControl = 5'bxxxxx;
              endcase

              7'b0000010: case (funct3)
                3'b000: ALUControl = 5'b01111; // ROL
                3'b001: ALUControl = 5'b10000; // ROR
                default: ALUControl = 5'bxxxxx;
              endcase

              7'b0000011: case (funct3)
                3'b000: ALUControl = 5'b10001; // ABS
                default: ALUControl = 5'bxxxxx;
              endcase

              default: ALUControl = 5'bxxxxx;
            endcase
          end
        endcase
      end
    endcase
  end
endmodule


module datapath(input  logic        clk, reset,
                input  logic [1:0]  ResultSrc, 
                input  logic        PCSrc, ALUSrc,
                input  logic        RegWrite,
                input  logic [1:0]  ImmSrc,
                input  logic [4:0]  ALUControl, // widened to 5 bits
                output logic        Zero,
                output logic [31:0] PC,
                input  logic [31:0] Instr,
                output logic [31:0] ALUResult, WriteData,
                input  logic [31:0] ReadData);

  logic [31:0] PCNext, PCPlus4, PCTarget;
  logic [31:0] ImmExt;
  logic [31:0] SrcA, SrcB;
  logic [31:0] Result;

  // next PC logic
  flopr #(32) pcreg(clk, reset, PCNext, PC); 
  adder       pcadd4(PC, 32'd4, PCPlus4);
  adder       pcaddbranch(PC, ImmExt, PCTarget);
  mux2 #(32)  pcmux(PCPlus4, PCTarget, PCSrc, PCNext);
 
  // register file logic
  regfile     rf(clk, RegWrite, Instr[19:15], Instr[24:20], 
                 Instr[11:7], Result, SrcA, WriteData);
  extend      ext(Instr[31:7], ImmSrc, ImmExt);

  // ALU logic
  mux2 #(32)  srcbmux(WriteData, ImmExt, ALUSrc, SrcB);
  alu         alu(SrcA, SrcB, ALUControl, ALUResult, Zero);
  mux3 #(32)  resultmux(ALUResult, ReadData, PCPlus4, ResultSrc, Result);
endmodule

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3 && a3 != 5'b00000) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module extend(
    input  logic [31:7] instr,
    input  logic [1:0]  immsrc,
    output logic [31:0] immext
);

  // Precompute fields
  wire [11:0] immI = instr[31:20];                   // I-type
  wire [11:0] immS = {instr[31:25], instr[11:7]};    // S-type
  wire [12:0] immB = {instr[31], instr[7], instr[30:25], instr[11:8], 1'b0}; // B-type
  wire [20:0] immJ = {instr[31], instr[19:12], instr[20], instr[30:21], 1'b0}; // J-type

  // Precompute sign-extended versions
  wire [31:0] extI = {{20{immI[11]}}, immI};
  wire [31:0] extS = {{20{immS[11]}}, immS};
  wire [31:0] extB = {{19{immB[12]}}, immB};
  wire [31:0] extJ = {{11{immJ[20]}}, immJ};

  // Select result
  always_comb begin
    case(immsrc)
      2'b00: immext = extI;
      2'b01: immext = extS;
      2'b10: immext = extB;
      2'b11: immext = extJ;
      default: immext = 32'bx;
    endcase
  end
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk or posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("rvx10.hex", RAM);

  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module dmem(input  logic        clk, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

module alu(
    input  logic [31:0] a, b,
    input  logic [4:0]  alucontrol,
    output logic [31:0] result,
    output logic        zero
);

  // basic add/sub
  wire [31:0] condinvb = alucontrol[0] ? ~b : b;
  wire [31:0] sum = a + condinvb + alucontrol[0];

  // overflow for slt
  wire v = (a[31] & condinvb[31] & ~sum[31]) | (~a[31] & ~condinvb[31] & sum[31]);

  // shift amount
  wire [4:0] shamt = b[4:0];

  // precompute all ALU results as wires
  wire [31:0] res_add   = sum;
  wire [31:0] res_sub   = sum;
  wire [31:0] res_slt   = (sum[31] ^ v) ? 32'd1 : 32'd0;
  wire [31:0] res_or    = a | b;
  wire [31:0] res_and   = a & b;
  wire [31:0] res_andn  = a & ~b;
  wire [31:0] res_orn   = a | ~b;
  wire [31:0] res_xnor  = ~(a ^ b);
  wire [31:0] res_min   = ($signed(a) < $signed(b)) ? a : b;
  wire [31:0] res_max   = ($signed(a) > $signed(b)) ? a : b;
  wire [31:0] res_minu  = (a < b) ? a : b;
  wire [31:0] res_maxu  = (a > b) ? a : b;
  wire [31:0] res_rol   = (shamt == 0) ? a : ((a << shamt) | (a >> (32 - shamt)));
  wire [31:0] res_ror   = (shamt == 0) ? a : ((a >> shamt) | (a << (32 - shamt)));
  wire [31:0] res_abs   = (a[31]) ? -a : a;

  // now use a simple always_comb to select among wires
  always_comb begin
    case (alucontrol)
      5'b00000: result = res_add;
      5'b00001: result = res_sub;
      5'b00010: result = res_slt;
      5'b00011: result = res_or;
      5'b00100: result = res_and;

      5'b01000: result = res_andn;
      5'b01001: result = res_orn;
      5'b01010: result = res_xnor;
      5'b01011: result = res_min;
      5'b01100: result = res_max;
      5'b01101: result = res_minu;
      5'b01110: result = res_maxu;
      5'b01111: result = res_rol;
      5'b10000: result = res_ror;
      5'b10001: result = res_abs;

      default: result = 32'bx;
    endcase

    zero = (result == 32'd0);
  end

endmodule
