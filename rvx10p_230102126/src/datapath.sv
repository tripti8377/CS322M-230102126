// datapath.sv
`timescale 1ns/1ps
// Full five-stage pipeline datapath with forwarding, hazard detection, and branch control.
// Implements all data hazards (forwarding, load-use) and control hazards (branches/jumps).

module datapath(input logic clk, reset,
                output logic [31:0] PC,
                input  logic [31:0] InstrIF,
                output logic MemWrite_out,
                output logic [31:0] DataAdr_out, WriteData_out,
                input  logic [31:0] ReadData);

  // Performance counters
  logic [31:0] cycle_count, instr_retired, stall_count, flush_count, branch_count;
  
  always_ff @(posedge clk, posedge reset) begin
    if (reset) begin
      cycle_count <= 32'd0;
      instr_retired <= 32'd0;
      stall_count <= 32'd0;
      flush_count <= 32'd0;
      branch_count <= 32'd0;
    end else begin
      cycle_count <= cycle_count + 32'd1;
      if (stallF || stallD) stall_count <= stall_count + 32'd1;
      if (flushE) flush_count <= flush_count + 32'd1;
    end
  end

  // --------------------------
  // IF stage
  // --------------------------
  logic [31:0] PC_reg, PC_next, PC_plus4;
  assign PC = PC_reg;
  assign PC_plus4 = PC_reg + 32'd4;

  // pipeline IF/ID regs
  logic [31:0] IFID_PC, IFID_Instr;

  // control from ID stage (for stall) and EX stage (for branch)
  logic stallF, stallD, flushE, flushD;
  
  // Branch/Jump control signals
  logic PCSrc;  // 1 = take branch/jump, 0 = PC+4
  logic [31:0] PCTarget;
  
  // PC selection
  assign PC_next = PCSrc ? PCTarget : PC_plus4;
  
  always_ff @(posedge clk, posedge reset) begin
    if (reset) begin
      PC_reg <= 32'd0;
    end else if (!stallF) begin
      PC_reg <= PC_next;
    end
  end

  // IF/ID pipeline regs
  always_ff @(posedge clk, posedge reset) begin
    if (reset) begin
      IFID_PC    <= 32'd0;
      IFID_Instr <= 32'h00000013; // NOP: addi x0, x0, 0
    end else begin
      if (flushD) begin
        // Flush on branch/jump taken
        IFID_PC    <= 32'd0;
        IFID_Instr <= 32'h00000013; // NOP
      end else if (!stallD) begin
        IFID_PC    <= PC_reg;
        IFID_Instr <= InstrIF;
      end
      // Note: stallD keeps the same instruction in IF/ID
    end
  end

  // --------------------------
  // ID stage
  // --------------------------
  logic [31:0] RegFile [0:31];
  integer i;
  initial for (i=0;i<32;i=i+1) RegFile[i]=32'd0;

  logic [4:0] Rs1D, Rs2D, RdD;
  logic [31:0] ReadData1D, ReadData2D;
  logic [31:0] InstrD;
  assign InstrD = IFID_Instr;
  assign Rs1D = InstrD[19:15];
  assign Rs2D = InstrD[24:20];
  assign RdD  = InstrD[11:7];

  // read RF
  assign ReadData1D = (Rs1D != 5'd0) ? RegFile[Rs1D] : 32'd0;
  assign ReadData2D = (Rs2D != 5'd0) ? RegFile[Rs2D] : 32'd0;

  // decode control
  logic RegWriteD, MemWriteD, MemToRegD, ALUSrcD, BranchD, JumpD;
  logic [1:0] ALUOpD, ImmSrcD, ResultSrcD;
  // instantiate controller (combinational)
  controller ctrl(.opcode(InstrD[6:0]),
                  .RegWrite(RegWriteD), .MemWrite(MemWriteD),
                  .MemToReg(MemToRegD), .ALUSrc(ALUSrcD),
                  .ALUOp(ALUOpD), .ImmSrc(ImmSrcD), .ResultSrc(ResultSrcD),
                  .Branch(BranchD), .Jump(JumpD));

  // immediate value (avoid part-select in procedural)
  logic [11:0] immI;
  logic [11:0] immS;
  logic [12:0] immB;
  logic [20:0] immJ;
  assign immI = InstrD[31:20];
  assign immS = {InstrD[31:25], InstrD[11:7]};
  assign immB = {InstrD[31], InstrD[7], InstrD[30:25], InstrD[11:8], 1'b0};
  assign immJ = {InstrD[31], InstrD[19:12], InstrD[20], InstrD[30:21], 1'b0};

  logic [31:0] ImmExtD;
  always_comb begin
    case (ImmSrcD)
      2'b00: ImmExtD = {{20{immI[11]}}, immI};
      2'b01: ImmExtD = {{20{immS[11]}}, immS};
      2'b10: ImmExtD = {{19{immB[12]}}, immB};
      2'b11: ImmExtD = {{11{immJ[20]}}, immJ};
      default: ImmExtD = 32'd0;
    endcase
  end

  // Capture fields to ID/EX regs
  logic [31:0] IDEX_ReadData1, IDEX_ReadData2, IDEX_Imm, IDEX_PC;
  logic [4:0]  IDEX_Rs1, IDEX_Rs2, IDEX_Rd;
  logic IDEX_RegWrite, IDEX_MemWrite, IDEX_MemToReg, IDEX_ALUSrc, IDEX_Branch, IDEX_Jump;
  logic [1:0] IDEX_ALUOp, IDEX_ResultSrc;
  logic [2:0] IDEX_funct3;
  logic [6:0] IDEX_funct7, IDEX_opcode;

  always_ff @(posedge clk, posedge reset) begin
    if (reset) begin
      IDEX_ReadData1 <= 32'd0; IDEX_ReadData2 <= 32'd0; IDEX_Imm <= 32'd0; IDEX_PC <= 32'd0;
      IDEX_Rs1 <= 5'd0; IDEX_Rs2 <= 5'd0; IDEX_Rd <= 5'd0;
      IDEX_RegWrite <= 1'b0; IDEX_MemWrite <= 1'b0; IDEX_MemToReg <= 1'b0;
      IDEX_ALUSrc <= 1'b0; IDEX_Branch <= 1'b0; IDEX_Jump <= 1'b0;
      IDEX_ALUOp <= 2'd0; IDEX_ResultSrc <= 2'd0;
      IDEX_funct3 <= 3'd0; IDEX_funct7 <= 7'd0; IDEX_opcode <= 7'd0;
    end else begin
      // CRITICAL FIX: Handle flushE first, then stallD
      if (flushE) begin
        // Insert bubble/NOP in EX stage
        IDEX_ReadData1 <= 32'd0; IDEX_ReadData2 <= 32'd0; IDEX_Imm <= 32'd0; IDEX_PC <= 32'd0;
        IDEX_Rs1 <= 5'd0; IDEX_Rs2 <= 5'd0; IDEX_Rd <= 5'd0;
        IDEX_RegWrite <= 1'b0; IDEX_MemWrite <= 1'b0; IDEX_MemToReg <= 1'b0;
        IDEX_ALUSrc <= 1'b0; IDEX_Branch <= 1'b0; IDEX_Jump <= 1'b0;
        IDEX_ALUOp <= 2'd0; IDEX_ResultSrc <= 2'd0;
        IDEX_funct3 <= 3'd0; IDEX_funct7 <= 7'd0; IDEX_opcode <= 7'b0010011; // NOP-like
      end else if (!stallD) begin
        // Normal pipeline advancement
        IDEX_ReadData1 <= ReadData1D;
        IDEX_ReadData2 <= ReadData2D;
        IDEX_Imm <= ImmExtD;
        IDEX_PC <= IFID_PC;
        IDEX_Rs1 <= Rs1D; IDEX_Rs2 <= Rs2D; IDEX_Rd <= RdD;
        IDEX_RegWrite <= RegWriteD; IDEX_MemWrite <= MemWriteD; 
        IDEX_MemToReg <= MemToRegD; IDEX_ALUSrc <= ALUSrcD;
        IDEX_Branch <= BranchD; IDEX_Jump <= JumpD;
        IDEX_ALUOp <= ALUOpD; IDEX_ResultSrc <= ResultSrcD;
        IDEX_funct3 <= InstrD[14:12];
        IDEX_funct7 <= InstrD[31:25];
        IDEX_opcode <= InstrD[6:0];
      end
      // if stallD is high and flushE is low, keep current IDEX values (stall)
    end
  end

  // --------------------------
  // EX stage
  // --------------------------
  // ALU control (including CUSTOM-0)
  // ALUControl encodings
  localparam [4:0] ALU_ADD  = 5'b00000;
  localparam [4:0] ALU_SUB  = 5'b00001;
  localparam [4:0] ALU_AND  = 5'b00010;
  localparam [4:0] ALU_OR   = 5'b00011;
  localparam [4:0] ALU_XOR  = 5'b00100;
  localparam [4:0] ALU_SLT  = 5'b00101;
  localparam [4:0] ALU_SLL  = 5'b00110;
  localparam [4:0] ALU_SRL  = 5'b00111;
  localparam [4:0] ALU_ANDN = 5'b01000;
  localparam [4:0] ALU_ORN  = 5'b01001;
  localparam [4:0] ALU_XNOR = 5'b01010;
  localparam [4:0] ALU_MIN  = 5'b01011;
  localparam [4:0] ALU_MAX  = 5'b01100;
  localparam [4:0] ALU_MINU = 5'b01101;
  localparam [4:0] ALU_MAXU = 5'b01110;
  localparam [4:0] ALU_ROL  = 5'b01111;
  localparam [4:0] ALU_ROR  = 5'b10000;
  localparam [4:0] ALU_ABS  = 5'b10001;

  function automatic [4:0] aluctrl(input logic [1:0] ALUOp, input logic [2:0] f3, input logic [6:0] f7, input logic [6:0] opcode);
    aluctrl = ALU_ADD;
    if (opcode == 7'b0001011) begin
      // CUSTOM-0
      unique case ({f7,f3})
        {7'b0000000,3'b000}: aluctrl = ALU_ANDN;
        {7'b0000000,3'b001}: aluctrl = ALU_ORN;
        {7'b0000000,3'b010}: aluctrl = ALU_XNOR;
        {7'b0000001,3'b000}: aluctrl = ALU_MIN;
        {7'b0000001,3'b001}: aluctrl = ALU_MAX;
        {7'b0000001,3'b010}: aluctrl = ALU_MINU;
        {7'b0000001,3'b011}: aluctrl = ALU_MAXU;
        {7'b0000010,3'b000}: aluctrl = ALU_ROL;
        {7'b0000010,3'b001}: aluctrl = ALU_ROR;
        {7'b0000011,3'b000}: aluctrl = ALU_ABS;
        default: aluctrl = ALU_ADD;
      endcase
    end else begin
      if (ALUOp == 2'b00) aluctrl = ALU_ADD;
      else if (ALUOp == 2'b01) aluctrl = ALU_SUB;
      else begin
        unique case (f3)
          3'b000: aluctrl = (f7[5]) ? ALU_SUB : ALU_ADD;
          3'b010: aluctrl = ALU_SLT;
          3'b110: aluctrl = ALU_OR;
          3'b111: aluctrl = ALU_AND;
          default: aluctrl = ALU_ADD;
        endcase
      end
    end
  endfunction

  // Declare EX/MEM and MEM/WB pipeline registers early for use in forwarding
  logic [31:0] EXMEM_aluOut, EXMEM_writeData;
  logic [4:0] EXMEM_rd;
  logic EXMEM_RegWrite_local, EXMEM_MemWrite_local, EXMEM_MemToReg_local;

  logic [31:0] MEMWB_aluOut, MEMWB_readData;
  logic [4:0] MEMWB_rd;
  logic MEMWB_RegWrite_local, MEMWB_MemToReg_local;

  // Forwarding unit inputs/outputs
  logic [1:0] ForwardA, ForwardB;
  logic [4:0] EX_Rs1, EX_Rs2;
  assign EX_Rs1 = IDEX_Rs1; assign EX_Rs2 = IDEX_Rs2;

  // Wires for forwarding unit interface
  logic [4:0] EXMEM_Rd, MEMWB_Rd;
  logic EXMEM_RegWrite, MEMWB_RegWrite;
  logic [31:0] EXMEM_ALUOut, MEMWB_Result;

  assign EXMEM_ALUOut = EXMEM_aluOut;
  assign EXMEM_Rd = EXMEM_rd;
  assign EXMEM_RegWrite = EXMEM_RegWrite_local;
  assign MEMWB_Rd = MEMWB_rd;
  assign MEMWB_RegWrite = MEMWB_RegWrite_local;

  // ALU inputs before forwarding
  logic [31:0] ALU_input_A, ALU_input_B;
  logic [31:0] ALU_srcA, ALU_srcB;

  assign ALU_srcA = IDEX_ReadData1;
  assign ALU_srcB = (IDEX_ALUSrc) ? IDEX_Imm : IDEX_ReadData2;

  // apply forwarding for A and B
  always_comb begin
    ALU_input_A = ALU_srcA;
    ALU_input_B = ALU_srcB;
    // Forward to A (only if Rs1 is not x0)
    if (IDEX_Rs1 != 5'd0) begin
      if (ForwardA == 2'b01) ALU_input_A = EXMEM_ALUOut;
      else if (ForwardA == 2'b10) ALU_input_A = MEMWB_Result;
    end
    // Forward to B (only if Rs2 is not x0, and not using immediate)
    if (IDEX_Rs2 != 5'd0 && !IDEX_ALUSrc) begin
      if (ForwardB == 2'b01) ALU_input_B = EXMEM_ALUOut;
      else if (ForwardB == 2'b10) ALU_input_B = MEMWB_Result;
    end
  end

  // ALU core
  function automatic [31:0] alu_core(input logic [31:0] a, input logic [31:0] b, input logic [4:0] ctrl);
    logic [31:0] add_res, sub_res;
    add_res = a + b; sub_res = a - b;
    case (ctrl)
      ALU_ADD:  alu_core = add_res;
      ALU_SUB:  alu_core = sub_res;
      ALU_AND:  alu_core = a & b;
      ALU_OR:   alu_core = a | b;
      ALU_XOR:  alu_core = a ^ b;
      ALU_SLT:  alu_core = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
      ALU_SLL:  alu_core = a << b[4:0];
      ALU_SRL:  alu_core = a >> b[4:0];
      ALU_ANDN: alu_core = a & ~b;
      ALU_ORN:  alu_core = a | ~b;
      ALU_XNOR: alu_core = ~(a ^ b);
      ALU_MIN:  alu_core = ($signed(a) < $signed(b)) ? a : b;
      ALU_MAX:  alu_core = ($signed(a) > $signed(b)) ? a : b;
      ALU_MINU: alu_core = (a < b) ? a : b;
      ALU_MAXU: alu_core = (a > b) ? a : b;
      ALU_ROL:  alu_core = (b[4:0] == 5'd0) ? a
                    : ((a << b[4:0]) | (a >> (6'd32 - b[4:0])));
      ALU_ROR:  alu_core = (b[4:0] == 5'd0) ? a
                    : ((a >> b[4:0]) | (a << (6'd32 - b[4:0])));
      ALU_ABS:  alu_core = ($signed(a) >= 0) ? a : (32'b0 - a);
      default:  alu_core = 32'd0;
    endcase
  endfunction

  // Determine ALUControl
  logic [4:0] ALUControlE;
  always_comb begin
    ALUControlE = aluctrl(IDEX_ALUOp, IDEX_funct3, IDEX_funct7, IDEX_opcode);
  end

  // ALU result
  logic [31:0] ALU_resultE;
  assign ALU_resultE = alu_core(ALU_input_A, ALU_input_B, ALUControlE);
  logic ZeroE; assign ZeroE = (ALU_resultE == 32'd0);
  
  // Branch/Jump logic in EX stage
  logic BranchTaken;
  always_comb begin
    BranchTaken = 1'b0;
    if (IDEX_Branch) begin
      case (IDEX_funct3)
        3'b000: BranchTaken = ZeroE;              // beq: branch if equal (zero)
        3'b001: BranchTaken = ~ZeroE;             // bne: branch if not equal
        3'b100: BranchTaken = ALU_resultE[0];     // blt: branch if less than (signed)
        3'b101: BranchTaken = ~ALU_resultE[0];    // bge: branch if greater/equal (signed)
        3'b110: BranchTaken = ALU_resultE[0];     // bltu: branch if less than (unsigned)
        3'b111: BranchTaken = ~ALU_resultE[0];    // bgeu: branch if greater/equal (unsigned)
        default: BranchTaken = 1'b0;
      endcase
    end
  end
  
  // PC target calculation (branch or jump)
  assign PCTarget = IDEX_PC + IDEX_Imm;
  assign PCSrc = (BranchTaken && IDEX_Branch) || IDEX_Jump;
  
  // Flush IF/ID when branch/jump taken
  assign flushD = PCSrc;

  // Store data forwarding (for sw instructions)
  logic [31:0] ForwardedStoreData;
  always_comb begin
    // For stores, Rs2 is the data source
    ForwardedStoreData = IDEX_ReadData2;
    
    // Only forward if Rs2 is not x0
    if (IDEX_Rs2 != 5'd0) begin
      // Forward from MEM stage
      if (EXMEM_RegWrite_local && (EXMEM_rd != 5'd0) && (EXMEM_rd == IDEX_Rs2)) 
        ForwardedStoreData = EXMEM_aluOut;
      // Forward from WB stage (lower priority)
      else if (MEMWB_RegWrite_local && (MEMWB_rd != 5'd0) && (MEMWB_rd == IDEX_Rs2))
        ForwardedStoreData = MEMWB_Result;
    end
  end

  // EX/MEM pipeline register updates
  always_ff @(posedge clk, posedge reset) begin
    if (reset) begin
      EXMEM_aluOut <= 32'd0; EXMEM_writeData <= 32'd0;
      EXMEM_rd <= 5'd0; EXMEM_RegWrite_local <= 1'b0; 
      EXMEM_MemWrite_local <= 1'b0; EXMEM_MemToReg_local <= 1'b0;
    end else begin
      EXMEM_aluOut <= ALU_resultE;
      EXMEM_writeData <= ForwardedStoreData;  // Use forwarded data
      EXMEM_rd <= IDEX_Rd;
      EXMEM_RegWrite_local <= IDEX_RegWrite;
      EXMEM_MemWrite_local <= IDEX_MemWrite;
      EXMEM_MemToReg_local <= IDEX_MemToReg;
    end
  end

  // MEM/WB pipeline register updates
  always_ff @(posedge clk, posedge reset) begin
    if (reset) begin
      MEMWB_aluOut <= 32'd0; MEMWB_readData <= 32'd0; MEMWB_rd <= 5'd0;
      MEMWB_RegWrite_local <= 1'b0; MEMWB_MemToReg_local <= 1'b0;
    end else begin
      MEMWB_aluOut <= EXMEM_aluOut;
      MEMWB_readData <= ReadData;
      MEMWB_rd <= EXMEM_rd;
      MEMWB_RegWrite_local <= EXMEM_RegWrite_local;
      MEMWB_MemToReg_local <= EXMEM_MemToReg_local;
    end
  end

  // Forwarding unit instantiation
  forwarding_unit fwd(.Rs1E(EX_Rs1), .Rs2E(EX_Rs2),
                      .RdM(EXMEM_Rd), .RdW(MEMWB_Rd),
                      .RegWriteM(EXMEM_RegWrite), .RegWriteW(MEMWB_RegWrite),
                      .ForwardA(ForwardA), .ForwardB(ForwardB));

  // --------------------------
  // MEM stage
  // --------------------------
  assign MemWrite_out = EXMEM_MemWrite_local;
  assign DataAdr_out = EXMEM_aluOut;
  assign WriteData_out = EXMEM_writeData;

  // --------------------------
  // WB stage
  // --------------------------
  logic [31:0] WB_value;
  assign WB_value = (MEMWB_MemToReg_local) ? MEMWB_readData : MEMWB_aluOut;
  assign MEMWB_Result = WB_value;

  // Write to register file
  always_ff @(posedge clk) begin
    if (MEMWB_RegWrite_local && (MEMWB_rd != 5'd0)) begin
      RegFile[MEMWB_rd] <= WB_value;
      instr_retired <= instr_retired + 32'd1;  // Count retired instructions
    end
  end
  
  // Verification checks
  always @(posedge clk) begin
    if (!reset) begin
      // Check x0 is always zero
      if (RegFile[0] !== 32'd0) begin
        $display("ERROR: x0 = 0x%08h (should be 0) at t=%0t", RegFile[0], $time);
      end
      
      // Check for back-to-back ALU forwarding (when EX writes and next EX reads same register)
      if (EXMEM_RegWrite_local && EXMEM_rd != 5'd0 && 
          (EXMEM_rd == IDEX_Rs1 || EXMEM_rd == IDEX_Rs2) &&
          (IDEX_opcode == 7'b0110011 || IDEX_opcode == 7'b0001011)) begin
        $display("FORWARDING: EX-to-EX detected for x%0d at t=%0t", EXMEM_rd, $time);
      end
      
      // Check load-use hazard stall
      if (stallF && stallD && flushE) begin
        $display("LOAD-USE STALL: Inserted bubble at t=%0t (RdE=x%0d, Rs1D=x%0d, Rs2D=x%0d)", 
                 $time, IDEX_Rd, Rs1D, Rs2D);
      end
      
      // Check branch taken
      if (PCSrc && IDEX_Branch) begin
        branch_count <= branch_count + 32'd1;
        $display("BRANCH TAKEN: PC=%0d → PC=%0d (target=%0d) at t=%0t", 
                 IDEX_PC, PCTarget, PCTarget, $time);
        $display("  Flushing IF/ID stage");
      end
      
      // Check jump
      if (PCSrc && IDEX_Jump) begin
        $display("JUMP: PC=%0d → PC=%0d (target=%0d) at t=%0t", 
                 IDEX_PC, PCTarget, PCTarget, $time);
        $display("  Flushing IF/ID stage");
      end
    end
  end

  // --------------------------
  // Hazard detection instantiation
  // --------------------------
  logic MemReadE;
  assign MemReadE = (IDEX_opcode == 7'b0000011) ? 1'b1 : 1'b0;

  hazard_unit hunit(.MemReadE(MemReadE), .RdE(IDEX_Rd),
                    .Rs1D(Rs1D), .Rs2D(Rs2D),
                    .stallF(stallF), .stallD(stallD), .flushE(flushE));
  
  // Performance summary at end of simulation
  final begin
    $display("\n========== PIPELINE PERFORMANCE SUMMARY ==========");
    $display("Total cycles:        %0d", cycle_count);
    $display("Instructions retired: %0d", instr_retired);
    $display("Stall cycles:        %0d", stall_count);
    $display("Flush cycles:        %0d", flush_count);
    $display("Branches taken:      %0d", branch_count);
    if (instr_retired > 0) begin
      $display("Average CPI:         %.2f", real'(cycle_count) / real'(instr_retired));
      $display("Pipeline efficiency: %.1f%%", 100.0 * real'(instr_retired) / real'(cycle_count));
    end
    $display("==================================================\n");
  end

endmodule