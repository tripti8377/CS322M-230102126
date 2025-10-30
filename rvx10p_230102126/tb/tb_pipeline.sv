// tb_pipeline.sv

`timescale 1ns/1ps
module tb_pipeline();
  logic clk, reset;
  logic [31:0] WriteData, DataAdr;
  logic MemWrite;
  
  top_pipeline dut(.clk(clk), .reset(reset), .WriteData(WriteData), .DataAdr(DataAdr), .MemWrite(MemWrite));
  
initial begin
  $dumpfile("pipeline_tb.vcd");
  $dumpvars(0, tb_pipeline);
  reset = 1; #22; reset = 0;
end
  always begin 
    clk = 1; #5; clk = 0; #5; 
  end
  
  always @(negedge clk) begin
    if (MemWrite) $display("STORE @ %0d = 0x%08h (t=%0t)", DataAdr, WriteData, $time);
    if (MemWrite) begin
      if ((DataAdr === 100) && (WriteData === 25)) begin
        $display("Simulation succeeded");
        // Print checksum x28 if accessible
        $display("CHECKSUM (x28) = %0d (0x%08h)", dut.cpu.dp.RegFile[28], dut.cpu.dp.RegFile[28]);
        $finish;
      end else if (DataAdr !== 96) begin
        $display("Simulation failed");
        $finish;
      end
    end
  end
    
  /*
  // Debugging Purposes to see result of RVX10 operations
  // Read from the correct pipeline stages
  wire        wb_we   = dut.cpu.dp.MEMWB_RegWrite_local;  // WB stage register write enable
  wire [31:0] wb_val  = dut.cpu.dp.WB_value;              // Final writeback value
  wire  [4:0] wb_rd   = dut.cpu.dp.MEMWB_rd;              // WB stage destination register
  
  // For better debugging, also monitor the EX stage when instruction executes
  wire [6:0]  id_opcode = dut.cpu.dp.InstrD[6:0];         // ID stage opcode
  wire [6:0]  ex_opcode = dut.cpu.dp.IDEX_opcode;         // EX stage opcode
  wire [31:0] ex_alu_result = dut.cpu.dp.ALU_resultE;     // EX stage ALU result
  wire [4:0]  ex_rd = dut.cpu.dp.IDEX_Rd;                 // EX stage destination
  wire [4:0]  ex_rs1 = dut.cpu.dp.IDEX_Rs1;               // EX stage rs1
  wire [4:0]  ex_rs2 = dut.cpu.dp.IDEX_Rs2;               // EX stage rs2
  wire [31:0] ex_readdata1 = dut.cpu.dp.IDEX_ReadData1;   // EX stage ReadData1
  wire [31:0] ex_readdata2 = dut.cpu.dp.IDEX_ReadData2;   // EX stage ReadData2
  wire [31:0] ex_imm = dut.cpu.dp.IDEX_Imm;               // EX stage Immediate
  wire [31:0] ex_alu_a = dut.cpu.dp.ALU_input_A;          // ALU input A
  wire [31:0] ex_alu_b = dut.cpu.dp.ALU_input_B;          // ALU input B
  wire [31:0] ex_fwd_store = dut.cpu.dp.ForwardedStoreData; // Forwarded store data
  wire [31:0] exmem_aluout = dut.cpu.dp.EXMEM_aluOut;     // MEM stage ALU result
  wire [4:0]  exmem_rd = dut.cpu.dp.EXMEM_rd;             // MEM stage destination
  wire        exmem_regwrite = dut.cpu.dp.EXMEM_RegWrite_local; // MEM stage regwrite
  
  always @(negedge clk) begin
    // Display when RVX10 instruction completes in EX stage
    if (ex_opcode == 7'b0001011 && ex_rd != 5'd0) begin
      $display("RVX10 EX stage: ALU result = %0d (0x%08h) -> x%0d  t=%0t",
                ex_alu_result, ex_alu_result, ex_rd, $time);
    end
    
    // Debug store instructions in EX stage
    if (ex_opcode == 7'b0100011) begin
      $display("STORE EX stage: Rs2=x%0d, ReadData2=0x%08h, FwdData=0x%08h, EXMEM(x%0d)=0x%08h, RegWr=%0b  t=%0t",
                ex_rs2, ex_readdata2, ex_fwd_store, exmem_rd, exmem_aluout, exmem_regwrite, $time);
    end
    
    // Display when any RVX10 result is written back
    if (wb_we && (wb_rd != 5'd0)) begin
      // Check if this was originally an RVX10 instruction (harder to track, so show all writebacks)
      $display("WB stage: Writing %0d (0x%08h) to x%0d  t=%0t",
                wb_val, wb_val, wb_rd, $time);
    end
  end
  */
endmodule