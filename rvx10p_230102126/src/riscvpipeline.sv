// src/riscvpipeline.sv
// Top-level wrapper for RVX10-P (5-stage pipeline)
`timescale 1ns/1ps

// top-pipeline
module top_pipeline(input logic clk, reset,
                    output logic [31:0] WriteData, DataAdr,
                    output logic MemWrite);

  logic [31:0] PC;
  logic [31:0] InstrIF;
  logic [31:0] ReadData;

  // IMEM: provides InstrIF
  imem imem(.a(PC), .rd(InstrIF));

  // Instantiate pipeline CPU
  riscvpipeline cpu(.clk(clk), .reset(reset),
                    .PC(PC),
                    .InstrIF(InstrIF),
                    .MemWrite_out(MemWrite),
                    .DataAdr_out(DataAdr),
                    .WriteData_out(WriteData),
                    .ReadData(ReadData));

  // DMEM (simple behavioral memory)
  dmem dmem(.clk(clk), .reset(reset),
            .we(MemWrite), .a(DataAdr), .wd(WriteData),
            .rd(ReadData));
endmodule

// imem.sv
module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[0:63];

  initial begin
    $readmemh("tests/rvx10_pipeline.hex", RAM);
  end

  // word-aligned
  assign rd = RAM[a >> 2];
endmodule


// dmem.sv
module dmem(input logic clk, reset, we,
            input logic [31:0] a, wd,
            output logic [31:0] rd);

  logic [31:0] RAM[0:63];

  // initialize to zero
  integer i;
  initial for (i=0; i<64; i=i+1) RAM[i] = 32'd0;

  assign rd = RAM[a >> 2];

  always_ff @(posedge clk) begin
  if (we && !reset) begin
    RAM[a >> 2] <= wd;
    //$display("DMEM WRITE @ %0d = 0x%08h (word-addr=%0d) (t=%0t)", a, wd, a >> 2, $time);
  end
end
endmodule

// riscvpipeline.sv
module riscvpipeline(input logic clk, reset,
                     output logic [31:0] PC,
                     input  logic [31:0] InstrIF,
                     output logic MemWrite_out,
                     output logic [31:0] DataAdr_out, WriteData_out,
                     input  logic [31:0] ReadData);

  // Instantiate controller (combinational decode in ID)
  // Controller signals are embedded in datapath; we keep modular pieces for clarity
  datapath dp(.clk(clk), .reset(reset),
              .PC(PC),
              .InstrIF(InstrIF),
              .MemWrite_out(MemWrite_out),
              .DataAdr_out(DataAdr_out),
              .WriteData_out(WriteData_out),
              .ReadData(ReadData));
endmodule
