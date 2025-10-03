//tests/cpu_tb.sv
`timescale 1ns/1ps

module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;

  // instantiate device to be tested
  top dut(
      .clk(clk), 
      .reset(reset), 
      .WriteData(WriteData), 
      .DataAdr(DataAdr), 
      .MemWrite(MemWrite)
  );
  
  // initialize test
  initial begin
      $display("Starting simulation...");
      reset <= 1; 
      #22; 
      reset <= 0;
      $display("Released reset at time %0t", $time);
  end

  // generate clock
  initial clk = 0;
  always #5 clk = ~clk;  // 10 ns period

  // monitor memory writes and CPU state
  always @(negedge clk) begin
      $display("PC=%0d, Instr=%h, ALUResult=%0d, WriteData=%0d, MemWrite=%b, DataAdr=%0d",
               dut.rvsingle.PC, dut.rvsingle.Instr, dut.rvsingle.ALUResult,
               WriteData, MemWrite, DataAdr);

      if (MemWrite) begin
          if (DataAdr === 100 && WriteData === 25) begin
              $display("Simulation succeeded at time %0t", $time);
              $stop;
          end else begin
              $display("Memory write to wrong address or value at time %0t", $time);
          end
      end
  end

  // optional timeout to prevent infinite run
  initial begin
      #500 $display("Simulation timeout, stopping."); 
      $stop;
  end

endmodule
