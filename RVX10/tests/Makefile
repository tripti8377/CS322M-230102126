//tests/Makefile

IVFLAGS = -g2012
TOP     = ../src/riscvsingle.sv
TB      = cpu_tb.sv
OUT     = cpu_tb
VCD     = wave.vcd

HEX     = rvx10.hex   # already present

.PHONY: all run waves clean

# Default target: build simulation binary
all: $(OUT)

# Compile RTL + testbench into simulation binary
$(OUT): $(TOP) $(TB) $(HEX)
	iverilog $(IVFLAGS) -o $(OUT) $(TOP) $(TB)

# Run simulation (uses existing rvx10.hex)
run: $(OUT) $(HEX)
	vvp $(OUT)

# Open waveform
waves: $(VCD)
	gtkwave $(VCD)

# Clean up build outputs
clean:
	del /Q $(OUT) $(VCD) 2>nul || rm -f $(OUT) $(VCD)
