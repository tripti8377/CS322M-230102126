# RISC-V Single-Cycle (RV32I) — Icarus Verilog Quick Start

This repo contains a **single-cycle RISC-V (RV32I) CPU** and test programs.  
Follow these steps to build and run the simulation with **Icarus Verilog**.

---

## Files

- `src/riscvsingle.sv` — SystemVerilog source with the **CPU + testbench**. The testbench top module is `testbench` and instantiates `top` (the CPU).
- `tests/riscvtest.s` — Assembly source for the baseline RV32I test.
- `tests/riscvtest.hex` — Instruction memory image loaded by `$readmemh` (one 32-bit hex word per line).
- `tests/rvx10.s` — Assembly test program for the **custom RVX10 instructions** (uses `.insn`).
- `tests/rvx10.hex` — Instruction memory image for the custom instructions, generated from `rvx10.s`.
- `docs/ENCODINGS.md` — Bitfield encodings for custom RVX10 ops.
- `docs/TESTPLAN.md` — Per-instruction inputs and expected results.

>**Success criterion:**  
> The testbench prints **“Simulation succeeded”** when the CPU stores the value **25 (0x19)** to memory address **100 (0x64)**.  
> For the RVX10 test, the program writes a **marker value (e.g., 1234)** to memory address **200 (0xC8)**.

---

## Requirements

- **Icarus Verilog** (iverilog / vvp)  
  - Ubuntu/Debian: `sudo apt-get install iverilog`  
  - macOS (Homebrew): `brew install icarus-verilog`  
  - Windows: install from MSYS2 or the official site.  
- (Optional) **GTKWave** for viewing waveforms:  
  `sudo apt-get install gtkwave` / `brew install gtkwave`

---

## Directory Layout

riscv_single/
├── src/
│ └── riscvsingle.sv
├── tests/
│ ├── riscvtest.s
│ ├── riscvtest.hex
│ ├── rvx10.s
│ └── rvx10.hex
└── docs/
├── ENCODINGS.md
└── TESTPLAN.md

> **Important:** The simulation reads `riscvtest.txt` using a **relative path**. Run the simulator **from the folder** that contains the file (or edit the path inside `riscvsingle.sv`).

---

## Build & Run (Terminal)

### Linux / macOS
```bash
cd /path/to/riscv_single

# Compile (enable SystemVerilog-2012 support)
iverilog -g2012 -o cpu_tb riscvsingle.sv

# Run
vvp cpu_tb
```

### Windows (PowerShell or CMD)
```bat
cd C:\path\to\riscv_single
iverilog -g2012 -o cpu_tb src\riscvsingle.sv
vvp cpu_tb
```

**Expected console output**
```
Simulation succeeded
```

---

## Makefile (optional)

You can also use the included `Makefile`:

```bash
make run        # build + run
make waves      # build + run + open wave.vcd in GTKWave
make clean      # remove generated files
```

If you prefer not to use Make, just run the iverilog/vvp commands shown above.

---

## Waveforms (Optional, with GTKWave)

The testbench is set up to dump `wave.vcd`. To open it:

```bash
# after running the simulation:
gtkwave wave.vcd
```

If you don’t see a VCD file, ensure the following block exists inside `module testbench;` in `riscvsingle.sv`:
```systemverilog
initial begin
  $dumpfile("wave.vcd");
  $dumpvars(0, testbench);
end
```

Rebuild and run again to regenerate the VCD.

---

# RVX10 Extension for RV32I Single-Cycle CPU
- This project extends the baseline RV32I CPU with 10 custom ALU instructions (ANDN, ORN, XNOR, MIN, MAX, MINU, MAXU, ROL, ROR, ABS).
- They are implemented under a custom opcode (0x0B = 0001011) with funct3/funct7 encodings described in docs/ENCODINGS.md.

## Build & Run Instructions

### 1. Assemble & Convert to Hex
``` bash
riscv32-unknown-elf-as -march=rv32i -o tests/rvx10.o tests/rvx10.s
riscv32-unknown-elf-objcopy -O verilog tests/rvx10.o tests/rvx10.hex
```
Place rvx10.hex in the tests/ folder.
In riscvsingle.sv, ensure the instruction memory loads it with:

```systemverilog
$readmemh("tests/rvx10.hex", imem);
```
# How to assemble and get machine codes?

```bash
#Assemble
riscv32-unknown-elf-as -march=rv32i -o rvx10.o rvx10.s

# Dump encodings
riscv32-unknown-elf-objdump -d rvx10.o
```

---

## License / Credits

This teaching setup is adapted for course use. Original single‑cycle RISC‑V example design is based on standard educational resources for RV32I.
=======
# Lab 1

**Course:** Digital Logic and Computer Architecture – CS322M  
**Name:** Tripti Rakesh Kumar  
**Roll No.:** 230102126

---

## Objective

This lab implements:
- **Q1**: A 1-bit comparator that outputs logic-1 on one of three ports depending on whether A > B, A == B, or A < B.
- **Q2**: A 4-bit equality comparator that outputs logic-1 only if both inputs are equal.

The implementation uses **structural modeling**, as taught in the lecture, by instantiating custom gate modules.

---

## File Structure

| File Name              | Description                            |
|------------------------|----------------------------------------|
| `Gate_And.v`          | AND gate module                         |
| `Gate_Not.v`          | NOT gate module                         |
| `Gate_Xor.v`          | XOR gate module                         |
| `Gate_Xnor.v`         | XNOR gate module                        |
| `Question_1.v`        | 1-bit comparator (Q1)                   |
| `Question_2.v`        | 4-bit equality comparator (Q2)          |
| `testbench_q1.v`      | Testbench for 1-bit comparator          |
| `testbench_q2.v`      | Testbench for 4-bit comparator          |
| `output_q1.txt`       | Simulation output for Q1                |
| `output_q2.txt`       | Simulation output for Q2                |

---

## How to Run

Make sure Icarus Verilog is installed and run the following commands from your project folder:

### For Q1 (1-bit Comparator)
```bash
iverilog -o Question_1.out Question_1.v testbench_q1.v Gate_And.v Gate_Not.v Gate_Xnor.v
vvp Question_1.out > output_q1.txt
type output_q1.txt
```

### For Q2 (4-bit Equality Comparator)
```bash
iverilog -o Question_2.out Question_2.v testbench_q2.v Gate_And.v Gate_Not.v Gate_Xnor.v
vvp Question_2.out > output_q2.txt
type output_q2.txt
```
