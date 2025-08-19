# Problem 1 — Serial Sequence Detector (Mealy, overlapping)

**Goal:** Detect serial bit pattern `1101` on `din` **with overlap**. Output `y` is a **1‑cycle pulse** in the same clock when the **last bit arrives**.

**Type:** Mealy &nbsp;&nbsp; **Reset:** synchronous, active‑high

---

## Deliverables

- `seq_detect_mealy.v` — FSM (Mealy) with proper fallback edges (supports overlap)
- `tb_seq_detect_mealy.v` — testbench that drives example streams, logs pulses, and dumps VCD
- `state_diagram.v` - handrawn state diagram for the fsm
- `waves/` — directory where VCD/PNG waveforms will be produced (create PNG via GTKWave)

---

## State machine (text)

States encode the longest matched **prefix** so far:

- `S0` → none matched
- `S1` → `1`
- `S11` → `11`
- `S110` → `110`

Transitions (inputs on the arrows):

- `S0 --1--> S1`, `S0 --0--> S0`
- `S1 --1--> S11`, `S1 --0--> S0`
- `S11 --0--> S110`, `S11 --1--> S11` (fallback keeps the `11` suffix)
- `S110 --1/pulse--> S1` (found `1101`, assert `y=1` **this** cycle, fallback to `S1`)
- `S110 --0--> S0`

This is equivalent to a KMP-style automaton for the pattern `1101`.

---

## How to run

### Icarus Verilog + GTKWave
```bash
cd problem1_seqdet
iverilog -o sim.out tb_seq_detect_mealy.v seq_detect_mealy.v
vvp sim.out
gtkwave waves/dump.vcd
```


The testbench already calls `$dumpfile("waves/dump.vcd")` and `$dumpvars`.

---

## Streams in the testbench and expected pulse indices

All indices below are **0‑based cycle counts** measured at the rising edge where `din` is sampled. Since this is a **Mealy** detector, the pulse occurs **on the same cycle** the last `1` of `1101` is applied.

1) **Stream‑1:** `1101101101`  
   **Expected y=1 at cycles:** `3, 6, 9` (three overlapping detections).


You’ll also see these cycles printed in the console via `$display` when you run the sim.

---

## Notes
ign
- Reset is **synchronous, active‑high**: the FSM returns to `S0` on the next clock when `rst=1`.
- Overlap is supported by the fallback from `S110 --1--> S1` (because the suffix `1` of `1101` is also a prefix), so back‑to‑back patterns are detected without missing bits.
- `y` is purely **Mealy** (combinational) and **one‑cycle** wide.
