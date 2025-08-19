# Problem 3: Vending Machine (Mealy FSM)

## Description
This project implements a vending machine controller using a **Mealy finite state machine (FSM)** in Verilog-2001.

- Item Price = 20
- Acceptable coins = 5 or 10
- When total ≥ 20:
  - `dispense = 1` for 1 clock cycle
  - Reset total to 0
- When total = 25:
  - `dispense = 1` and `chg5 = 1` for 1 clock cycle (product dispensed + 5 returned)
- Idle input = `coin=2'b00`
- Illegal input (`coin=2'b11`) ignored
- FSM states represent partial totals: **0, 5, 10, 15**

---

## Files
```
problem3_vending/
 ├── vending_mealy.v        # FSM RTL design
 ├── tb_vending_mealy.v     # Testbench
 ├── README.md              # Instructions
 └── waves/                 # Directory to store waveforms (dump.vcd)
```

---

## How to Run (Icarus Verilog + GTKWave)

### 1. Compile the design and testbench
```sh
iverilog -o sim.out tb_vending_mealy.v vending_mealy.v
```

### 2. Run the simulation
```sh
vvp sim.out
```
This will generate a waveform file at `waves/dump.vcd`.

### 3. Open the waveform
```sh
gtkwave waves/dump.vcd
```

---

## Expected Results
1. **Case 1 (10 + 10)** → `dispense` pulse asserted (no change).  
2. **Case 2 (5 + 5 + 10)** → `dispense` pulse asserted (no change).  
3. **Case 3 (10 + 10 + 5)** → both `dispense` and `chg5` pulses asserted.  

When viewed in GTKWave, you should see single-cycle pulses for `dispense` and `chg5` exactly when the total reaches **20** or **25**.
