# Problem 4 – Master–Slave Handshake FSMs

This project implements two FSMs (Master and Slave) with a **4-phase req/ack handshake** and 8-bit data bus.

---

## 📌 Protocol
1. Master drives data, raises `req`.
2. Slave latches data on `req`, asserts `ack` (held 2 cycles).
3. Master sees `ack`, drops `req`. Slave then drops `ack`.
4. Repeats for 4 bytes. Master asserts `done` for 1 cycle at the end.

---

## 📂 Files
- `master_fsm.v` → Master FSM (drives 4-byte burst A0..A3).
- `slave_fsm.v` → Slave FSM (latches data, holds `ack` for 2 cycles).
- `link_top.v` → Connects Master and Slave.
- `tb_link_top.v` → Testbench with clock/reset, dumps waveforms.
- `README.md` → Instructions.

---

## ▶️ Run Instructions

### Icarus Verilog + GTKWave
```sh
iverilog -o sim.out tb_link_top.v link_top.v master_fsm.v slave_fsm.v
vvp sim.out
gtkwave dump.vcd
```


---

## 📊 Expected Output
- Waveform showing **4 handshakes** (req/ack/data).
- Master asserts `done` after 4 transfers.
- Slave `last_byte` shows the last received byte.

---
