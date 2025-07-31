# Lab 1

**Course:** Digital Logic and Computer Architecture – CS322M  
**Name:** Tripti Rakesh Kumar  
**Roll No.:** 230102126
**Instructor:** Satyajit Das  

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
