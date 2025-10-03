# RVX10 Test Plan

This document describes the verification strategy for the **RVX10** custom instruction set extension.  
The goal is to confirm correctness of all 10 new single-cycle instructions (`ANDN, ORN, XNOR, MIN, MAX, MINU, MAXU, ROL, ROR, ABS`) in the single-cycle RV32I core.

---

## 1. Test Strategy

- **Deterministic Tests**: Each instruction is tested with known register values. The expected result is accumulated into a checksum register (x28).  
- **End Condition**: The final checksum is stored to memory address `100`. A correct implementation writes the value `25`, causing the testbench to print *“Simulation succeeded”*.  
- **Coverage**:  
  - Signed vs unsigned behavior (MIN/MAX vs MINU/MAXU).  
  - Corner cases for shift/rotate (shamt = 0, 1, 31).  
  - ABS negative inputs, including `INT_MIN` overflow case.  
  - Register x0 behavior (writes ignored, reads always zero).  
- **Self-Checking**: No manual inspection needed; test passes automatically if memory[100] = 25.  

---

## 2. Instruction Test Cases

For each instruction, we provide representative test vectors (`rs1`, `rs2`) and the expected `rd` result.

### 2.1 ANDN (rd = rs1 & ~rs2)
| rs1         | rs2         | Expected Result |
|-------------|-------------|-----------------|
| 0xF0F0_A5A5 | 0x0F0F_FFFF | 0xF0F0_0000     |
| 0xFFFF_FFFF | 0x0000_0000 | 0xFFFF_FFFF     |
| 0x1234_5678 | 0xFFFF_FFFF | 0x0000_0000     |

---

### 2.2 ORN (rd = rs1 \| ~rs2)
| rs1         | rs2         | Expected Result |
|-------------|-------------|-----------------|
| 0x0000_0000 | 0x0000_0000 | 0xFFFF_FFFF     |
| 0x1234_5678 | 0xFFFF_0000 | 0xFFFF_5678     |

---

### 2.3 XNOR (rd = ~(rs1 ⊕ rs2))
| rs1         | rs2         | Expected Result |
|-------------|-------------|-----------------|
| 0xFFFF_FFFF | 0xFFFF_FFFF | 0xFFFF_FFFF     |
| 0x0000_0000 | 0xFFFF_FFFF | 0x0000_0000     |
| 0x1234_5678 | 0x1234_5678 | 0xFFFF_FFFF     |

---

### 2.4 MIN (signed)
| rs1         | rs2         | Expected Result |
|-------------|-------------|-----------------|
| 0x0000_0005 | 0xFFFF_FFF0 (-16) | 0xFFFF_FFF0 |
| 0x8000_0000 (-2^31) | 0x7FFF_FFFF | 0x8000_0000 |

---

### 2.5 MAX (signed)
| rs1         | rs2         | Expected Result |
|-------------|-------------|-----------------|
| 0x0000_0005 | 0xFFFF_FFF0 (-16) | 0x0000_0005 |
| 0x8000_0000 | 0x7FFF_FFFF | 0x7FFF_FFFF |

---

### 2.6 MINU (unsigned)
| rs1         | rs2         | Expected Result |
|-------------|-------------|-----------------|
| 0xFFFF_FFFE | 0x0000_0001 | 0x0000_0001 |
| 0x0000_0005 | 0x0000_0009 | 0x0000_0005 |

---

### 2.7 MAXU (unsigned)
| rs1         | rs2         | Expected Result |
|-------------|-------------|-----------------|
| 0xFFFF_FFFE | 0x0000_0001 | 0xFFFF_FFFE |
| 0x0000_0005 | 0x0000_0009 | 0x0000_0009 |

---

### 2.8 ROL (rotate left)
| rs1         | shamt | Expected Result |
|-------------|-------|-----------------|
| 0x8000_0001 | 3     | 0x0000_000B |
| 0x1234_5678 | 0     | 0x1234_5678 |
| 0x1234_5678 | 31    | (manual compute) |

---

### 2.9 ROR (rotate right)
| rs1         | shamt | Expected Result |
|-------------|-------|-----------------|
| 0x8000_0001 | 1     | 0xC000_0000 |
| 0x1234_5678 | 0     | 0x1234_5678 |

---

### 2.10 ABS
| rs1         | rs2 (ignored) | Expected Result |
|-------------|---------------|-----------------|
| 0xFFFF_FF80 (-128) | 0 | 0x0000_0080 |
| 0x8000_0000 (INT_MIN) | 0 | 0x8000_0000 |
| 0x0000_1234 | 0 | 0x0000_1234 |

---

## 3. Final Acceptance Test

- Program executes all ops, accumulates results into checksum (x28).  
- Store x28 to memory[100].  
- Testbench prints *“Simulation succeeded”* if value `25` is observed.

---
