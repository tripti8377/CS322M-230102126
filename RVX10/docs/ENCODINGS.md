# Instruction Encodings

This document lists the encodings for both the standard RV32I ALU instructions and the custom RVX10 extension.  
Each instruction is shown with its `funct7`, `funct3` fields (from the ISA spec), and the corresponding 5-bit `ALUControl` code used internally by the ALU in `riscvsingle.sv`.


## Standard RV32I Instructions

 Instruction | Type | funct7  | funct3 | opcode  | ALUControl (internal) |
 ADD         | R    | 0000000 | 000    | 0110011 | 00000                 |
 SUB         | R    | 0100000 | 000    | 0110011 | 00001                 |
 SLT         | R    | 0000000 | 010    | 0110011 | 00010                 |
 OR          | R    | 0000000 | 110    | 0110011 | 00011                 |
 AND         | R    | 0000000 | 111    | 0110011 | 00100                 |

## RVX10 Custom Extension

 Ins | Semantics                                   | funct7  | funct3 | opcode  | ALUControl |
 ANDN| rd = rs1 & ~rs2                             | 0000000 | 000    | 0001011 | 01000      |
 ORN | rd = rs1 | ~rs2                             | 0000000 | 001    | 0001011 | 01001      |
 XNOR| rd = ~(rs1 ⊕ rs2)                          | 0000000 | 010    | 0001011 | 01010      |
 MIN | rd = (int32(rs1) < int32(rs2))? rs1:rs2     | 0000001 | 000    | 0001011 | 01011      |
 MAX | rd = (int32(rs1) > int32(rs2))? rs1:rs2     | 0000001 | 001    | 0001011 | 01100      |
 MINU| rd = (rs1 < rs2)? rs1:rs2 (unsigned)        | 0000001 | 010    | 0001011 | 01101      |
 MAXU| rd = (rs1 > rs2)? rs1:rs2 (unsigned)        | 0000001 | 011    | 0001011 | 01110      |
 ROL | rd = (rs1 ≪ s) |(rs1 ≫ (32−s)), s=rs2[4:0] | 0000010 | 000    | 0001011 | 01111      |
 ROR | rd = (rs1 ≫ s) |(rs1 ≪ (32−s)), s=rs2[4:0] | 0000010 | 001    | 0001011 | 10000      |
 ABS | rd = (int32(rs1) ≥ 0)? rs1 : −rs1           | 0000011 | 000    | 0001011 | 10001      |


## Notes
- All RVX10 instructions use the **custom opcode `0x0B` (0001011)**.  
- `ALUControl` codes are internal signals to select the ALU operation.  
- Undefined funct7/funct3 combinations map to `xxxxx` and are treated as illegal instructions.  

### Special Cases (Assignment Requirements)
- **ABS**: Encoded as R-type with `rs2 = x0`. Hardware ignores `rs2` when `funct7 = 0000011`.  
  - Overflow: `ABS(0x80000000) = 0x80000000` (two’s complement wrap, no trap).  
- **ROL / ROR**: Use `s = rs2[4:0]` as shift amount.  
  - If `s = 0`, result = `rs1` unchanged (avoids shift-by-32 undefined case).  
- **Register x0**: Writes to `x0` are ignored as per RISC-V spec (always reads as zero).  


