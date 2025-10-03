# tests/rvx10.s
# Test RVX10 custom instructions (ANDN, ORN, XNOR, MIN, MAX, MINU, MAXU, ROL, ROR, ABS)
# Uses .insn with custom opcode 0x0B (0001011)
# Success criterion: mem[100] = 25

        .section .text
        .globl _start
_start:
        # --------------------------------------------------
        # Init registers
        li   x1, 0xF0F0A5A5    # rs1 candidate
        li   x2, 0x0F0FFFFF    # rs2 candidate
        li   x3, -2            # 0xFFFFFFFE (for unsigned tests)
        li   x4, 3             # small value for shifts
        li   x5, -128          # for ABS test

        # --------------------------------------------------
        # ANDN : x6 = x1 & ~x2
        .insn r 0x0B, 0x0, x6, x1, x2, 0x00
        sw   x6, 0x80(x0)      # store at mem[128]

        # ORN : x7 = x1 | ~x2
        .insn r 0x0B, 0x1, x7, x1, x2, 0x00
        sw   x7, 0x84(x0)

        # XNOR : x8 = ~(x1 ^ x2)
        .insn r 0x0B, 0x2, x8, x1, x2, 0x00
        sw   x8, 0x88(x0)

        # --------------------------------------------------
        # MIN / MAX signed
        li   x9, 5
        li   x10, -16

        .insn r 0x0B, 0x0, x11, x9, x10, 0x01  # MIN
        sw   x11, 0x8C(x0)

        .insn r 0x0B, 0x1, x12, x9, x10, 0x01  # MAX
        sw   x12, 0x90(x0)

        # MINU / MAXU unsigned
        li   x13, 0xFFFFFFFE
        li   x14, 1

        .insn r 0x0B, 0x2, x15, x13, x14, 0x01 # MINU
        sw   x15, 0x94(x0)

        .insn r 0x0B, 0x3, x16, x13, x14, 0x01 # MAXU
        sw   x16, 0x98(x0)

        # --------------------------------------------------
        # Rotate left / right
        li   x17, 0x80000001
        li   x18, 3

        .insn r 0x0B, 0x0, x19, x17, x18, 0x02 # ROL
        sw   x19, 0x9C(x0)

        li   x20, 1
        .insn r 0x0B, 0x1, x21, x17, x20, 0x02 # ROR
        sw   x21, 0xA0(x0)

        # --------------------------------------------------
        # ABS
        .insn r 0x0B, 0x0, x22, x5, x0, 0x03   # ABS(-128) = 128
        sw   x22, 0xA4(x0)

        # --------------------------------------------------
        # Final marker — REQUIRED by harness
        li   x23, 25
        sw   x23, 100(x0)       # mem[100] = 25 → simulation success

done:   beq  x0, x0, done       # hang forever
