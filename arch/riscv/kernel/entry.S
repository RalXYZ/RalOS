.extern trap_handler
    .section .text.entry
    .align 2
    .globl _traps
_traps:
# YOUR CODE HERE
# -----------

    # 1. save 32 registers and sepc to stack
    addi sp, sp, -0x100
    sd x1, 0xF8(sp)
    sd x2, 0xF0(sp)
    sd x3, 0xE8(sp)
    sd x4, 0xE0(sp)
    sd x5, 0xD8(sp)
    sd x6, 0xD0(sp)
    sd x7, 0xC8(sp)
    sd x8, 0xC0(sp)
    sd x9, 0xB8(sp)
    sd x10, 0xB0(sp)
    sd x11, 0xA8(sp)
    sd x12, 0xA0(sp)
    sd x13, 0x98(sp)
    sd x14, 0x90(sp)
    sd x15, 0x88(sp)
    sd x16, 0x80(sp)
    sd x17, 0x78(sp)
    sd x18, 0x70(sp)
    sd x19, 0x68(sp)
    sd x20, 0x60(sp)
    sd x21, 0x58(sp)
    sd x22, 0x50(sp)
    sd x23, 0x48(sp)
    sd x24, 0x40(sp)
    sd x25, 0x38(sp)
    sd x26, 0x30(sp)
    sd x27, 0x28(sp)
    sd x28, 0x20(sp)
    sd x29, 0x18(sp)
    sd x30, 0x10(sp)
    sd x31, 0x08(sp)
    csrr a1, sepc
    sd a1, 0(sp)

# -----------

    # 2. call trap_handler
    csrr a0, scause
    jal ra, trap_handler    

# -----------

    # 3. restore sepc and 32 registers (x2(sp) should be restore last) from stack
    ld t0, 0(sp)
    csrw sepc, t0
    ld x1, 0xF8(sp)
    ld x3, 0xE8(sp)
    ld x4, 0xE0(sp)
    ld x5, 0xD8(sp)  # x5 is t0
    ld x6, 0xD0(sp)
    ld x7, 0xC8(sp)
    ld x8, 0xC0(sp)
    ld x9, 0xB8(sp)
    ld x10, 0xB0(sp)
    ld x11, 0xA8(sp)
    ld x12, 0xA0(sp)
    ld x13, 0x98(sp)
    ld x14, 0x90(sp)
    ld x15, 0x88(sp)
    ld x16, 0x80(sp)
    ld x17, 0x78(sp)
    ld x18, 0x70(sp)
    ld x19, 0x68(sp)
    ld x20, 0x60(sp)
    ld x21, 0x58(sp)
    ld x22, 0x50(sp)
    ld x23, 0x48(sp)
    ld x24, 0x40(sp)
    ld x25, 0x38(sp)
    ld x26, 0x30(sp)
    ld x27, 0x28(sp)
    ld x28, 0x20(sp)
    ld x29, 0x18(sp)
    ld x30, 0x10(sp)
    ld x31, 0x08(sp)
    
    
    ld x2, 0xF0(sp)  # x2 is sp

    addi sp, sp, 0x100

# -----------

    # 4. return from trap
    sret

# -----------