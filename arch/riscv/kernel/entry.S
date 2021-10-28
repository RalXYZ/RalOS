.altmacro  # add this macro to make '.rept' macro works

.set REG_SIZE, 8
.set REG_NUM, 32

.macro SAVE_TO_SP_OFFSET reg, offset
    sd \reg, (\offset) * 8(sp)
.endm

.macro SAVE_TO_SP_OFFSET_N n
    SAVE_TO_SP_OFFSET x\n, (\n) - 1
.endm

.macro LOAD_FROM_SP_OFFSET reg, offset
    ld \reg, (\offset) * 8(sp)
.endm

.macro LOAD_FROM_SP_OFFSET_N n
    LOAD_FROM_SP_OFFSET x\n, (\n) - 1
.endm

.extern trap_handler
.section .text.entry
.align 2
.globl _traps

_traps:
# -----------

    # 1. save 32 registers and sepc to stack
    addi sp, sp, -1 * REG_SIZE * REG_NUM

    .set n, 1
    .rept 31
        SAVE_TO_SP_OFFSET_N %n
        .set n, n + 1
    .endr
    csrr a1, sepc  # a1 is the parameter of 'trap_handler' below! do not change this register!
    SAVE_TO_SP_OFFSET a1, 31

# -----------

    # 2. call trap_handler
    csrr a0, scause
    jal ra, trap_handler    

# -----------

    # 3. restore sepc and 32 registers (x2(sp) should be restore last) from stack
    LOAD_FROM_SP_OFFSET t0, 31
    csrw sepc, t0
    .set n, 1
    .rept 31
        LOAD_FROM_SP_OFFSET_N %n
        .set n, n + 1
    .endr
    
    addi sp, sp, REG_SIZE * REG_NUM

# -----------

    # 4. return from trap
    sret

# -----------