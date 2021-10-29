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

.macro SAVE_SX n, base
    sd s\n, \n * 8(\base)
.endm

.macro LOAD_SX n, base
    ld s\n, \n * 8(\base)
.endm

.extern trap_handler
.extern dummy
.section .text.entry
.align 2

# ------------------------------------------------------------------------

.globl _traps

_traps:
    # save 32 registers and sepc to stack
    addi sp, sp, -1 * REG_SIZE * REG_NUM

    .set n, 1
    .rept 31
        SAVE_TO_SP_OFFSET_N %n
        .set n, n + 1
    .endr
    csrr a1, sepc  # a1 is the parameter of 'trap_handler' below! do not change this register!
    SAVE_TO_SP_OFFSET a1, 31

    # call trap_handler
    csrr a0, scause
    jal ra, trap_handler    

    # restore sepc and 32 registers (x2(sp) should be restore last) from stack
    LOAD_FROM_SP_OFFSET t0, 31
    csrw sepc, t0
    .set n, 1
    .rept 31
        LOAD_FROM_SP_OFFSET_N %n
        .set n, n + 1
    .endr
    
    addi sp, sp, REG_SIZE * REG_NUM

    # return from trap
    sret

# ------------------------------------------------------------------------

.global __dummy
__dummy:
    la t0, dummy
    csrw sepc, t0
    sret

# ------------------------------------------------------------------------

.globl __switch_to
__switch_to:
    # save state to prev process
    addi t0, a0, 5 * 8
    sd ra, 0(t0)
    sd sp, 8(t0)

    addi t0, t0, 2 * 8
    .set n, 0
    .rept 12
        SAVE_SX %n, t0
        .set n, n + 1
    .endr

    # restore state from next process
    addi t0, a1, 5 * 8
    ld ra, 0(t0)
    ld sp, 8(t0)

    addi t0, t0, 2 * 8
    .set n, 0
    .rept 12
        LOAD_SX %n, t0
        .set n, n + 1
    .endr

    ret
