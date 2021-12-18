#include "defs.h"

.altmacro  # add this macro to make '.rept' macro works

.set REG_SIZE, 8
.set REG_NUM, 33

.macro SAVE_TO_SP_OFFSET reg, offset
    sd \reg, (\offset) * 8(sp)
.endm

.macro SAVE_TO_SP_OFFSET_N n
    SAVE_TO_SP_OFFSET x\n, (\n)
.endm

.macro LOAD_FROM_SP_OFFSET reg, offset
    ld \reg, (\offset) * 8(sp)
.endm

.macro LOAD_FROM_SP_OFFSET_N n
    LOAD_FROM_SP_OFFSET x\n, (\n)
.endm

.macro SAVE_SX n, base
    sd s\n, \n * 8(\base)
.endm

.macro LOAD_SX n, base
    ld s\n, \n * 8(\base)
.endm

.extern current  # points to the current thread, declared in proc.cpp
.extern trap_handler
.extern dummy
.section .text.entry
.align 2

# ------------------------------------------------------------------------

# swap sscratch and sp
__switch_sscratch_sp:
    sd t0, -8(sp)
    sd t1, -16(sp)

    mv t1, sp

    # swap
    csrr t0, sscratch
    csrw sscratch, sp
    mv sp, t0

    ld t0, -8(t0)
    ld t1, -16(t1)

    ret

# ------------------------------------------------------------------------

.globl _traps
_traps:
    # switch to supervisor mode stack
    call __switch_sscratch_sp

    # when `sp` is zero, it means we are in kernel mode, 
    # so we should not switch between user and supervisor stack
    bnez sp, _trap_continue
    call __switch_sscratch_sp
    
_trap_continue:
    # save 32 registers and sepc to stack
    addi sp, sp, -1 * REG_SIZE * REG_NUM

    .set n, 0
    .rept 32
        SAVE_TO_SP_OFFSET_N %n
        .set n, n + 1
    .endr
    csrr a1, sepc  # a1 could be the parameter of 'trap_handler' below! do not change this register!
    SAVE_TO_SP_OFFSET a1, 32

    # `regs` param
    mv a2, sp

    # call trap_handler
    csrr a0, scause
    jal ra, trap_handler

    # restore sepc and 32 registers (x2(sp) should be restore last) from stack
    LOAD_FROM_SP_OFFSET t0, 32
    csrw sepc, t0
    .set n, 0
    .rept 32
        LOAD_FROM_SP_OFFSET_N %n
        .set n, n + 1
    .endr
    
    addi sp, sp, REG_SIZE * REG_NUM

    # switch to user mode stack
    call __switch_sscratch_sp

    # return from trap
    sret

# ------------------------------------------------------------------------

.global __dummy
__dummy:
    # la t0, dummy
    # csrw sepc, t0

    call __switch_sscratch_sp

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

    addi t0, t0, 12 * 8  # points to `thread_struct.sepc`
    csrr t1, sepc
    csrr t2, sstatus
    csrr t3, sscratch
    sd t1, 0(t0)
    sd t2, 8(t0)
    sd t3, 16(t0)

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

    addi t0, t0, 12 * 8  # points to `thread_struct.sepc`
    ld t1, 0(t0)
    ld t2, 8(t0)
    ld t3, 16(t0)
    csrw sepc, t1
    csrw sstatus, t2
    csrw sscratch, t3

    ld t4, 24(t0)
    srli t4, t4, 12
    li t5, RISCV_SV39_MODE_MASK
    or t4, t4, t5
    csrw satp, t4
    sfence.vma zero, zero

    ret
