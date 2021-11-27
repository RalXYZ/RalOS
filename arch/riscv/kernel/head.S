#include "defs.h"

.extern start_kernel
.extern clock_set_next_event
.extern mm_init

.extern unsigned long TIME_CLOCK  # defined in clock.cpp

.section .text.init
.globl _start
_start:

    la sp, boot_stack_top

    call setup_vm
    call relocate

    # set stvec = _traps
    la a0, _traps
    csrw stvec, a0

    # sie: SEIE UEIE 0 0 STIE UTIE 0 0 SSIE USIE
    # set sie[STIE] = 1
    csrr a0, sie
    ori a0, a0, 0x20   # mask the sixth bit
    csrw sie, a0

    # set first time interrupt
    rdtime a0
    ld a1, TIME_CLOCK
    add a0, a0, a1
    li a1, 0
    li a2, 0
    li a3, 0
    li a4, 0
    li a5, 0
    li a6, 0
    li a7, 0
    ecall

    # sstatus: ... UPIE 0 0 SIE UIE
    # set sstatus[SIE] = 1
    csrsi sstatus, 0b10

    # initialize memory management system
    jal ra, mm_init

    # initialize threads
    jal ra, task_init

    j start_kernel



relocate:
    ld t0, PA2VA_OFFSET

    # set ra = ra + PA2VA_OFFSET
    add ra, ra, t0

    # set sp = sp + PA2VA_OFFSET
    add sp, sp, t0

    # set satp with early_pgtbl
    la t1, early_pgtbl
    srli t1, t1, 12
    li t2, 0x8000000000000000  # set mode to 8, which means page-based 39 bit virtual addressing
    or t1, t1, t2

    csrw satp, t1

    # flush tlb
    sfence.vma zero, zero

    ret



.section .bss.stack
.globl boot_stack
boot_stack:
.space 0x1000

.globl boot_stack_top
boot_stack_top:
