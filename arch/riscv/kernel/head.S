.extern start_kernel
.extern clock_set_next_event
.extern unsigned long TIME_CLOCK  # defined in clock.cpp

.section .text.init
.globl _start
_start:
# YOUR CODE HERE

    la sp, boot_stack_top

# ------------------

    # set stvec = _traps
    la a0, _traps
    csrw stvec, a0

# ------------------

    # sie: SEIE UEIE 0 0 STIE UTIE 0 0 SSIE USIE
    # set sie[STIE] = 1
    csrr a0, sie
    ori a0, a0, 0x20   # mask the sixth bit
    csrw sie, a0

# ------------------

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

# ------------------

    # sstatus: ... UPIE 0 0 SIE UIE
    # set sstatus[SIE] = 1
    csrsi sstatus, 0b10

# ------------------

# ------------------
# - your lab1 code -
# ------------------
    j start_kernel

    .section .bss.stack
    .globl boot_stack
boot_stack:
    .space 0x1000

    .globl boot_stack_top
boot_stack_top:
