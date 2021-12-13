#include "syscall.hpp"

extern "C" {
    #include "printk.h"
    #include "clock.h"
    #include "proc.h"
}

extern "C" {
    extern struct task_struct *current;  // proc.cpp
    void trap_handler(uint64 cause, uint64 epc, struct pt_regs *regs);
}

constexpr auto INTERRUPT_MASK = 0x8000'0000'0000'0000ul;

struct pt_regs {
    uint64 x[32];
    uint64 sepc;
    // uint64 sstatus;
};

enum struct REGISTERS : uint64 {
    zero, ra, sp, gp, tp, t0, t1, t2, s0, s1, a0, a1, a2, a3, a4, a5, a6, a7,
    s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, t3, t4, t5, t6,
};

auto non_interrupt_handler(uint64 scause, pt_regs *regs) -> void {
    // judge whether an ecall from user mode happened
    if ((scause & (~INTERRUPT_MASK)) != 8) {
        return;
    }

    switch (regs->x[static_cast<uint64>(REGISTERS::a7)]) {
    case SYS_WRITE:
        regs->x[static_cast<uint64>(REGISTERS::a0)] = sys_write(
                static_cast<uint32_t>(regs->x[static_cast<uint64>(REGISTERS::a1)]),
                reinterpret_cast<const char *>(regs->x[static_cast<uint64>(REGISTERS::a2)]),
                regs->x[static_cast<uint64>(REGISTERS::a3)]
        );
        break;
    
    case SYS_GETPID:
        regs->x[static_cast<uint64>(REGISTERS::a0)] = sys_getpid();
        break;
    }

    regs->sepc += 4;

    return;

}

auto trap_handler(uint64 scause, uint64 sepc, pt_regs *regs) -> void {
    // judge the type of trap by scause
    if (!(scause & INTERRUPT_MASK)) {
       non_interrupt_handler(scause, regs);
        return;
    }

    sepc += 0; // just use this parameter, in case of -Werror

    // judge whether it is a timer interrupt
    if ((scause & (~INTERRUPT_MASK)) != 5) {
        return;
    }

    // output message, set the next timer interrupt by calling clock_set_next_event
    // printk("kernel is running!\n[S] Supervisor Mode Timer Interrupt\n");
    
    clock_set_next_event();

    current->counter--;
    do_timer();

    // other interrupts or exceptions can be ignored
    return;
}
