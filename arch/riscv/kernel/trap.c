#include "printk.h"
#include "clock.h"
#include "proc.h"

#define INTERRUPT_MASK 0x8000000000000000

extern struct task_struct* current;  // proc.cpp

void trap_handler(unsigned long scause) {  // there could exist a second parameter `sepc`
    // judge the type of trap by scause
    if (!(scause & INTERRUPT_MASK)) {
        return;
    }

    // judge whether it is a timer interrupt
    if ((scause & (~INTERRUPT_MASK)) != 5) {  // Supervisor timer interrupt
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
