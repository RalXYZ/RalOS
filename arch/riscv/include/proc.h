#ifndef _PROC_H
#define _PROC_H

#include "types.h"

#define NR_TASKS  (1 + 31) // control maximum thread amount （idle thread + 31 kernel threads）

#define TASK_RUNNING 0 // to simplify the lab， all threads has only one state

#define PRIORITY_MIN 1
#define PRIORITY_MAX 10

/* kernel and user mode stack pointer of the thread */
struct thread_info {
    uint64 kernel_sp;
    uint64 user_sp;
};

/* data structrue of thread state */
struct thread_struct {
    uint64 ra;
    uint64 sp;
    uint64 s[12];
};

/* data structure of a thread */
struct task_struct {
    struct thread_info* thread_info;
    uint64 state;    // thread state
    uint64 counter;  // remaining execution time 
    uint64 priority; // execution priority, 1 is the lowest, 10 is the highest
    uint64 pid;      // thread id

    struct thread_struct thread;
};

/* thread initialization, creates NR_TASKS threads */ 
void task_init(); 

/* called in timer interrupt handler to judge whether it is necessary to conduct schedule */
void do_timer();

/* schedule function, choose the next thread to be executed */
void schedule();

/* thread switch entrance function */
void switch_to(struct task_struct* next);

/* dummy function: a loop function, prints the thread's pid and a self-increasing local variable iteratively */
void dummy(); 

#endif
