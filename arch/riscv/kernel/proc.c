#include "proc.h"
#include "mm.h"
#include "rand.h"
#include "defs.h"
#include "printk.h"

extern void __dummy();
extern void __switch_to(struct task_struct* prev, struct task_struct* next);

struct task_struct* idle;           // idle process
struct task_struct* current;        // points to the current running thread
struct task_struct* task[NR_TASKS]; // thread array, all threads are stored here

void task_init() {
    // call kalloc() to allocate a physical page for idle
    idle = (struct task_struct *)kalloc();

    // set state to TASK_RUNNING
    idle->state = TASK_RUNNING;

    // since idle does not participate in scheduling, we can set its counter and priority to 0
    idle->counter = 0;
    idle->priority = 0;

    //  set the pid of idle to 0
    idle->pid = 0;

    // point current and task[0] to idle
    current = idle;
    task[0] = idle;

    /* ----------------------------------------------- */

    // initialize task[1] ~ task[NR_TASKS - 1]
    for (unsigned int i = 1; i < NR_TASKS; i++) {
        task[i] = (struct task_struct *)kalloc();
        task[i]->state = TASK_RUNNING;
        task[i]->counter = 0;
        task[i]->priority = rand();
        task[i]->pid = i;

        // set `ra` and `sp` in `thread_struct`
        // `ra` is set to be the address of __dummy
        // `sp` is set to the highest address of the applied physical page
        task[i]->thread.ra = (uint64)__dummy;
        task[i]->thread.sp = (((uint64)task) + PGSIZE);
    }

    printk("...proc_init done!\n");
}

void dummy() {
    uint64 MOD = 1000000007;
    uint64 auto_inc_local_var = 0;
    int last_counter = -1;
    while(1) {
        if (last_counter == -1 || current->counter != last_counter) {
            last_counter = current->counter;
            auto_inc_local_var = (auto_inc_local_var + 1) % MOD;
            printk("[PID = %d] is running. auto_inc_local_var = %d\n", current->pid, auto_inc_local_var); 
        }
    }
}

void switch_to(struct task_struct* next) {
    if (next != current) {
        struct task_struct* prev = current;
        current = next;
        __switch_to(prev, next);
    }
}
