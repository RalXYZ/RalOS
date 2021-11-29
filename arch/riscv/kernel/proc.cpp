extern "C" {
    #include "defs.h"
    #include "mm.h"
    #include "rand.h"
    #include "printk.h"
    #include "proc.h"
    #include "log.h"
}

extern "C" {
    extern void __dummy();
    extern void __switch_to(struct task_struct* prev, struct task_struct* next);
    struct task_struct* idle;           // idle process
    struct task_struct* current;        // points to the current running thread
    struct task_struct* task[NR_TASKS]; // thread array, all threads are stored here
}

int last_counter = -1;  // this variable is previously declared in dummy()

auto task_init() -> void {
    // call kalloc() to allocate a physical page for idle
    idle = reinterpret_cast<task_struct *>(kalloc());

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
    for (size_t i = 1; i < NR_TASKS; i++) {
        task[i] = reinterpret_cast<task_struct *>(kalloc());
        task[i]->state = TASK_RUNNING;
        task[i]->counter = 0;
        task[i]->priority = rand();
        task[i]->pid = i;

        // set `ra` and `sp` in `thread_struct`
        // `ra` is set to be the address of __dummy
        // `sp` is set to the highest address of the applied physical page
        task[i]->thread.ra = reinterpret_cast<uint64>(__dummy);
        task[i]->thread.sp = reinterpret_cast<uint64>(task[i]) + PGSIZE;
    }

    log_ok(const_cast<char*>("Process initialization succeeded"));
}

auto dummy() -> void {
    const uint64 MOD = 1'000'000'007;
    uint64 auto_inc_local_var = 0;
    while (true) {
        if (current->counter != static_cast<uint64>(::last_counter) or ::last_counter == -1) {
            ::last_counter = current->counter;
            auto_inc_local_var = (auto_inc_local_var + 1) % MOD;
            printk(GREEN "[PID = %d]" NC "is running, auto_inc_local_var = %d, thread space begin at 0xffffffe0%x\n", current->pid, auto_inc_local_var, current);
        }
    }
}

auto switch_to(task_struct* next) -> void {
    if (next != current) {
        auto prev = current;
        current = next;
        ::last_counter = -1;
        printk(YELLOW "\nswitch to [PID = %d COUNTER = %d PRIORITY = %d]\n" NC, next->pid, next->counter, next->priority);
        __switch_to(prev, next);
    }
}

auto do_timer() -> void {
    // if current thread is the idle thread or the remaining execution time is 0, execute a schedule
    if (current->pid == 0 or current->counter == 0) {
        schedule();
    }
}

#ifdef SJF
auto schedule() -> void {
    auto has_min = false;
    decltype(current->pid) min_remaining_time = -1;  // max of decltype(current->pid)
    size_t min_index = 0;
    for (size_t i = 1; i < NR_TASKS; i++) {
        if (task[i]->state != TASK_RUNNING) {
            continue;
        }
        if (task[i]->counter != 0 and task[i]->counter < min_remaining_time) {
            has_min = true;
            min_remaining_time = task[i]->counter;
            min_index = i;
        }
    }
    if (has_min) {
        switch_to(task[min_index]);
    } else {
        printk("\n");
        for (size_t i = 1; i < NR_TASKS; i++) {
            task[i]->counter = rand();
            printk(BLUE "set [PID = %d COUNTER = %d]\n" NC, task[i]->pid, task[i]->counter);
        }
        schedule();
    }
}
#endif

#ifdef PRIORITY
auto schedule() -> void {
    auto has_prior = false;
    decltype(current->priority) max_priority = 0;
    decltype(current->pid) min_remaining_time = -1;  // max of decltype(current->pid)
    size_t min_index = 0;
    for (size_t i = 1; i < NR_TASKS; i++) {
        if (task[i]->state != TASK_RUNNING) {
            continue;
        }
        if (task[i]->counter != 0 and task[i]->priority >= max_priority) {
            if ((task[i]->priority == max_priority and task[i]->counter < min_remaining_time) or task[i]->priority > max_priority) {
                max_priority = task[i]->priority;
                has_prior = true;
                min_remaining_time = task[i]->counter;
                min_index = i;
            }
            
        }
    }
    if (has_prior) {
        switch_to(task[min_index]);
    } else {
        printk("\n");
        for (size_t i = 1; i < NR_TASKS; i++) {
            task[i]->counter = rand();
            printk(BLUE "set [PID = %d COUNTER = %d PRIORITY = %d]\n" NC, task[i]->pid, task[i]->counter, task[i]->priority);
        }
        schedule();
    }
}
#endif
