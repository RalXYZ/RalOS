#include "syscall.hpp"

extern "C" {
    #include "printk.h"
    #include "proc.h"
}

extern "C" {
    extern struct task_struct* current;
}

auto sys_write(uint32_t fd, const char *buf, uint64_t count) -> uint64_t {
    if (fd == 1) {
        for (uint64_t i = 0; i < count; i++) {
            putc(buf[i]);
        }
        return count;
    }
    return static_cast<uint64_t>(-1);
}

auto sys_getpid() -> uint64_t {
    return current->pid;
}
