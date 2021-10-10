extern "C" {
    #include "sbi.h"
    void clock_set_next_event();
}

// the frequency of QEMU has been set to be 10MHz
// which means one second contains 10000000 clock cycles。
const unsigned long TIMECLOCK = 10000000;

auto get_cycles() {
    // get the value stored in `time` register
    unsigned long mtime;
    __asm__ volatile (
        "rdtime %[mtime]"
        : [mtime] "=r" (mtime)
        : 
        : "memory"
    );
    return mtime;
}

auto clock_set_next_event() -> void {
    // time of the next timer interrupt
    const auto next = get_cycles() + TIMECLOCK;

    // use sbi_ecall to set the next timer interrupt
    // sbi_set_timer: fid 0, ext 0
    sbi_ecall(0, 0, next, 0, 0, 0, 0, 0);
    return;
}
