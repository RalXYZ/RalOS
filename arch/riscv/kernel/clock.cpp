extern "C" {
    #include "sbi.h"
    void clock_set_next_event();
}

using time_t = unsigned long;

// the frequency of QEMU has been set to be 10MHz
// which means one second contains 10000000 clock cycles
time_t TIME_CLOCK = 10'000'000;  // do not change this value

auto get_cycles() {
    // get the value stored in `time` register
    time_t mtime;
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
    const auto next = get_cycles() + TIME_CLOCK;

    // use sbi_ecall to set the next timer interrupt
    // sbi_set_timer: fid 0, ext 0
    sbi_ecall(0, 0, next, 0, 0, 0, 0, 0);
    return;
}
