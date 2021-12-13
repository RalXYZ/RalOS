#include "log.h"
#include "sbi.h"
#include "proc.h"

extern void test();

int start_kernel() {
    log_ok("Kernel started");

    schedule();
                             
    test(); // DO NOT DELETE !!!

	return 0;
}
