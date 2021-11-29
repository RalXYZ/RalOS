#include "log.h"
#include "sbi.h"

extern void test();

int start_kernel() {
    log_ok("Kernel started");
                             
    test(); // DO NOT DELETE !!!

	return 0;
}
