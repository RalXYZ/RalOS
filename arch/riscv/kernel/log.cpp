extern "C" {
    #include "log.h"
    #include "defs.h"
    #include "printk.h"
}

auto log_ok(char* content) -> void {
    printk("[  " GREEN "OK" NC "  ] %s\n", content);
}

auto log_failed(char* content) -> void {
    printk("[" RED "FAILED" NC "] %s\n", content);
}
