extern "C" {
    #include <defs.h>
}

extern "C" {
    void setup_vm();
}

using uint64 = unsigned long;

/* early_pgtbl: used for 1GB mapping of setup_vm */
unsigned long early_pgtbl[512] __attribute__((__aligned__(0x1000)));

// protection bits of Page Table Entries: | RSW |D|A|G|U|X|W|R|V|
// set bit V | R | W | X to 1
const uint64 PTE_VRWX = 0b00'0000'1111;

const uint64 PTE_PROTECTION_BIT_LEN = 10;

const uint64 PHY_START_CPP = static_cast<uint64>(PHY_START);
const uint64 VM_START_CPP = static_cast<uint64>(VM_START);

// length of page offset for both virtual and physical address
const uint64 PAGE_OFFSET_lEN = 12;



auto setup_vm() -> void {
    /* 
    1. 由于是进行 1GB 的映射 这里不需要使用多级页表 
    2. 将 va 的 64bit 作为如下划分： | high bit | 9 bit | 30 bit |
        high bit 可以忽略
        中间9 bit 作为 early_pgtbl 的 index
        低 30 bit 作为 页内偏移 这里注意到 30 = 9 + 9 + 12， 即我们只使用根页表， 根页表的每个 entry 都对应 1GB 的区域。 
    3. Page Table Entry 的权限 V | R | W | X 位设置为 1
    */
    for (uint64 i = 0; i < 512; i++) {
        early_pgtbl[i] = 0;
    }
    early_pgtbl[PHY_START_CPP >> 30] = PHY_START_CPP >> PAGE_OFFSET_lEN << PTE_PROTECTION_BIT_LEN;
    early_pgtbl[VM_START_CPP >> 30] = PHY_START_CPP >> PAGE_OFFSET_lEN << PTE_PROTECTION_BIT_LEN;
}