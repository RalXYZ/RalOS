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
constexpr uint64 PTE_VRWX = 0b00'0000'1111;

constexpr uint64 PTE_PROTECTION_BIT_LEN = 10;

constexpr uint64 PHY_START_CPP = static_cast<uint64>(PHY_START);
constexpr uint64 VM_START_CPP = static_cast<uint64>(VM_START);

// length of page offset for both virtual and physical address
// PTE: |  44 PPN  |  10 flags |
constexpr uint64 PAGE_OFFSET_lEN = 12;

// perform a 1GB mapping, only use one level page table
auto setup_vm() -> void {
    for (auto i = 0; i < 512; i++) {
        early_pgtbl[i] = 0;
    }

    constexpr auto one_level_pte_content = PHY_START_CPP 
            >> PAGE_OFFSET_lEN                               // get 44 PPN of PTE
            << PTE_PROTECTION_BIT_LEN                        // move left 10 bits for flags
            |  PTE_VRWX;                                     // set V | R | W | X to 1

    // we split 64-bit va into: | high bit | 9 bit | 30 bit |
    // the middle 9 bits are used as index of early_pgtbl
    early_pgtbl[PHY_START_CPP >> 30 & 0b1111'1111] = one_level_pte_content;
    early_pgtbl[VM_START_CPP >> 30 & 0b1111'1111] = one_level_pte_content;
}


