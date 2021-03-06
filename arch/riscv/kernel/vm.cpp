#include "vm.hpp"

extern "C" {
    #include "mm.h"
    #include "log.h"
    #include "defs.h"
    #include "string.h"
}

extern "C" {
    void setup_vm();
    void setup_vm_final();
}

using uint64 = unsigned long;

// generated by vmlinux.lds.S
extern uint64 _stext[];
extern uint64 _etext[];
extern uint64 _sdata[];
extern uint64 _edata[];
extern uint64 _sbss[];
extern uint64 _ebss[];
extern uint64 _srodata[];
extern uint64 _erodata[];
extern uint64 _ekernel[];
extern uint64 uapp_start[];
extern uint64 uapp_end[];

static constexpr auto PGTBL_ELEMENT_COUNT = PGSIZE / sizeof(uint64);

// early_pgtbl: used for 1GB mapping of setup_vm
uint64 early_pgtbl[PGTBL_ELEMENT_COUNT] __attribute__((__aligned__(0x1000)));

uint64 swapper_pg_dir[PGTBL_ELEMENT_COUNT] __attribute__((__aligned__(0x1000)));

// protection bits of Page Table Entries: | RSW |D|A|G|U|X|W|R|V|
static constexpr auto PTE_V = 0b00'0000'0001ul;
static constexpr auto PTE_R = 0b00'0000'0010ul;
static constexpr auto PTE_W = 0b00'0000'0100ul;
static constexpr auto PTE_X = 0b00'0000'1000ul;
static constexpr auto PTE_U = 0b00'0001'0000ul;

static constexpr auto PTE_FLAGS_LEN = 10ul;

static constexpr auto PHY_START_CPP = static_cast<uint64>(PHY_START);
static constexpr auto VM_START_CPP = static_cast<uint64>(VM_START);
static constexpr auto PA2VA_OFFSET_CPP = static_cast<uint64>(PA2VA_OFFSET);

// length of page offset for both virtual and physical address
// PTE: |  44 PPN  |  10 flags |
static constexpr auto PAGE_OFFSET_lEN = 12ul;
static constexpr auto PAGE_INDEX_LEN = 9ul;

static consteval auto generate_mask(const uint64 len) -> uint64 {
    return (1 << len) - 1;
}

auto va_to_pa(const uint64 va) -> uint64 {
    return va - PA2VA_OFFSET_CPP;
}

static auto panic(char *content) -> void {
    log_failed(content);
    __asm__ volatile("ebreak");
};

static constexpr auto get_pgtbl_index(const uint64 va, const uint64 level) -> uint64 {
    auto ret_val = va;
    ret_val >>= PAGE_OFFSET_lEN;
    switch (level) {
        case 1:
            ret_val >>= PAGE_INDEX_LEN;
        [[fallthrough]];
        case 2:
            ret_val >>= PAGE_INDEX_LEN;
        [[fallthrough]];
        case 3:
            ret_val &= generate_mask(PAGE_INDEX_LEN);
            break;
        default:
            panic(const_cast<char*>("Kernel panic: invalid page table level"));
    }
    return ret_val;
}

static inline constexpr auto get_ppn_pgtbl_addr(const uint64 pte) -> uint64 {
    return pte 
            >> PTE_FLAGS_LEN 
            << PAGE_OFFSET_lEN;
}

static inline constexpr auto set_pte(const uint64 page_addr, const uint64 flags) -> uint64 {
    return page_addr
            >> PAGE_OFFSET_lEN          // get 44-bit PPN of PTE
            << PTE_FLAGS_LEN            // move left 10 bits for flags
            |  flags;                   // set flags
}

// perform a 1GB mapping, only use one level page table
auto setup_vm() -> void {
    memset(early_pgtbl, 0x0, PGSIZE);

    constexpr auto pte = set_pte(PHY_START_CPP, PTE_V | PTE_R | PTE_W | PTE_X);

    // we split 64-bit va into: | high bit | 9 bit | 30 bit |
    // the middle 9 bits are used as index of early_pgtbl

    // since we only use one level page table,
    // then a PTE is enough to map 1GB

    // map 0x80000000(mem) to 0x80000000(phy)
    early_pgtbl[get_pgtbl_index(PHY_START_CPP, 1)] = pte;

    // map 0xffffffe000000000(mem) to 0x80000000(phy)
    early_pgtbl[get_pgtbl_index(VM_START_CPP, 1)] = pte;

    log_ok(const_cast<char*>("First-time virtual memory mapping finished"));
}

// create multi-level page table mapping
// pgtbl: the address of base page table
// va, pa: the virtual and physical address to be mapped
// sz: the size of the mapping
// flags: the protection bits of the mapping
static auto create_mapping(uint64* const pgtbl, const uint64 va, const uint64 pa, const uint64 sz, const uint64 flags) -> void {
    const auto PTE_V_FINAL = PTE_V | (flags & PTE_U);
    for (auto i = 0ul; i < sz; i += PGSIZE) {
        const auto current_va = va + i;
        const auto current_pa = pa + i;
        const auto level_1_index = get_pgtbl_index(current_va, 1);
        const auto level_2_index = get_pgtbl_index(current_va, 2);
        const auto level_3_index = get_pgtbl_index(current_va, 3);
    
        if ((pgtbl[level_1_index] & generate_mask(PTE_FLAGS_LEN)) != PTE_V_FINAL) {
            auto new_pgtbl = kalloc();
            pgtbl[level_1_index] = set_pte(va_to_pa(new_pgtbl), PTE_V_FINAL);
        }

        auto* const level_2_pgtbl_ptr = reinterpret_cast<uint64 *>(get_ppn_pgtbl_addr(pgtbl[level_1_index]) + PA2VA_OFFSET_CPP);

        if ((level_2_pgtbl_ptr[level_2_index] & generate_mask(PTE_FLAGS_LEN)) != PTE_V_FINAL) {
            auto new_pgtbl = kalloc();
            level_2_pgtbl_ptr[level_2_index] = set_pte(va_to_pa(new_pgtbl), PTE_V_FINAL);
        }

        auto* const level_3_pgtbl_ptr = reinterpret_cast<uint64 *>(get_ppn_pgtbl_addr(level_2_pgtbl_ptr[level_2_index]) + PA2VA_OFFSET_CPP);

        level_3_pgtbl_ptr[level_3_index] = set_pte(current_pa, flags);
    }
}

// perform a 128MB mapping, using three level page table
// and considering kernel segments 
auto setup_vm_final() -> void {
    auto stext_addr = reinterpret_cast<uint64>(_stext);
    auto etext_addr = reinterpret_cast<uint64>(_etext);
    auto sdata_addr = reinterpret_cast<uint64>(_sdata);
    auto edata_addr = reinterpret_cast<uint64>(_edata);
    auto sbss_addr = reinterpret_cast<uint64>(_sbss);
    auto ebss_addr = reinterpret_cast<uint64>(_ebss);
    auto srodata_addr = reinterpret_cast<uint64>(_srodata);
    auto erodata_addr = reinterpret_cast<uint64>(_erodata);
    auto ekernel_addr = reinterpret_cast<uint64>(_ekernel);

    memset(swapper_pg_dir, 0x0, PGSIZE);

    // No OpenSBI mapping required

    // mapping kernel text X|-|R|V
    create_mapping(swapper_pg_dir,
            stext_addr,
            va_to_pa(stext_addr),
            PGROUNDUP(etext_addr - stext_addr),
            PTE_V | PTE_R | PTE_X
    );
    log_ok(const_cast<char*>(".text   virtual memory mapped, permission set to R-X"));

    // mapping kernel rodata -|-|R|V
    create_mapping(swapper_pg_dir,
            srodata_addr,
            va_to_pa(srodata_addr),
            PGROUNDUP(erodata_addr - srodata_addr),
            PTE_V | PTE_R
    );
    log_ok(const_cast<char*>(".rodata virtual memory mapped, permission set to R--"));

    // mapping other memory -|W|R|V
    create_mapping(swapper_pg_dir,
            sdata_addr,
            va_to_pa(sdata_addr),
            PGROUNDUP(edata_addr - sdata_addr),
            PTE_V | PTE_R | PTE_W
    );
    log_ok(const_cast<char*>(".data   virtual memory mapped, permission set to RW-"));
    create_mapping(swapper_pg_dir,
            sbss_addr,
            va_to_pa(sbss_addr),
            PGROUNDUP(ebss_addr - sbss_addr),
            PTE_V | PTE_R | PTE_W
    );
    log_ok(const_cast<char*>(".bss    virtual memory mapped, permission set to RW-"));

    const uint64 ekernel_rnd_up_addr = PGROUNDUP(ekernel_addr);
    create_mapping(swapper_pg_dir,
            ekernel_rnd_up_addr,
            va_to_pa(ekernel_rnd_up_addr),
            PGROUNDUP(VM_START + PHY_SIZE - ekernel_rnd_up_addr),
            PTE_V | PTE_R | PTE_W
    );
    log_ok(const_cast<char*>("Other   virtual memory mapped, permission set to RW-"));

    // set satp with swapper_pg_dir
    const uint64 satp_content = va_to_pa(reinterpret_cast<uint64>(swapper_pg_dir))
            >> PAGE_OFFSET_lEN
            |  RISCV_SV39_MODE_MASK; 
    __asm__ volatile (
        "csrw satp, %[satp_content]"
        : 
        : [satp_content] "r" (satp_content)
        : "memory"
    );

    // flush TLB
    __asm__ volatile("sfence.vma zero, zero");

    log_ok(const_cast<char*>("TLB flushed, second-time virtual memory initialization succeeded"));

    return;
}

[[nodiscard]]
auto construct_u_mode_pgtbl() -> uint64* {
    // in order to prevent page table switching while switching 
    // between U-Mode and S-Mode, we also copy kernel page table 
    // `swapper_pg_dir` to each process' page table
    auto* const u_mode_pgtbl = reinterpret_cast<uint64*>(kalloc());
    for (auto i = 0ul; i < PGTBL_ELEMENT_COUNT; i++) {
        u_mode_pgtbl[i] = swapper_pg_dir[i];
    }

    // user mode `uapp` memory mapping
    auto uapp_start_addr = reinterpret_cast<uint64>(uapp_start);
    auto uapp_end_addr = reinterpret_cast<uint64>(uapp_end);
    auto uapp_start_addr_round_down = PGROUNDDOWN(uapp_start_addr);
    
    // The user program requires accessing (read and write) 
    // .data segment, since it has global variable. In this 
    // case, we need to set R-W-X permission.
    create_mapping(u_mode_pgtbl,
            USER_START,
            va_to_pa(uapp_start_addr_round_down),
            PGROUNDUP(uapp_end_addr - uapp_start_addr_round_down),
            PTE_V | PTE_R | PTE_W | PTE_X | PTE_U
    );

    // user mode stack memory mapping
    create_mapping(u_mode_pgtbl,
            USER_END - PGSIZE,
            va_to_pa(kalloc()),     // kalloc the user mode stack
            PGSIZE,
            PTE_V | PTE_R | PTE_W | PTE_U
    );

    return u_mode_pgtbl;
}
