#ifndef _DEFS_H
#define _DEFS_H

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define BLUE  "\033[0;34m"
#define RED   "\033[0;31m"
#define NC "\033[0m"

#define PHY_START 0x0000000080000000
#define PHY_SIZE  128 * 1024 * 1024 // 128MB, default size of QEMU memory 
#define PHY_END   (PHY_START + PHY_SIZE)

#define PGSIZE 0x1000 // 4KB
#define PGROUNDUP(addr) ((addr + PGSIZE - 1) & (~(PGSIZE - 1)))
#define PGROUNDDOWN(addr) (addr & (~(PGSIZE - 1)))

// the following macros are used to calculate the physical address
// which will be used in vmlinux.lds.S

#define RISCV_SV39_MODE_MASK 0x8000000000000000  // mode number 8
#define OPENSBI_SIZE (0x200000)

#define VM_START (0xffffffe000000000)
#define VM_END   (0xffffffff00000000)
#define VM_SIZE  (VM_END - VM_START)

#define PA2VA_OFFSET (VM_START - PHY_START)



#define csr_read(csr)                       \
({                                          \
    register unsigned int __v;              \
    __asm__ volatile ("csrr %0, " #csr      \
            : "=r"(__v));                   \
    __v;                                    \
})

#define csr_write(csr, val)                 \
({                                          \
    unsigned int __v = (uint64)(val);       \
    __asm__ volatile ("csrw " #csr ", %0"   \
                    : : "r" (__v)           \
                    : "memory");            \
})

#endif
