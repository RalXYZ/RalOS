#ifndef _DEFS_H
#define _DEFS_H

#define PHY_START 0x0000000080000000
#define PHY_SIZE  128 * 1024 * 1024 // 128MB, default size of QEMU memory 
#define PHY_END   (PHY_START + PHY_SIZE)

#define PGSIZE 0x1000 // 4KB
#define PGROUNDUP(addr) ((addr + PGSIZE - 1) & (~(PGSIZE - 1)))
#define PGROUNDDOWN(addr) (addr & (~(PGSIZE - 1)))

// the following macros are used to calculate the physical address
// which will be used in vmlinux.lds.S

#define OPENSBI_SIZE (0x200000)

#define VM_START (0xffffffe000000000)
#define VM_END   (0xffffffff00000000)
#define VM_SIZE  (VM_END - VM_START)

#define PA2VA_OFFSET (VM_START - PHY_START)



#define csr_read(csr)                       \
({                                          \
    register unsigned int __v;              \
    asm volatile ("csrr %0, " #csr          \
		    : "=r"(__v));                   \
    __v;                                    \
})

#define csr_write(csr, val)                 \
({                                          \
    unsigned int __v = (uint64)(val);       \
    asm volatile ("csrw " #csr ", %0"       \
                    : : "r" (__v)           \
                    : "memory");            \
})

#endif
