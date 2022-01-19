# RalOS

A simple OS built from scratch, which is the final project of ZJU *Operating System* course.  

## Feature List

- [Bootstrap](#kernel-boot-load)
- [Interrupt handling](#timer-interrupt)
- [Process scheduling](#process-scheduling)
- [Virtual memory](#virtual-memory-management )
- [User mode](#user-mode)
- Page fault handling and `fork` (hasn't been implemented yet)

## Highlights

- Many core procedures are implemented using C++ as the programming language
- You can find reasonable usages of some modern C++ features, like enumeration base, `decltype`, `consteval`, `[[fallthrough]]` and so on
- Compile with `-Werror -Wall -Wextra` tags

## Build

Firstly, pull `alphavake/oslab` and start a container from this image. This image includes build tools, as well as Qemu.  

```shell
docker pull alphavake/oslab
# then start a container from this image
```

Then, clone this repository in the container, and build it.  

```shell
git clone https://github.com/RalXYZ/RalOS.git
cd RalOS
make        # build
make run    # build and run
```

![lab5](https://raw.githubusercontent.com/RalXYZ/repo-pictures/main/RalOS/lab5.png)



## Kernel Boot-load

- Implement `head.S` and `Makefile`, call OpenSBI API

### Implement `head.S`

We allocated $\text{0x1000}$ bytes using `.space`which will be our stack, and `boot_stack_top` is the top of this stack, which is currently empty. In this case, the first thing we need to do is to load the address of stack top to `sp`:  

```asm
la sp, boot_stack_top
```

Then, we just need to jump to `start_kernel`. There is no need to perform stack push and update `ra`, because the program is not supposed to return to `_start` again:  

```asm
j start_kernel
```

In this case, `head.S` will looks like this:  

```asm
.extern start_kernel

    .section .text.entry
    .globl _start
_start:
    la sp, boot_stack_top
    j start_kernel
    .section .bss.stack
    .globl boot_stack
boot_stack:
    .space 0x1000

    .globl boot_stack_top
boot_stack_top:
```

Which will be compiled into:  

```asm
0000000000000000 <_start>:
   0:   00000117                auipc   sp,0x0
   4:   00010113                mv      sp,sp
   8:   ff9ff06f                j       0 <_start>
```

### Implement `lib/Makefile`

This step is simple, because the only thing I need to do is refer to the other Makefiles that have been already given. In this case, writing this Makefile is easy:  

```makefile
C_SRC  = $(sort $(wildcard *.c))
OBJ    = $(patsubst %.c,%.o,$(C_SRC))

file = print.o
all: $(OBJ) 

%.o:%.c
        ${GCC}  ${CFLAG}  -c $<

clean:
        $(shell rm *.o 2>/dev/null)
```

### Implement `sbi.c`

The operation we perform in `sbi_ecall()`, is to pass some arguments, make a `ecall`, and get the return value.  
Some other things also need to be specified, telling the compiler registers you've modified in the inline assembly.  

```c
#include "types.h"
#include "sbi.h"


struct sbiret sbi_ecall(int ext, int fid, uint64 arg0,
                                    uint64 arg1, uint64 arg2,
                                    uint64 arg3, uint64 arg4,
                                    uint64 arg5) 
{
        struct sbiret ret_val;
        __asm__ volatile (
                "mv a7, %[ext]\n"
                "mv a6, %[fid]\n"
                "mv a0, %[arg0]\n"
                "mv a1, %[arg1]\n"
                "mv a2, %[arg2]\n"
                "mv a3, %[arg3]\n"
                "mv a4, %[arg4]\n"
                "mv a5, %[arg5]\n"
                "ecall\n"
                "mv %[ret_val_error], a0\n"
                "mv %[ret_val_value], a1"
                : [ret_val_error] "=r" (ret_val.error), [ret_val_value] "=r" (ret_val.value)
                : [ext] "r" (ext), [fid] "r" (fid), [arg0] "r" (arg0), [arg1] "r" (arg1), [arg2] "r" (arg2), [arg3] "r" (arg3), [arg4] "r" (arg4), [arg5] "r" (arg5)
                : "memory"
        );
        return ret_val;           
}
```

The source code shown above will be compiled into:  

```asm
0000000000000000 <sbi_ecall>:
   0:   ff010113                addi    sp,sp,-16
   4:   00050893                mv      a7,a0
   8:   00058813                mv      a6,a1
   c:   00060513                mv      a0,a2
  10:   00068593                mv      a1,a3
  14:   00070613                mv      a2,a4
  18:   00078693                mv      a3,a5
  1c:   00080713                mv      a4,a6
  20:   00088793                mv      a5,a7
  24:   00000073                ecall
  28:   00050513                mv      a0,a0
  2c:   00058593                mv      a1,a1
  30:   01010113                addi    sp,sp,16
  34:   00008067                ret
```



## Timer Interrupt

- Complete the initialization of exception handling

- Understand the concept of context switching, and correctly implement context switch

- Implement exception handling function

- Call OpenSBI API to complete the setting of timer interrupt

### Set Exception Handling

Firstly, `sp` need to be set propely to the stack top:  

```asm
la sp, boot_stack_top
```
Then, set `stvec` to the address of `_traps`:  

```asm
la a0, _traps
csrw stvec, a0
```

After that, to enable timer interrupt, set `sie[SITE]` to 1. The structure of `sie` is:  

|  ...  |   SEIE   |   UEIE   |  0    |   0   |   **SITE**   |   UTIE   |   0   |   0   |  SSIE    |   USIE   |
| --- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |

`SITE` is at the sixth bit. This means, the sixth bit need to be set 1:  

```asm
csrr a0, sie
ori a0, a0, 0x20   # mask the sixth bit
csrw sie, a0
```

To set the first timer interrupt, we need to get the current time, then use `ecall` to set a timer interrupt after 1s:  

```asm
rdtime a0          # get current time
ld a1, TIME_CLOCK
add a0, a0, a1     # +1s
li a1, 0
li a2, 0
li a3, 0
li a4, 0
li a5, 0
li a6, 0  # fid
li a7, 0  # ext
ecall
```

To enable the interrupt response in S mode, set `sstatus[SIE]` to 1. The structure of `sstatus` is:  

| ...  | UPIE | 0    | 0    | **SIE** | UIE  |
| ---- | ---- | ---- | ---- | ------- | ---- |

In this case, the second bit should be set:  

```asm
csrsi sstatus, 0b10
```

### Implement Context Switch

Since we need to store all registers, the following assembly code must exist:  
```asm
sd x1, 0(sp)
sd x2, 8(sp)
sd x3, 16(sp)
sd x4, 24(sp)
sd x5, 32(sp)
# ... multiple lines omitted
```

However, the problem is, the programmer must manually type in 32 lines of assembly code, which is ineffective. We need to find out a proper way to deal with this problem.  
Note: the following description will use `sd` as an example. The `ld` procedure is simmilar to this.  

#### Function
Each `sd` assembly instruction can be described as the following equation:  

$$
\text{sd x}n\text{, }((n-1)*8)\text{(sp)}
$$

In which $n = 1, 2, 3, ..., 31$.  

#### GCC Macro Function
The equation shown above can be implemented by GCC macro function:  

```asm
.macro SAVE_TO_SP_OFFSET reg, offset
    sd \reg, (\offset) * 8(sp)
.endm

.macro SAVE_TO_SP_OFFSET_N n
    SAVE_TO_SP_OFFSET x\n, (\n) - 1
.endm
```

In this case, a call of `SAVE_TO_SP_OFFSET_N 5` will be translated to:  

```asm
SAVE_TO_SP_OFFSET x5, (5) - 1
```

Which will then be translated to:  

```asm
sd x5, ((5) - 1) * 8(sp)
```

And will be executed as:  

```asm
sd x5, 32(sp)
```

#### `.rept`

Next, we need to repeat the macro function. Luckily, GCC provides `.rept`. If we want to use this macro, we need to declare `.altmacro` in advance.  

```asm
.set n, 1
.rept 31
    SAVE_TO_SP_OFFSET_N %n
    .set n, n + 1
.endr
```

`n` will increase by 1 in every iteration, and the loop will iterate 31 times.  

#### Body

The following code shows the whole procedure of register backup:  

```asm
addi sp, sp, -1 * REG_SIZE * REG_NUM
.set n, 1
.rept 31
    SAVE_TO_SP_OFFSET_N %n
    .set n, n + 1
.endr
csrr a1, sepc
SAVE_TO_SP_OFFSET a1, 31
```

Firstly, the stack has been pushed by a proper amount. Then, store the 31 register, and store `spec`.  

We have already stored the value of `sepc` into `a1`, which happens to be the second parameter of `trap_handler`. So, we need to set `a0` (which is another parameter) properly, then call `trap_handler`.  

```asm
csrr a0, scause
jal ra, trap_handler   
```

Then, restore `sepc` and the 32 registers from stack:  

```asm
LOAD_FROM_SP_OFFSET t0, 31
csrw sepc, t0
.set n, 1
.rept 31
    LOAD_FROM_SP_OFFSET_N %n
    .set n, n + 1
.endr
addi sp, sp, REG_SIZE * REG_NUM
```

Since we are currently in `S` mode, we need to use `sret` to set `pc=sepc`:  

```asm
sret
```

### Implement Error Handling Function

The highest bit of `scause` stores whether an interrupt has been caused. If the low bit is 5, it means a timer interrupt has occurred.  

```c
#include "printk.h"
#include "clock.h"

#define INTERRUPT_MASK 0x8000000000000000

void trap_handler(unsigned long scause, unsigned long sepc) {
    // judge the type of trap by scause
    if (!(scause & INTERRUPT_MASK)) {
        return;
    }

    // judge whether it is a timer interrupt
    if ((scause & (~INTERRUPT_MASK)) != 5) {  // Supervisor timer interrupt
        return;
    }

    // output message, set the next timer interrupt by calling clock_set_next_event
    printk("kernel is running!\n[S] Supervisor Mode Timer Interrupt\n");
    clock_set_next_event();

    // other interrupts or exceptions can be ignored
    return;
}
```

### Implement Timer Interrupt

Just get current time, add 1 second, and set it as the next timer interrupt time.  

```c++
extern "C" {
    #include "sbi.h"
    void clock_set_next_event();
}

unsigned long TIME_CLOCK = 10'000'000;  // const

auto get_cycles() {
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
    // next timer interrupt
    const auto next = get_cycles() + TIMECLOCK;

    // use sbi_ecall to set the next timer interrupt
    // sbi_set_timer: fid 0, ext 0
    sbi_ecall(0, 0, next, 0, 0, 0, 0, 0);
    return;
}
```

Whats interesting is that this is a C++ file. To compile this file properly, some changes needs to be made in the makefile:  

```makefile
CXX_SRC     = $(sort $(wildcard *.cpp))
OBJ		    = $(patsubst %.S,%.o,$(ASM_SRC)) $(patsubst %.c,%.o,$(C_SRC)) $(patsubst %.cpp,%.o,$(CXX_SRC))
%.o:%.cpp
	${GXX}  ${CFLAG} -c $<
```

Also, we need to specify the C++ compiler in the root makefile:  

```makefile
GXX=${CROSS_}g++
```



## Process Scheduling

- Understand the concept of thread, learn the struct related to thread, and implement the initialization of a thread.  
- Understand how to implement thread scheduling by using clock interrupts.  
- Understand the principle of thread switching, and implement it.  
- Understand the basic thread scheduling algorithm, and implement two simple scheduling algorithm.  

### Task Initialization

#### Initialize Idle Task

Firstly, a physical page needs to be allocated, and the values in the `task_struct` needs to be initialized properly. Since `task_struct` is located at the top of the allocated memory, we can convert the pointer to this memory space to a `task_struct` pointer.  

```c++
// call kalloc() to allocate a physical page for idle
idle = reinterpret_cast<task_struct *>(kalloc());

// set state to TASK_RUNNING
idle->state = TASK_RUNNING;

// since idle does not participate in scheduling, 
// we can set its counter and priority to 0
idle->counter = 0;
idle->priority = 0;

//  set the pid of idle to 0
idle->pid = 0;

// point current and task[0] to idle
current = idle;
task[0] = idle;
```

### Initialize all Other Tasks

The initialization of other task are the same. After a memory space has been allocated, various values, like state, counter, priority and pid has been set.  
There are two important assignments. Firstly, `ra` has been set to the address of `__dummy` symbol. This enables the procedure jumps to `__dummy` after the first context switching. What's more, `sp` has bees set to `task[i] + PGSIZE`, which is the end of the allocated memory.  

```c++
// initialize task[1] ~ task[NR_TASKS - 1]
for (size_t i = 1; i < NR_TASKS; i++) {
    task[i] = reinterpret_cast<task_struct *>(kalloc());
    task[i]->state = TASK_RUNNING;
    task[i]->counter = 0;
    task[i]->priority = rand();
    task[i]->pid = i;

    // set `ra` and `sp` in `thread_struct`
    // `ra` is set to be the address of __dummy
    // `sp` is set to the highest address of the applied physical page
    task[i]->thread.ra = reinterpret_cast<uint64>(__dummy);
    task[i]->thread.sp = reinterpret_cast<uint64>(task[i]) + PGSIZE;
}
```

### Dummy

#### `__dummy`

The reason why we implement `__dummy` is that there needs a handler during the first context switch. Since the context switch happens the first time, we must specify a special address for it to jump to. Meanwhile, the procedure is in an interrupt, and needs to return to an ordinary memory space by using `sret`. This results in the implementation of the following code fragment.  

```asm
.global __dummy
__dummy:
    la t0, dummy
    csrw sepc, t0
    sret
```

#### Dummy Function

The core of this so called *dummy function* is a infinite loop that does nothing. What's important is that if the timer interrupt happened and the context switch has not been triggered, then the `auto_inc_local_var` will increase by one, and will be outputted to standard output.  

```c++
auto dummy() -> void {
    const uint64 MOD = 1'000'000'007;
    uint64 auto_inc_local_var = 0;
    while (true) {
        if (current->counter != static_cast<uint64>(::last_counter) or ::last_counter == -1) {
            ::last_counter = current->counter;
            auto_inc_local_var = (auto_inc_local_var + 1) % MOD;
            printk(GREEN "[PID = %d]" NC "is running. auto_inc_local_var = %d\n", current->pid, auto_inc_local_var);
        }
    }
}
```

### Context Switch

Finally, we are about to implement context switch. It is simple: set current pointer to previous pointer, and set next pointer to current pointer. 

```c++
if (next != current) {
    auto prev = current;
    current = next;
    ::last_counter = -1;
    printk(YELLOW "\nswitch to [PID = %d COUNTER = %d PRIORITY = %d]\n" NC, next->pid, next->counter, next->priority);
    __switch_to(prev, next);
}
```

What truly complicated is the `__siwtch_to` assembly. It saves `ra`, `sp` and twelve `s` registers. The tricky thing is that I implemented a GNU AS macro to fulfill this requirement.  

```asm
.macro SAVE_SX n, base
    sd s\n, \n * 8(\base)
.endm

addi t0, t0, 2 * 8
.set n, 0
.rept 12
    SAVE_SX %n, t0
    .set n, n + 1
.endr
```

`.rept 12` can *repeat* the assembly code in the code block 12 times. For each time, `.set n, n + 1` adds n by one.  

### Scheduling 

#### SJF (Shortest Job First)

This algorithm set the next task to be the shortest task. Here I use a brute force algorithm, traversing and finding the shortest task, and the time complexity is certainly $O(n)$. 

```c++
auto schedule() -> void {
    auto has_min = false;
    decltype(current->pid) min_remaining_time = -1;  // max of decltype(current->pid)
    size_t min_index = 0;
    for (size_t i = 1; i < NR_TASKS; i++) {
        if (task[i]->state != TASK_RUNNING) {
            continue;
        }
        if (task[i]->counter != 0 and task[i]->counter < min_remaining_time) {
            has_min = true;
            min_remaining_time = task[i]->counter;
            min_index = i;
        }
    }
    if (has_min) {
        switch_to(task[min_index]);
    } else {
        printk("\n");
        for (size_t i = 1; i < NR_TASKS; i++) {
            task[i]->counter = rand();
            printk(BLUE "set [PID = %d COUNTER = %d]\n" NC, task[i]->pid, task[i]->counter);
        }
        schedule();
    }
}
```

#### Priority Scheduling

While performing a priority scheduling, I also traversed the tasks, get the task with the highest priority and costs the shortest time. The whole structure is similar to SJF.  



## Virtual Memory Management 

- Implement switching from physical address to virtual address. 
- Implement mapping from physical address to virtual address by using *Sv39* paging mode. 
- Set different permissions to the corresponding segments.

### First-time Mapping

At the very start of the kernel, just after the initialization of `sp`, a memory mapping happens. This mapping makes the kernel executes in virtual address, but doesn't distinguish the privileges. The more detailed privileges will be set in another procedure, which will be covered later.  

`setup_vm` performs two mappings, and the details is shown as below.  

```cpp
// perform a 1GB mapping, only use one level page table
auto setup_vm() -> void {
    memset(early_pgtbl, 0x0, PGSIZE);

    constexpr auto pte = set_pte(PHY_START_CPP, PTE_VRWX);

    // we split 64-bit va into: | high bit | 9 bit | 30 bit |
    // the middle 9 bits are used as index of early_pgtbl

    // since we only use one level page table,
    // then a PTE is enough to map 1GB

    // map 0x80000000(mem) to 0x80000000(phy)
    early_pgtbl[get_pgtbl_index(PHY_START_CPP, 1)] = pte;

    // map 0xffffffe000000000(mem) to 0x80000000(phy)
    early_pgtbl[get_pgtbl_index(VM_START_CPP, 1)] = pte;
}
```

`relocate` adds `ra` and `sp` by the offset between physical address and virtual address. Then, it sets `satp` using `eraly_pgbl`, which is the root page of the current mapping. It is wroth notice that `early_pgtbl` is currently a physical address, since virtual address has not yet implemented into the current system. 

```asm
relocate:
    li t0, PA2VA_OFFSET

    # set ra = ra + PA2VA_OFFSET
    add ra, ra, t0

    # set sp = sp + PA2VA_OFFSET
    add sp, sp, t0

    # set satp with early_pgtbl
    la t1, early_pgtbl
    srli t1, t1, 12              # put 44-bit PPN at the lowest 44 bits
    li t2, RISCV_SV39_MODE_MASK  # set mode to 8, which means page-based 39-bit virtual addressing
    or t1, t1, t2

    csrw satp, t1

    # flush tlb
    sfence.vma zero, zero

    ret
```

### Second-time Mapping

After the first-time mapping succeeded, many procedures can be executed. This includes trap vector setting. However, the interrupt enable bit in `sstatus` cannot be set to `1`, or the timer interrupt will occurs once a second, which may cause error while setting page table.  
The function `setup_vm_final` must be called *after* `mm_init`. This is because `setup_vm_final` depends on `kalloc`, and the reason why we can use this function properly is `mm_init` set the memory free list up.  
In this case, the position we need to place `setup_vm_final` is quite clear, that is, after `mm_init` and before the setting of `sstatus`.  

```asm
    # initialize memory management system
    call mm_init

    # after initialized free list of memory, we can use kalloc() to allocate page table
    call setup_vm_final  

    # sstatus: ... UPIE 0 0 SIE UIE
    # set sstatus[SIE] = 1
    csrsi sstatus, 0b10
```

#### `setup_vm_final`

The procedure of this function is very straight-forward. It creates maps `.text`, `.rodata`, `.data`, `.bss` and other memory in QEMU. Finally, it sets `satp` with `swapper_pg_dir`, the root page table of the current mapping procedure, and the inline assembly of this procedure is shown below:  

```c
__asm__ volatile (
     "csrw satp, %[satp_content]"
     : 
     : [satp_content] "r" (satp_content)
     : "memory"
 );
```

#### create_mapping

The most complicated part of the virtual memory project is, apparently, implementing this three layers mapping. The thing I want to stress is that the **physical page number (PPN) stored in page table entry (PTE)** is constructed by the physical address, not the virtual address. This means that the whole procedure of page table addressing is based on physical memory. The only thing related to virtual memory is the 39-bit virtual memory input we want to map.  
Furthermore, while setting a PTE, one thing worth mention is that RISCV will try to find the next level of page table **iff** RWX bits are set to `0` and V bit is set to `1`. 
The brief implementation is shown as follows.  

- for each page to be mapped:  
    - find PTE in root page table by using `va[38:30]` as index
    - if PTE is not valid:
        - allocate a new page, **convert the page address to physical address** and store it into PTE
    - use the address stored in PTE as the address of the second level page table
    - find PTE in second level page table by using `va[29:21]` as index
    - if PTE is not valid:
        - allocate a new page, **convert the page address to physical address** and store it into PTE
    - use the address stored in PTE as the address of the third level page table
    - set PTE indexed by `va[20:12]`, using the concatenation of the target physical page number and the flags

```cpp
// create multi-level page table mapping
// pgtbl: the address of base page table
// va, pa: the virtual and physical address to be mapped
// sz: the size of the mapping
// flags: the protection bits of the mapping
auto create_mapping(uint64* const pgtbl, const uint64 va, const uint64 pa, const uint64 sz, const uint64 flags) -> void {
    for (auto i = 0ul; i < sz; i += PGSIZE) {
        const auto current_va = va + i;
        const auto current_pa = pa + i;
        const auto level_1_index = get_pgtbl_index(current_va, 1);
        const auto level_2_index = get_pgtbl_index(current_va, 2);
        const auto level_3_index = get_pgtbl_index(current_va, 3);
    
        if ((pgtbl[level_1_index] & generate_mask(PTE_FLAGS_LEN)) != PTE_V) {
            auto new_pgtbl = kalloc();
            memset(reinterpret_cast<void*>(new_pgtbl), 0x0, PGSIZE);
            pgtbl[level_1_index] = set_pte(va_to_pa(new_pgtbl) , PTE_V);
        }

        auto* const level_2_pgtbl_ptr = reinterpret_cast<uint64 *>(get_ppn_pgtbl_addr(pgtbl[level_1_index]));

        if ((level_2_pgtbl_ptr[level_2_index] & generate_mask(PTE_FLAGS_LEN)) != PTE_V) {
            auto new_pgtbl = kalloc();
            memset(reinterpret_cast<void*>(new_pgtbl), 0x0, PGSIZE);
            level_2_pgtbl_ptr[level_2_index] = set_pte(va_to_pa(new_pgtbl), PTE_V);
        }

        auto* const level_3_pgtbl_ptr = reinterpret_cast<uint64 *>(get_ppn_pgtbl_addr(level_2_pgtbl_ptr[level_2_index]));

        level_3_pgtbl_ptr[level_3_index] = set_pte(current_pa, flags);
    }
}
```



## User Mode

- Create user mode process, and set `sstatus` to switch from kernel mode to user mode.
- Set user mode stack and kernel mode stack of the user process correctly, and switch them while handling the exception. 
- Complete the exception handling procedure, complete the feature of the required system calls: `SYS_WRITE`, `SYS_GETPID`.

### Task Initialization

Since we need to set up four U-mode processes, this procedure should be executed in `task_init`, because that is the function where all processes are created.  
Part of the function remains unchanged, because they are related to the `S-mode` of each process, which must be remained for the correctness of S-mode procedure. This includes process state, process counter, process priority, process id, initial return value and the initial value of S-mode stack pointer.  

```c++
task[i] = reinterpret_cast<task_struct *>(kalloc());
task[i]->state = TASK_RUNNING;
task[i]->counter = 0;
task[i]->priority = rand();
task[i]->pid = i;
task[i]->thread.ra = reinterpret_cast<uint64>(__dummy);
task[i]->thread.sp = reinterpret_cast<uint64>(task[i]) + PGSIZE;
```

The rest of the function changed. Firstly, `sepc` has been set to `USER_START`, which is `0x0`, the starting address of the user function. `sscratch` has been set to `USER_END`, because it is the bottom of the user stack. The page table of U-mode is set in `construct_u_mode_pgtbl`, which will be talked in depth in the next section.  
The only thing remains to be discussed here is `sstatus`. To protect the original data in `sstatus`, we need to read `sstatus` in advanced. Then, based on the result, we set `SPP` to zero, and set `SPIE`, `SUM` to one.

```c++
task[i]->thread.sepc = USER_START;
task[i]->thread.sstatus = csr_read(sstatus);
task[i]->thread.sstatus &= ~(1ull << 8);  // clear SPP
task[i]->thread.sstatus |= 1ull << 5;     // set SPIE
task[i]->thread.sstatus |= 1ull << 18;    // set SUM    
task[i]->thread.sscratch = USER_END;
task[i]->pgd = reinterpret_cast<uint64*>(va_to_pa(reinterpret_cast<uint64>(construct_u_mode_pgtbl())));
```

### Construct U-Mode Page Table

In order to prevent page table switching while switching between U-Mode and S-Mode, we also copy kernel page table `swapper_pg_dir` to each process's page table. Keep in mind that this is a *shallow copy*, which copied the first level page table by it's value. This does not have any safety issue, because the S-mode page table remains unchanged after initialization, and all page entries can only be accessed in S-mode.  

```c++
auto* const u_mode_pgtbl = reinterpret_cast<uint64*>(kalloc());
for (auto i = 0ul; i < PGTBL_ELEMENT_COUNT; i++) {
    u_mode_pgtbl[i] = swapper_pg_dir[i];
}
```

Then, the mappings of U-mode are created. The user app are mapped with `R-W-X` permission. The user program requires reading and writing `.data` segment, since it has global variable. In this case, we `R-W-X` permission is required. The user mode stack are also mapped, with `R-W` permission.  

### Store and Restore CSRs

The store and restore process are symmetrical, so I choose to only talk about the restore process.  
Firstly, the data are restored from `thread_struct`.  

```asm
addi t0, t0, 12 * 8  # points to `thread_struct.sepc`
ld t1, 0(t0)
ld t2, 8(t0)
ld t3, 16(t0)
csrw sepc, t1
csrw sstatus, t2
csrw sscratch, t3
```

Then, the address of the first level page table of the next process is loaded. After the address has been written into `satp`, TLB is flushed to make sure the cache of the previous process will not affect the new one.  

```asm
ld t4, 24(t0)
srli t4, t4, 12
li t5, RISCV_SV39_MODE_MASK
or t4, t4, t5
csrw satp, t4
sfence.vma zero, zero
```

### Switch `sscratch` and `sp`

Since an exception is triggered in U-mode, `_trap` in S-mode start to execute to handle the exception. The problem is, U-mode and S-mode uses different stacks. As we switched into S-mode, we also need to use S-mode stack. The macro `SWITCH_SSCRATCH_SP`, providing a lossless `sp` swapping, solved this problem.  
The implementation is tricky. Since `sscract` is a CSR, it needs to be read into a register `reg_1`, and will override the original value of this register. In this case, a logical solution is backup `reg_1` into the stack. However, what we perform in this procedure is to switch `sp`, so in order to correctly access `reg_1`, we need another register `reg_2` to store the original `sp`. The implementation is shown below.  

```asm
.macro SWITCH_SSCRATCH_SP
    sd t0, -8(sp)
    sd t1, -16(sp)

    mv t1, sp

    # swap
    csrr t0, sscratch
    csrw sscratch, sp
    mv sp, t0

    ld t0, -8(t0)
    ld t1, -16(t1)
.endm
```

We need to place `SWITCH_SSCRATCH_SP` in three different locations: at the start and end of `_traps`, and in `__dummy`. But things gets a little bit complicated at the start of `_traps`. The thing is, we might encounter a exception from idle process. The idle process comes from S-mode, but will `sret` to user mode via `__dummy`. So, we need to detect the idle process, and if the process that creates the exception is idle process, then we have to switch the two stacks back. 
Then, how to detect a idle process? The thing we know is that `sscratch` is initialized to zero iff the process is a idle process. This means that we only need to check the new `sp`, whose value used to store in `sscratch` and has just swapped with the original `sp`.  

```asm
SWITCH_SSCRATCH_SP
# when `sp` is zero, it means we are in kernel mode, 
# so we should not switch between user and supervisor stack
bnez sp, _trap_continue
SWITCH_SSCRATCH_SP
```

### Access Registers Using High Level Language

I defined `pt_regs` in the following way, and it is based on the sequence registers are stored in `_traps`. I also defined a `enum struct` to add literal meaning to register indexes.  

```c++
struct pt_regs {
    uint64 x[32];
    uint64 sepc;
};
enum struct REGISTERS : uint64 {
    zero, ra, sp, gp, tp, t0, t1, t2, s0, s1, a0, a1, a2, a3, a4, a5, a6, a7,
    s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, t3, t4, t5, t6,
};
```

### System Call Handler

Firstly, I added some code, to filter system call from exceptions. Then, what I do is switching `a7`, which is the system call code. For each system call, I filled in the required amount of arguments, and put the return value into `a0`.  

```c++
switch (regs->x[static_cast<uint64>(REGISTERS::a7)]) {
    case SYS_WRITE:
        regs->x[static_cast<uint64>(REGISTERS::a0)] = sys_write(
                static_cast<uint32_t>(regs->x[static_cast<uint64>(REGISTERS::a0)]),
                reinterpret_cast<const char *>(regs->x[static_cast<uint64>(REGISTERS::a1)]),
                regs->x[static_cast<uint64>(REGISTERS::a2)]
        );
        break;
    case SYS_GETPID:
        regs->x[static_cast<uint64>(REGISTERS::a0)] = sys_getpid();
        break;
}
```

Then, we added `sepc` by 4, because since the system call has been completed, the user program needs to skip the `ecall` instruction.  

### Implementation of System Calls

`sys_write` calls `putc`, printing the required number of characters. `sys_getpid` returns the value of `pid` from the current process pointer.  

```c++
auto sys_write(uint32_t fd, const char *buf, uint64_t count) -> uint64_t {
    if (fd == 1) {
        for (uint64_t i = 0; i < count; i++) {
            putc(buf[i]);
        }
        return count;
    }
    return static_cast<uint64_t>(-1);
}
auto sys_getpid() -> uint64_t {
    return current->pid;
}
```

### Modify `head.S` and `start_kernel`

1. In `start_kernel`, put `schedule` before `test`.
2. Remove *enable interrupt sstatus.SIE* in `head.S`.



## Credit

This project is based on the work of [zjusec/os21fall](https://gitee.com/zjusec/os21fall) on gitee.  



