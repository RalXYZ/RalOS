/* target architecture */
OUTPUT_ARCH( "riscv" )

/* program entry */
ENTRY( _start )

/* starting address of kernel assembly */
BASE_ADDR = 0x80200000;

SECTIONS
{
    /* . means current address */
    . = BASE_ADDR;

    /* store the starting address of kernel asm */
    _skernel = .;

    /* ALIGN(0x1000) means align to 4KB */
    /* _stext, _etext stores the starting and ending address of .text segment respectively */
    .text : ALIGN(0x1000){
        _stext = .;

        *(.text.init)
        *(.text.entry)
        *(.text .text.*)
        
        _etext = .;
    }

    .rodata : ALIGN(0x1000){
        _srodata = .;

        *(.rodata .rodata.*)
        
        _erodata = .;
    }

    .data : ALIGN(0x1000){
        _sdata = .;

        *(.data .data.*)
        
        _edata = .;
    }

    .bss : ALIGN(0x1000){
        _sbss = .;

        *(.bss.stack)
        *(.sbss .sbss.*)
        *(.bss .bss.*)
        
        _ebss = .;
    }

    /* store the ending address of kernel asm */
    _ekernel = .;
}
