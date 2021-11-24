#include "defs.h"

/* target architecture */
OUTPUT_ARCH( "riscv" )

/* program entrance */
ENTRY( _start )

MEMORY {
    ram  (wxa!ri): ORIGIN = PHY_START + OPENSBI_SIZE, LENGTH = PHY_SIZE - OPENSBI_SIZE
    ramv (wxa!ri): ORIGIN = VM_START + OPENSBI_SIZE, LENGTH = VM_SIZE - OPENSBI_SIZE
}

/* the starting address of kernel code */
BASE_ADDR = VM_START + OPENSBI_SIZE;

SECTIONS
{
    /* . represents the current address */
    . = BASE_ADDR;

    /* store the starting address of kernel code */
    _skernel = .;

    /* ALIGN(0x1000) means align to 4KB */
    /* _stext, _etext stores the starting and ending address of text segment respectively */
    .text : ALIGN(0x1000){
        _stext = .;

        *(.text.init)
        *(.text.entry)
        *(.text .text.*)
        
        _etext = .;
    } >ramv AT>ram

    .rodata : ALIGN(0x1000){
        _srodata = .;

        *(.srodata .srodata.*)
        *(.rodata .rodata.*)
        
        _erodata = .;
    } >ramv AT>ram

    .data : ALIGN(0x1000){
        _sdata = .;

        *(.sdata .sdata*)
        *(.data .data.*)
        
        _edata = .;
    } >ramv AT>ram

    .bss : ALIGN(0x1000){
        _sbss = .;

        *(.bss.stack)
        *(.sbss .sbss.*)
        *(.bss .bss.*)
        
        _ebss = .;
    } >ramv AT>ram

    /* stores the ending address of kernel code */
    _ekernel = .;
} 
