/* Linker script to configure memory regions. */
MEMORY
{
  TCM (rwx) : ORIGIN = 0x6013F000, LENGTH = 4K /* TCM second last 4K */
}

SECTIONS
{
    .text :
    {
        _stext = ABSOLUTE(.);

        KEEP(*(.vectors))

        . = 0x100;
        /* image magic */
        LONG(0x21494350)
        LONG(_stext)
        LONG(_etcm - _stext)
        LONG(reset_handler)

        . = ALIGN(4);
        *(.text*)
        __CTOR_LIST__ = .;
        LONG((__CTOR_END__ - __CTOR_LIST__) / 4 - 2)
        *(.ctors)
        LONG(0)
        __CTOR_END__ = .;
        __DTOR_LIST__ = .;
        LONG((__DTOR_END__ - __DTOR_LIST__) / 4 - 2)
        *(.dtors)
        LONG(0)
        __DTOR_END__ = .;
        *(.lit)
        *(.shdata)
    } > TCM

    .rodata : {
        . = ALIGN(4);
        *(.rodata*);
    } > TCM

    .data :
    {
        . = ALIGN(4);
        data_start = .;
        *(.data*)
        *(.sdata*)
        data_end = .;
    } > TCM

    _etcm = ABSOLUTE(.);

    data_load_start = LOADADDR(.data);

    .bss :
    {
        . = ALIGN(4);
        bss_start = .;
        *(.bss*)
        *(.sbss*)
        *(COMMON)
        bss_end = .;
    } > TCM

    . = ALIGN(4);

    stack_top = ORIGIN(TCM) + LENGTH(TCM);
    PROVIDE(__stack = stack_top);
}
