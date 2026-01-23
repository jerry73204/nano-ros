/* Linker script for STM32F429ZI (NUCLEO-F429ZI board) */
/* 2MB Flash, 256KB SRAM (192KB + 64KB CCM) */

MEMORY
{
    FLASH  : ORIGIN = 0x08000000, LENGTH = 2048K
    RAM    : ORIGIN = 0x20000000, LENGTH = 192K
    CCMRAM : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Ethernet DMA descriptors must be in non-CCM RAM */
/* Place them at the end of main RAM */
_sethram = ORIGIN(RAM) + LENGTH(RAM) - 16K;

SECTIONS
{
    /* Ethernet DMA descriptors and buffers */
    .ethram (NOLOAD) : ALIGN(4)
    {
        *(.ethram .ethram.*);
    } > RAM
}
