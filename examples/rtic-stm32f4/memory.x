/* Linker script for STM32F429ZI (NUCLEO-F429ZI) */
/* 2MB Flash, 256KB RAM */

MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 2M
  RAM   : ORIGIN = 0x20000000, LENGTH = 192K
  CCMRAM : ORIGIN = 0x10000000, LENGTH = 64K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
