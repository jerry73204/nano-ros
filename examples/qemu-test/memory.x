/* Memory layout for LM3S6965 (QEMU Cortex-M3 target) */
MEMORY
{
    /* Flash: 256K */
    FLASH : ORIGIN = 0x00000000, LENGTH = 256K
    /* RAM: 64K */
    RAM : ORIGIN = 0x20000000, LENGTH = 64K
}
