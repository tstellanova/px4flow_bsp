/* Memory layout for stm32f407 */
MEMORY
{
  /* NOTE K = 1024 bytes */
  /* FLASH and RAM are mandatory memory regions */

  /* NOTE flash actually starts at 0x08000000
  but we reserve space for the default PX4 bootloader */
  BLFLASH :   ORIGIN = 0x08000000, LENGTH = 16K
  FLASH  :    ORIGIN = 0x08004000, LENGTH = 1008K
  CCMRAM :    ORIGIN = 0x10000000, LENGTH = 64K
  RAM   :     ORIGIN = 0x20000000, LENGTH = 128K


}

SECTIONS
{
    .ccmram (NOLOAD) : ALIGN(4)
    {
        *(.ccmram .ccmram.*);
        . = ALIGN(4);
    } > CCMRAM
} INSERT AFTER .bss;

/*
This is where the call stack will be allocated.
The stack is of the full descending type.
Place the stack at the end of RAM.
*/
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* The location of the .text section can be overridden using the
   `_stext` symbol.  By default it will place after .vector_table */
/* _stext = ORIGIN(FLASH) + 0x40c; */

