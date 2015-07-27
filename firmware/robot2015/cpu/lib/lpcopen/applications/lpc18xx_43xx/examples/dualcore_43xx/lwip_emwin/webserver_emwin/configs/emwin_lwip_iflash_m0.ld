/*
 * LPC43XX Dual core LWIP Webserver & emWin stand-alone Cortex-M0 LD script
*/

MEMORY
{
  /* Define each memory region */
  RO_MEM (rx) : ORIGIN = 0x1a040000, LENGTH = 0x40000 /* 256K */
  RW_MEM (rwx) : ORIGIN = 0x10080000, LENGTH = 0x8000 /* 32k */
  RW_MEM1 (rwx) : ORIGIN = 0x20008000, LENGTH = 0x8000 /* 32K */
  SH_MEM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x2000 /* 8k */
  FAT12_MEM (rwx) : ORIGIN = 0x20002000, LENGTH = 0x2000 /* 8k */
  SDRAM_MEM (rwx) : ORIGIN = 0x28000000, LENGTH = 0x1000000 /* 16M */
}

  __top_RW_MEM = 0x10080000 + 0x8000;

INCLUDE "lpc43xx_dualcore_lib.ld"
INCLUDE "lpc43xx_dualcore.ld"

SECTIONS
{
 	/* SDRAM section */
 	.bss_LCDBUF (NOLOAD) : ALIGN(4)
	{
		*(.shmem_0x28000000)
	} > SDRAM_MEM
	
	.bss_EMWIN (NOLOAD) : ALIGN(4)
	{
		*(.shmem_0x28050000)
	} > SDRAM_MEM
	
}

