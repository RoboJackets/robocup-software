/*
 * GENERATED FILE - DO NOT EDIT
 * (C) Code Red Technologies Ltd, 2008-2012
 * Linker script memory definitions
 * Created from LinkMemoryTemplate
 * By LPCXpresso v4.3.0 [Build 1023] [2012-09-19]  on Mon Oct 29 10:18:09 ICT 2012)
*/

MEMORY
{
  /* Define each memory region */
  RW_MEM (rwx) : ORIGIN = 0x10090000, LENGTH = 0x2000 /* 8k */
  RO_MEM (rx) : ORIGIN = 0x14000000, LENGTH = 0x40000 /* 256K */

}
  /* Define a symbol for the top of each memory region */
  __top_RW_MEM = 0x10090000 + 0x2000;
  __top_RO_MEM = 0x14000000 + 0x40000;
  
