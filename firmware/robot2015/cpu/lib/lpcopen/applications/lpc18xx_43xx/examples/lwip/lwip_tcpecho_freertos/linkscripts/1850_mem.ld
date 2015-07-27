/*
 * GENERATED FILE - DO NOT EDIT
 * (C) Code Red Technologies Ltd, 2008-2012
 * Linker script memory definitions
 * Created from LinkMemoryTemplate
 * By LPCXpresso v4.3.0 [Build 1023] [2012-09-19]  on Tue Oct 30 13:28:17 ICT 2012)
*/

MEMORY
{
  /* Define each memory region */
  RamLoc96 (rwx) : ORIGIN = 0x10000000, LENGTH = 0x18000 /* 96k */
  RamLoc40 (rwx) : ORIGIN = 0x10080000, LENGTH = 0xa000 /* 40k */
  RamAHB32 (rwx) : ORIGIN = 0x20000000, LENGTH = 0x8000 /* 32k */
  RamAHB16 (rwx) : ORIGIN = 0x20008000, LENGTH = 0x4000 /* 16k */
  RamAHB_ETB16 (rwx) : ORIGIN = 0x2000c000, LENGTH = 0x4000 /* 16k */
  RO_MEM (rx) : ORIGIN = 0x14000000, LENGTH = 0x40000 /* 256K */
  
}
  /* Define a symbol for the top of each memory region */
  __top_RamLoc96 = 0x10000000 + 0x18000;
  __top_RamLoc40 = 0x10080000 + 0xa000;
  __top_RamAHB32 = 0x20000000 + 0x8000;
  __top_RamAHB16 = 0x20008000 + 0x4000;
  __top_RamAHB_ETB16 = 0x2000c000 + 0x4000;
  __top_RO_MEM = 0x14000000 + 0x40000;