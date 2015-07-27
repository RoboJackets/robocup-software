MEMORY
{
  /* Define each memory region */
  RamLoc64 (rwx) : ORIGIN = 0x10000000, LENGTH = 0x10000 /* 64k */
  RamPeriph32 (rwx) : ORIGIN = 0x20000000, LENGTH = 0x8000 /* 32k */

}
  /* Define a symbol for the top of each memory region */
  __top_RamLoc64 = 0x10000000 + 0x10000;
  __top_RamPeriph32 = 0x20000000 + 0x8000;

