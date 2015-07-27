/*
 * Common lpc43xx dual core linker descriptor file
 */

ENTRY(ResetISR)

SECTIONS
{
	/* MAIN TEXT SECTION */	
	.text : ALIGN(4)
	{
		FILL(0xff)
		KEEP(*(.isr_vector))
		
		/* Global Section Table */
		. = ALIGN(4) ;
		__section_table_start = .;
		__data_section_table = .;
		LONG(LOADADDR(.data));
		LONG(    ADDR(.data)) ;
		LONG(  SIZEOF(.data));
		LONG(LOADADDR(.data_RAM2));
		LONG(    ADDR(.data_RAM2)) ;
		LONG(  SIZEOF(.data_RAM2));
		LONG(LOADADDR(.data_RAM3));
		LONG(    ADDR(.data_RAM3)) ;
		LONG(  SIZEOF(.data_RAM3));
		LONG(LOADADDR(.data_RAM4));
		LONG(    ADDR(.data_RAM4)) ;
		LONG(  SIZEOF(.data_RAM4));
		__data_section_table_end = .;
		__bss_section_table = .;
		LONG(    ADDR(.bss));
		LONG(  SIZEOF(.bss));
		__bss_section_end = .;
		LONG(    ADDR(.bss_ETH));
		LONG(  SIZEOF(.bss_ETH));
		LONG(    ADDR(.bss_SHMEM));
		LONG(  SIZEOF(.bss_SHMEM));
		LONG(    ADDR(.bss_RAM4));
		LONG(  SIZEOF(.bss_RAM4));
		__bss_section_table_end = .;
		__section_table_end = . ;
		/* End of Global Section Table */

		*(.after_vectors*)
		
		*(.text*)
		*(.rodata .rodata.*)
		. = ALIGN(4);
		
	} > RO_MEM

	/*
	 * for exception handling/unwind - some Newlib functions (in common
	 * with C++ and STDC++) use this. 
	 * Use KEEP so not discarded with --gc-sections
	 */
	.ARM.extab : ALIGN(4)
	{
		KEEP(*(.ARM.extab* .gnu.linkonce.armextab.*))
	} > RO_MEM
	__exidx_start = .;
	
	.ARM.exidx : ALIGN(4)
	{
		KEEP(*(.ARM.exidx* .gnu.linkonce.armexidx.*))
	} > RO_MEM
	__exidx_end = .;
	
	_etext = .;
		
	
	.data_RAM2 : ALIGN(4)
	{
	   FILL(0xff)
		*(.data.$RAM2*)
		*(.data.$RamLoc40*)
	   . = ALIGN(4) ;
	} > RW_MEM AT>RO_MEM
	
	.data_RAM3 : ALIGN(4)
	{
	   FILL(0xff)
		*(.data.$RAM3*)
		*(.data.$RamAHB32*)
	   . = ALIGN(4) ;
	} > RW_MEM1 AT>RO_MEM
	
	.data_RAM4 : ALIGN(4)
	{
	   FILL(0xff)
		*(.data.$RAM4*)
		*(.data.$RamAHB16*)
	   . = ALIGN(4) ;
	} > FAT12_MEM AT>RO_MEM
	
	/* MAIN DATA SECTION */

	.uninit_RESERVED : ALIGN(4)
	{
		KEEP(*(.bss.$RESERVED*))
		. = ALIGN(4) ;
		_end_uninit_RESERVED = .;
	} > RW_MEM

	.data : ALIGN(4)
	{
		FILL(0xff)
		_data = .;
		*(vtable)
		*(.data*)
		. = ALIGN(4) ;
		_edata = .;
	} > RW_MEM AT>RO_MEM

	/* MAIN BSS SECTION */
	.bss : ALIGN(4)
	{
		_bss = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4) ;
		*(.bss.$RAM2*)
		*(.bss.$RAM3*)
		*(.bss.$RAM4*)
		_ebss = .;
		end = .;
		KEEP(*(STACK));
		_vStackTop = .;
	} > RW_MEM

	.bss_SHMEM : ALIGN(4)
	{
		*(.shmem_0x20000000)
		*(.shmem_0x20000020)

		*(.bss.$RamAHB32*)
	   . = ALIGN(4) ;
	} > SH_MEM
	
	.bss_RAM4 : ALIGN(4)
	{
		*(.shmem_0x20002000)
	   . = ALIGN(4) ;
	} > FAT12_MEM
	
	.bss_ETH : ALIGN(4)
	{
		/* Required if lwip now using malloc */
		mem*.o(.bss*)

		/* Keep the heap in AHB RAM area */
		_pvHeapStart = .;
		KEEP(*(HEAP));
	} > RW_MEM1 
	
}
