INCLUDE "lib.ld"
INCLUDE "1857_mem.ld"

ENTRY(ResetISR)

SECTIONS
{
	
	.text_Flash2 : ALIGN(4)
	{
	   FILL(0xff)
		*(.text_Flash2*)
		*(.text_MFlashB512*)
	} > MFlashB512

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
		LONG(LOADADDR(.data_RAM5));
		LONG(    ADDR(.data_RAM5)) ;
		LONG(  SIZEOF(.data_RAM5));
		__data_section_table_end = .;
		__bss_section_table = .;
		LONG(    ADDR(.bss));
		LONG(  SIZEOF(.bss));
		LONG(    ADDR(.bss_RAM2));
		LONG(  SIZEOF(.bss_RAM2));
		LONG(    ADDR(.bss_RAM3));
		LONG(  SIZEOF(.bss_RAM3));
		LONG(    ADDR(.bss_RAM4));
		LONG(  SIZEOF(.bss_RAM4));
		LONG(    ADDR(.bss_RAM5));
		LONG(  SIZEOF(.bss_RAM5));
		__bss_section_table_end = .;
		__section_table_end = . ;
		/* End of Global Section Table */
		

		*(.after_vectors*)
		
		*(.text*)
		*(.rodata .rodata.*)
		. = ALIGN(4);
		
	} > MFlashA512

	/*
	 * for exception handling/unwind - some Newlib functions (in common
	 * with C++ and STDC++) use this. 
	 * Use KEEP so not discarded with --gc-sections
	 */
	.ARM.extab : ALIGN(4)
	{
		KEEP(*(.ARM.extab* .gnu.linkonce.armextab.*))
	} > MFlashA512
	__exidx_start = .;
	
	.ARM.exidx : ALIGN(4)
	{
		KEEP(*(.ARM.exidx* .gnu.linkonce.armexidx.*))
	} > MFlashA512
	__exidx_end = .;
	
	_etext = .;
		
	
	.data_RAM2 : ALIGN(4)
	{
	   FILL(0xff)
		*(.data.$RAM2*)
		*(.data.$RamLoc40*)
	   . = ALIGN(4) ;
	} > RamLoc40 AT>MFlashA512
	
	.data_RAM3 : ALIGN(4)
	{
	   FILL(0xff)
		*(.data.$RAM3*)
		*(.data.$RamAHB32*)
	   . = ALIGN(4) ;
	} > RamAHB32 AT>MFlashA512
	
	.data_RAM4 : ALIGN(4)
	{
	   FILL(0xff)
		*(.data.$RAM4*)
		*(.data.$RamAHB16*)
	   . = ALIGN(4) ;
	} > RamAHB16 AT>MFlashA512
	
	.data_RAM5 : ALIGN(4)
	{
	   FILL(0xff)
		*(.data.$RAM5*)
		*(.data.$RamAHB_ETB16*)
	   . = ALIGN(4) ;
	} > RamAHB_ETB16 AT>MFlashA512
	
	/* MAIN DATA SECTION */

	.uninit_RESERVED : ALIGN(4)
	{
		KEEP(*(.bss.$RESERVED*))
		. = ALIGN(4) ;
		_end_uninit_RESERVED = .;
	} > RamLoc32

	.data : ALIGN(4)
	{
		FILL(0xff)
		_data = .;
		*(vtable)
		*(.data*)
		. = ALIGN(4) ;
		_edata = .;
	} > RamLoc32 AT>MFlashA512

	
	.bss_RAM2 : ALIGN(4)
	{
		*(.bss.$RAM2*)
		*(.bss.$RamLoc40*)
	   . = ALIGN(4) ;
	} > RamLoc40
	
	.bss_RAM3 : ALIGN(4)
	{
		*(.bss.$RAM3*)
		*(.bss.$RamAHB32*)
	   . = ALIGN(4) ;
	} > RamAHB32
	
	.bss_RAM4 : ALIGN(4)
	{
		*(.bss.$RAM4*)
		*(.bss.$RamAHB16*)
	   . = ALIGN(4) ;
	} > RamAHB16
	
	.bss_RAM5 : ALIGN(4)
	{
		*(.bss.$RAM5*)
		*(.bss.$RamAHB_ETB16*)
	   . = ALIGN(4) ;
	} > RamAHB_ETB16

	/* MAIN BSS SECTION */
	.bss : ALIGN(4)
	{
		_bss = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4) ;
		_ebss = .;
		PROVIDE(end = .);
	} > RamLoc32
	
	PROVIDE(_pvHeapStart = .);
	PROVIDE(_vStackTop = __top_RamLoc32 - 0);
}

SECTIONS
{
 	/* SDRAM section */
	.bss_EMWIN (NOLOAD) : ALIGN(4)
	{
		*(.shmem_0x28050000)
	} > SDRAM
	
}


