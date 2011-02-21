ENTRY(_start)

/* Memory Definitions */
MEMORY
{
  ROM (rx) : ORIGIN = 0x00000000, LENGTH = 32k
  RAM (rw) : ORIGIN = 0x40000000, LENGTH = 8k
}


C_STACK_SIZE = 0x800; /* see startup-file and .stack-section */
FIQ_STACK_SIZE = 0x00000004;	
IRQ_STACK_SIZE = 0X00000080;	



/* Section Definitions */
SECTIONS {
   .reset : {
	*crt0.o (.text)		  /* Startup code*/
	. = ALIGN(4);
   } >ROM
   
   .text : {
        *(.text .text.* .gnu.linkonce.t.*)
	*(.glue_7t)
	*(.glue_7) 
	*(.gcc_except_table)
	*(.rodata .rodata.* .gnu.linkonce.r.*)
	*(.init)
	*(.fini)
	. = ALIGN(4);
	_etext = .;
	PROVIDE(_etext = .);
   } >ROM
  
   /* .data section which is used for initialized data */
  .data : {
	_data = .;
	KEEP(*(.jcr))
	*(.got.plt) *(.got)
	*(.shdata)
	*(.data .data.* .gnu.linkonce.d*)
	. = ALIGN(4);
	_edata = .;
	PROVIDE(_edata = .);
	PROVIDE (_data = .);
  } >RAM AT >ROM
 
  /* .bss section which is used for uninitialized data */
   .bss : {
    	__bss_start = .;
    	*(.shbss)
    	*(.bss .bss.* .gnu.linkonce.b*)
    	*(COMMON)
    	. = ALIGN(4);
    	__bss_end = .;
    	PROVIDE (__bss_end = .);
	PROVIDE (_bss_start = .);
   } >RAM

   .stack 0x40001F78 : {
	. += IRQ_STACK_SIZE;
	. = ALIGN(4);
	__irq_stack_top__ = .;		
	
	. += FIQ_STACK_SIZE;
	. = ALIGN(4);
	__fiq_stack_top__ = .;	
	_stack = .;
	PROVIDE (_stack = .);
    } >RAM

  _end = . ;
  PROVIDE (end = .);
}

