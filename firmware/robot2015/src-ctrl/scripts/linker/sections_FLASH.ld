/* Linker script for mbed LPC1768 */

/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 512K
  RAM (rwx) : ORIGIN = 0x100000C8, LENGTH = (32K - 0xC8)

  USB_RAM(rwx) : ORIGIN = 0x2007C000, LENGTH = 16K
  ETH_RAM(rwx) : ORIGIN = 0x20080000, LENGTH = 16K
}

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 * 
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 */
ENTRY(Reset_Handler)

SECTIONS
{
    /*
     * The '.text' section is where the executable instructions of the program are stored.
     */
    .text :
    {
        KEEP(*(.isr_vector))

        /* ===== Parameters/TOC Logging ===== */
        . = ALIGN(4);                   /* Align to the nearest word-aligned address */
        PROVIDE (_param_start = .);     /* Set the address here to the '_param_start' variable. Define it if undefined. */
        KEEP(*(.param))                 /* Place variables set with a section attribute of '*.param' here */
        KEEP(*(.param.*))               /* Place variables set with a section attribute of '*.param.*' here */
        PROVIDE (_param_stop = .);      /* Set the address here to the '_param_stop' variable. Define it if undefined. */
        . = ALIGN(4);                   /* Align to the nearest word-aligned address */
        PROVIDE (_log_start = .);       /* Set the address here to the '_log_start' variable. Define it if undefined. */
        KEEP(*(.log))                   /* Place variables set with a section attribute of '*.log' here */
        KEEP(*(.log.*))                 /* Place variables set with a section attribute of '*.log.*' here */
        PROVIDE (_log_stop = .);        /* Set the address here to the '_log_stop' variable. Define it if undefined. */

        *(.text*)

        KEEP(*(.init))
        KEEP(*(.fini))


        /* ===== Constructors/Deconstructors ===== */
        /* .ctors (constructors) */
        PROVIDE (__ctors_start__ = .);
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)
        PROVIDE (__ctors_end__ = .);

        /* .dtors (deconstructors) */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)


        /* ===== Read-only Data ===== */
        . = ALIGN(4);                   /* Align to the nearest word-aligned address */
        *(.rodata*)


        /* ===== Exception Handling ===== */
        /* The '.eh_frame' section contains 1 or more 'Call Frame Information' records
         * that describes the call frames that must be unwound during the processing of
         * an exception. It shall be aligned to an addressing unit sized boundary.
         */
        . = ALIGN(4);
        KEEP(*(.eh_frame*))

    } > FLASH   /* Assign the '.text' section to the 'FLASH' memory region. */


    /* The '.ARM.extab' section contains exception unwinding information. */
    .ARM.extab : 
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH   /* Place in the 'FLASH' memory region. */


    /* The '.ARM.exidx' section contains a sorted list of 'key=>value' pairs for indexing section unwinding. */
    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH   /* Place in the 'FLASH' memory region. */
    __exidx_end = .;


    /* Get the address for the end of the '.text' section. */
    __etext = .;
        

    .data : AT (__etext)    /* Load the '.data' section starting at the address set for '__etext'. */
    {
        __data_start__ = .; /* Store the address for the start of the '.data' section. */
        Image$$RW_IRAM1$$Base = .;
        *(vtable)
        *(.data*)
        

        /* 
         * ===== preinit data =====
         * This section holds an array of function pointers that contributes to a
         * single pre-initialization array for the executable or shared object containing
         * the section.
         */
        . = ALIGN(4);
        PROVIDE (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE (__preinit_array_end = .);


        /* 
         * ===== init data =====
         * This section holds an array of function pointers that contributes to a
         * single initialization array for the executable or shared object containing
         * the section.
         */
        . = ALIGN(4);
        PROVIDE (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE (__init_array_end = .);


        /* 
         * ===== finit data =====
         * This section holds an array of function pointers that contributes to a
         * single termination array for the executable or shared object containing
         * the section.
         */
        . = ALIGN(4);
        PROVIDE (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE (__fini_array_end = .);


        /* All data end */
        . = ALIGN(4);
        __data_end__ = .;

    } > RAM     /* Place in the 'RAM' memory region. */

    
    .bss :
    {
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        __bss_end__ = .;
        Image$$RW_IRAM1$$ZI$$Limit = . ;
    } > RAM     /* Place in the 'RAM' memory region. */

    
    .heap :
    {
        __end__ = .;
        end = __end__;
        *(.heap*)
        __HeapLimit = .;
    } > RAM     /* Place in the 'RAM' memory region. */


    /*
     * The '.stack_dummy' section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later.
     */
    .stack_dummy :
    {
        *(.stack)
    } > RAM     /* Place in the 'RAM' memory region. */


    /* 
     * Set stack top to end of 'RAM' region, and stack limit move down by
     * size of the '.stack_dummy' section.
     */
    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);
    

    /* Check if '.data' + '.heap' + '.stack' exceeds the 'RAM' region's limit. */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")


    /* 
     * Code can explicitly ask for data to be placed in these higher 
     * RAM banks where they will be left uninitialized. 
     */
    .AHBSRAM0 (NOLOAD):
    {
        Image$$RW_IRAM2$$Base = . ;
        *(AHBSRAM0)
        Image$$RW_IRAM2$$ZI$$Limit = .;
    } > USB_RAM     /* Place in the 'USB_RAM' memory region. */

    .AHBSRAM1 (NOLOAD):
    {
        Image$$RW_IRAM3$$Base = . ;
        *(AHBSRAM1)
        Image$$RW_IRAM3$$ZI$$Limit = .;
    } > ETH_RAM     /* Place in the 'ETH_RAM' memory region. */
}
