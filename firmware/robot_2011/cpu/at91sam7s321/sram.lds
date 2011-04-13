/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*------------------------------------------------------------------------------
 *      Linker script for running in internal SRAM on the AT91SAM7S64.
 *----------------------------------------------------------------------------*/

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(entry)

MEMORY
{
	/* Interrupt vectors must be at the beginning of SRAM, but we can't write
	 * there while SAM-BA is running so the code must be copied during
	 * our initialization.
	 */
    low_sram (W!RX) : ORIGIN = 0x200000, LENGTH = 0x1400

    /* This is where our code will be loaded and start executing */
    sram (W!RX) : ORIGIN = 0x201400, LENGTH = 0x800

    flash (RX) : ORIGIN = 0x100000, LENGTH = 0x10000
}

SECTIONS
{
    . = ALIGN(4);
    .fixed :
    {
        _sfixed = .;
		*(.startup)
        *(.text*)
		*(.reflash)
        *(.ramfunc)
        *(.rodata*)
        *(.data)
        . = ALIGN(4);
        _efixed = .;
    } >sram

    /DISCARD/ : { *(.note.gnu.build-id) }
	
    .relocate : AT (_efixed)
    {
        . = ALIGN(4);
        _srelocate = .;
        *(.vectors)
        . = ALIGN(4);
        _erelocate = .;
    } >low_sram
    
    .bss (NOLOAD) : {
        _szero = .;
        *(.bss)
		*(COMMON)
        . = ALIGN(4);
        _ezero = .;
    } >sram
    
    _sstack = 0x202000;
}
end = .;

