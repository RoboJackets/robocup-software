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
 *      Linker script for running in internal Flash on the AT91SAM7S321.
 *----------------------------------------------------------------------------*/

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(entry)

MEMORY
{
    sram (W!RX) : ORIGIN = 0x200000, LENGTH = 0x2000
    flash (RX) : ORIGIN = 0x100000, LENGTH = 0x8000
}

SECTIONS
{  
    .fixed :
    {
        . = ALIGN(4);
        _sfixed = .;
		*(.startup)
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
		_sbuild_id = .;
		*(.note.gnu.build-id)
        . = ALIGN(4);
		_ebuild_id = .;
        _efixed = .;
    } >flash

    .relocate : AT (_efixed)
    {
        . = ALIGN(4);
        _srelocate = .;
        *(.vectors)
        *(.ramfunc)
        *(.data)
		
		/* Reflashing code is stored in flash after relocate when not in use, but will be
		* copied to SRAM.  It doesn't actually matter where in SRAM this goes
		* as long as it doesn't hit the stack.  When reflash runs, the rest
		* of the system is unrecoverable and interrupts are off so collisions
		* (except for the stack) are irrelevant.
		* 
		* FIXME - This keeps reflash in RAM all the time.  I want it to overlap data/bss
		* so we don't waste space when not reflashing.  Can you force ld to overlap?
		*/
		*(.reflash)
		
        . = ALIGN(4);
        _erelocate = .;
    } >sram

/*
	.reflash : AT(_efixed + _erelocate - _srelocate)
	{
        . = ALIGN(4);
		_sreflash = .;
		*(.reflash)
		. = ALIGN(4);
		_ereflash = .;
	} >sram
	*/
	
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

