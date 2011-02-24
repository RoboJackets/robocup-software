.set Stack_Size, 512	

.set FIQ_STACK_SIZE, 0x00000004	
.set IRQ_STACK_SIZE, 0X00000080	

.set MODE_USR, 0x10
.set MODE_FIQ, 0x11
.set MODE_IRQ, 0x12
.set MODE_SYS, 0x1F           

.equ I_BIT, 0x80               /* when I bit is set, IRQ is disabled*/
.equ F_BIT, 0x40               /* when F bit is set, FIQ is disabled*/

.text

                .globl _start
_start:         // Interrupt vector table
                ldr pc, reset_addr      // Reset
                ldr pc, reset_addr      // Undefined instruction
                ldr pc, reset_addr      // Software interrupt
                ldr pc, reset_addr      // Prefetch abort
                ldr pc, reset_addr      // Data abort
                .word 0xb9205fc0        // Reserved (vector checksum goes here)
                ldr pc, [pc,#-0xFF0]    // Interrupt Auto Vectoring
                ldr pc, fiq_addr        // FIQ

reset_addr:     .word init
fiq_addr:       .word init

		
init:           // This is the entry point for a reset or unrecoverable event.

                // Copy data from ROM to RAM
                ldr r1, =_etext
                ldr r2, =_data
                ldr r3, =_edata
1:              cmp r2, r3
                ldrlo r0, [r1], #4
                strlo r0, [r2], #4
                blo 1b

                // Clear BSS
                mov r0, #0
                ldr r1, =__bss_start
                ldr r2, =__bss_end
2:              cmp r1, r2
                strlo r0, [r1], #4
                blo 2b

/* Initialize Interrupt System*/
/* - Set stack location for each mode*/
/* - Leave in System Mode with Interrupts Disabled*/
/* Linker sets _stack at top of stack because stack is assumed to grow down */
                ldr r0, =_stack
                msr CPSR_c, #MODE_FIQ|I_BIT|F_BIT 
                mov sp, r0
                sub r0, r0, #FIQ_STACK_SIZE
                msr CPSR_c, #MODE_IRQ|I_BIT|F_BIT 
                mov sp, r0
                sub r0, r0, #IRQ_STACK_SIZE
                msr CPSR_c, #MODE_SYS|I_BIT|F_BIT 
                mov sp, r0
		
                // Enable interrupts
                msr CPSR_c, #MODE_SYS|F_BIT
				
                b main
