############################################
#
# neocore.s 
#  
# Allen Wild 
# March 2014
# 
# Modified by Jonathan Jones for GCC
# August 2015
# 
# ARM assembly functions for writing to Adafruit NeoPixels 
# with the mbed NXP LPC1768 
#
############################################
	.text {a w x}

	.thumb
	
	.global neo_fio_reg
	.global neo_bitmask

.globl XPUT32
XPUT32:
    push {%lr}
    # @ call an arm function from thumb asm
    ldr %r2,=PUT32
    mov %lr,pc
    bx %r2
    pop {%r2}
    bx %r2
	
############################################
.thumb_func
neo_write_pin:
# Set the GPIO pin to the value passed in %r0 
# Registers and bitmasks are stored in variables set by the C++ library 
	LDR		%r1, =neo_fio_reg	@ load pointers to register values 
	LDR		%r2, =neo_bitmask
	LDR		%r1, [%r1]			@ load actual values from memory 
	LDR		%r2, [%r2]

	CMP		%r0, #0				@ VALUE == 0 ? 
	ITE EQ					 	@ (IF-THEN-ELSE) ON NEXT TWO INSTRUCTIONS USING "EQ" FLAG 
	STREQ	%r2, [%r1, #0x1C]	@ if==0, CLEAR BIT 
	STRNE	%r2, [%r1, #0x18]	@ if==1, SET BIT 
	BX		%lr

############################################
neo_zero:
# Output a NeoPixel zero, composed of a short 
# HIGH pulse and a long LOW pulse 
	PUSH	{%lr}
	MOV		%r0,  #1 
	BL		neo_write_pin	 	@ set pin high 
	
	MOV		%r0,  #10			@ delay for long enough 
	BL		neo_delay
	
	MOV		%r0,  #0			@ set pin low 
	BL		neo_write_pin
	
	MOV		%r0,  #20			@ delay 
	BL		neo_delay
	
	POP		{%lr}
	BX		%lr

############################################
neo_one:
# Output a NeoPixel one, composed of a long 
# HIGH pulse and a short LOW pulse 
	PUSH	{%lr}
	MOV		%r0,  #1 
	BL		neo_write_pin
	
	MOV		%r0,  #86 
	BL		neo_delay
	
	MOV		%r0,  #0 
	BL		neo_write_pin
	
	NOP		 				@ really short delay 
	NOP
	NOP
	
	POP		{%lr}
	BX		%lr
	
############################################
# void neo_out(int *data, int n)
# Main function called from the C++ library 
# %r0 contains a pointer to the array of color data to send 
# %r1 contains the number of bytes of data to send 
neo_out:
	PUSH	{%lr, %r4, %r5, %r6, %r7, %r8}
	MOV		%r7, %r1		 	@ move length to %r7 
	MOV		%r6, %r0		 	@ move address to %r6 
	
neo_byteloop:
	LDRB	%r5, [%r6]			@ load byte to send 
	MOV		%r4, #0x80	 		@ load initial bitmask 
	
neo_bitloop:
	AND		%r3, %r5, %r4	 	@ mask current byte 
	CMP		%r3, #0
	BLEQ	neo_zero	 		@ send current bit 
	BLNE	neo_one
	
	LSR		%r4, %r4, #1	 	@ shift bitmask right one 
	CMP		%r4, #0		 		@ if still more bits, loop back 
	BNE		neo_bitloop
	
	ADD		%r6, %r6, #1	 	@ increment address 
	SUB		%r7, %r7, #1	 	@ decrement count 
	CMP		%r7, #0
	BNE		neo_byteloop	 	@ continue if not done 

	MOV		%r0, #0
	BL		neo_write_pin
	POP		{%r8, %r7, %r6, %r5, %r4, %lr}
	BX		%lr

############################################
neo_delay:
# delay the specified number of cycles in %r0 with a bunch of nops 
	LDR		%r2, =neo_delay_end
	SUB		%r2, %r2, %r0
	BX		%r2
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP

neo_delay_end:
	BX	 %lr
	end 	@ end code region 
