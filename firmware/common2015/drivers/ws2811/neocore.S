############################################
###
### neocore.S
###  
### Allen Wild 
### March 2014
### 
### Modified by Jonathan Jones for GCC
### August 2015
### 
### ARM assembly functions for writing to Adafruit NeoPixels 
### with the mbed NXP LPC1768 
###
############################################

	.file   "neocore.S"
	.syntax unified

	.thumb
	.align  2
	.arch armv7
	.cpu cortex-m3

	# these 2 variables are set in the neostrip class
	.global neo_fio_reg
	.global neo_bitmask

############################################
neo_write_pin:
### Set the GPIO pin to the value passed in r0 
### Registers and bitmasks are stored in variables set by the C++ library 
	LDR		r1,  =neo_fio_reg	@ load pointers to register values 
	LDR		r2,  =neo_bitmask
	LDR		r1, [r1]			@ load actual values from memory
	LDR		r2, [r2]

	CMP		r0, 0 
	ITE 	EQ
	STREQ	r2, [r1, 0x1C]
	STRNE	r2, [r1, 0x18]
	BX		lr

############################################
neo_zero:
### Output a NeoPixel zero, composed of a short 
### HIGH pulse and a long LOW pulse 
	PUSH	{lr}
	MOV		r0,  1 
	BL		neo_write_pin	 	@ set pin high 
	
	MOV		r0,  3				@ delay long enough 
	BL		neo_delay
	
	MOV		r0,  0				@ set pin low 
	BL		neo_write_pin
	
	MOV		r0,  8				@ delay 
	BL		neo_delay
	
	POP		{lr}
	BX		lr

############################################
neo_one:
### Output a NeoPixel one, composed of a long 
### HIGH pulse and a short LOW pulse 
	PUSH	{lr}
	MOV		r0,  1
	BL		neo_write_pin 		@ set pin high
	
	MOV		r0,  10
	BL		neo_delay 			@ delay long enough
	
	MOV		r0,  0
	BL		neo_write_pin 		@ set pin low
	
	MOV 	r0, 1
	BL 		neo_delay			@ delay 
	
	POP		{lr}
	BX		lr
	

############################################
	.text
	.thumb_func
	.global neo_out
	.type   neo_out, %function
neo_out:
### void neo_out(int *data, int n)
### Main function called from the C++ library 
### r0 contains a pointer to the array of color data to send 
### r1 contains the number of bytes of data to send 
	.fnstart
	PUSH	{r4, r5, r6, r7, r8, lr}
	MOV		r7, r1		 		@ move length to %r7 
	MOV		r6, r0		 		@ move address to %r6 
neo_byteloop:
	LDRB	r5, [r6]			@ load byte to send 
	MOV		r4, 0x80	 		@ load initial bitmask 
neo_bitloop:
	AND		r3, r5, r4	 		@ mask current byte 
	CMP		r3, 0
	BNE 	neo_one_b
	BL 		neo_zero
	B 		neo_set_done
neo_one_b:
	BL 		neo_one
neo_set_done:
	LSR		r4, r4, 1	 		@ shift bitmask right one 
	CMP		r4, 0		 		@ if still more bits, loop back 
	BNE		neo_bitloop
	
	ADD		r6, r6, 1	 		@ increment address 
	SUB		r7, r7, 1	 		@ decrement count 
	CMP		r7, 0
	BNE		neo_byteloop	 	@ continue if not done 

	MOV		r0, 0
	BL		neo_write_pin
	POP		{r4, r5, r6, r7, r8, lr}
	BX		lr
	.fnend
	.size    neo_out, . - neo_out

############################################
neo_delay:
### delay the specified number of cycles in r0. A cycle
### is generalized to be a time it takes to decrement r0
### and make a comparison for branching back or not.
	SUB		r0, r0, 1	 		@ decrement count 
	CMP		r0, 0
	BNE		neo_delay	 		@ continue if not done
	BX	 	lr

.end
