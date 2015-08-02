;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; NeoCore.s
; 
; Allen Wild
; March 2014
;
; ARM assembly functions for writing to Adafruit NeoPixels
; with the mbed NXP LPC1768
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	AREA neo_core, CODE, READONLY
	
	IMPORT neo_fio_reg
	IMPORT neo_bitmask
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
neo_write_pin
; Set the GPIO pin to the value passed in R0
; Registers and bitmasks are stored in variables set by the C++ library
	LDR		R1, =neo_fio_reg	; load pointers to register values
	LDR		R2, =neo_bitmask
	LDR		R1, [R1]			; load actual values from memory
	LDR		R2, [R2]

	CMP		R0, #0			; VALUE == 0 ?
	ITE EQ					; (IF-THEN-ELSE) ON NEXT TWO INSTRUCTIONS USING "EQ" FLAG
	STREQ	R2, [R1,#0x1C]	; if==0, CLEAR BIT
	STRNE	R2, [R1,#0x18]	; if==1, SET BIT
	BX		LR

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
neo_zero
; Output a NeoPixel zero, composed of a short
; HIGH pulse and a long LOW pulse
	PUSH	{LR}
	MOV		R0, #1
	BL		neo_write_pin	; set pin high
	
	MOV		R0, #10			; delay for long enough
	BL		neo_delay
	
	MOV		R0, #0			; set pin low
	BL		neo_write_pin
	
	MOV		R0, #20			; delay
	BL		neo_delay
	
	POP		{LR}
	BX		LR

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
neo_one
; Output a NeoPixel one, composed of a long
; HIGH pulse and a short LOW pulse
	PUSH	{LR}
	MOV		R0, #1
	BL		neo_write_pin
	
	MOV		R0, #86
	BL		neo_delay
	
	MOV		R0, #0
	BL		neo_write_pin
	
	NOP		; really short delay
	NOP
	NOP
	
	POP		{LR}
	BX		LR
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	EXPORT neo_out ; void neo_out(int *data, int n);
; Main function called from the C++ library
; R0 contains a pointer to the array of color data to send
; R1 contains the number of bytes of data to send
neo_out
	PUSH	{LR, R4, R5, R6, R7, R8}
	MOV		R7, R1		; move length to R7
	MOV		R6, R0		; move address to R6
	
neo_byteloop
	LDRB	R5, [R6]	; load byte to send
	MOV		R4, #0x80	; load initial bitmask
	
neo_bitloop
	AND		R3, R5, R4	; mask current byte
	CMP		R3, #0
	BLEQ	neo_zero	; send current bit
	BLNE	neo_one
	
	LSR		R4, R4, #1	; shift bitmask right one
	CMP		R4, #0		; if still more bits, loop back
	BNE		neo_bitloop
	
	ADD		R6, R6, #1	; increment address
	SUB		R7, R7, #1	; decrement count
	CMP		R7, #0
	BNE		neo_byteloop	; continue if not done

	MOV		R0, #0
	BL		neo_write_pin
	POP		{R8, R7, R6, R5, R4, LR}
	BX		LR

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 
neo_delay
; delay the specified number of cycles in R0 with a bunch of nops
	LDR		R2, =neo_delay_end
	SUB		R2, R2, R0
	BX		R2
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
neo_delay_end
	BX	 LR

	END ; end code region
