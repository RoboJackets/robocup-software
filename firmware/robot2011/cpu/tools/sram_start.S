		.section .startup
		.global entry
entry:
		/* SAM-BA doesn't expect to regain control, but fixing the stack makes it work
		 * (the USB RX bank has to be correct too).
		 */
		add sp, sp, #0x60
		ldr r0, total_size
		ldr r1, =reflash_main
		bx r1

total_size:	.word 4096
