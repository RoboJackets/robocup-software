		PUBLIC  PendSV_Handler
		IMPORT  OS_CPU_PendSVHandler

;********************************************************************************************************
;                                     CODE GENERATION DIRECTIVES
;********************************************************************************************************
        RSEG CODE:CODE:NOROOT(2)
        THUMB

PendSV_Handler
	LDR R0, =OS_CPU_PendSVHandler
	BX  R0
    END
