;create interrupt vector table for C6000 DSP
;--------------------------------------------------------------
;This file can be modified to add Interrupt Service Routine(ISR) 
;for an interrupt, the steps are:
;1,reference to the externally defined ISR, for example
;	.ref EDMA_ISR
;2,modify the corresponding entry in the interrupt vector table.
;  For example, if interrupt 8 is used for EDMA, then you should
;  modify the entry number 8 like below:
;	VEC_ENTRY EDMA_ISR     	;interrupt 8 
;--------------------------------------------------------------
;Author: Brighton Feng
;Created on 2010-12-6
;--------------------------------------------------------------

;reference to the externally defined ISR
	.ref _c_int00
	.ref GE_Message_ISR
	.ref GE_MISC_MDIO_ISR
	.ref Exception_service_routine
	.ref Nested_Exception_service_routine
	.ref exception_record
	.global vectors 

;--------------------------------------------------------------
	.sect ".text"
;create interrupt vector for NMI	
NMI_ISR:
	STW 	B1,*-B15[1]

	;save some key registers when exception happens
	MVKL  exception_record,B1
	MVKH  exception_record,B1

	STW 	B3, *+B1[0]
	STW 	A4, *+B1[1]
	STW 	B4, *+B1[2]
	STW 	B14, *+B1[3]
	STW 	B15, *+B1[4]
	
	;jump to exception service routine
	MVKL  	Exception_service_routine, B1
	MVKH  	Exception_service_routine, B1
	B 	B1

	LDW 	*-B15[1],B1
	NOP 	4

;--------------------------------------------------------------
;create interrupt vector for reset (interrupt 0)
VEC_RESET .macro addr
	MVKL  addr,B0
	MVKH  addr,B0
	B     B0
	MVC   PCE1,B0
	NOP   4
	.align 32
	.endm

;create interrupt vector for other used interrupts	
VEC_ENTRY .macro addr
	STW   B0,*--B15
	MVKL  addr,B0
	MVKH  addr,B0
	B     B0
	LDW   *B15++,B0
	NOP   4
	.align 32
	.endm

;create interrupt vector for unused interrupts	
VEC_DUMMY .macro
unused_int?:
	B    unused_int? ;dead loop for unused interrupts
	NOP  5
	.align 32
	.endm

	
;--------------------------------------------------------------
;interrupt vector table	
	.sect "vecs"
	.align 1024
	
vectors:
	VEC_RESET Nested_Exception_service_routine     		;Nested exception  
	VEC_ENTRY NMI_ISR	;NMI/Exception
	VEC_DUMMY   		;RSVD
	VEC_DUMMY   		;RSVD
	VEC_DUMMY			;interrupt 4
	VEC_DUMMY			;interrupt 5
	VEC_DUMMY   		;interrupt 6
	VEC_ENTRY GE_Message_ISR   		;interrupt 7
	VEC_DUMMY     		;interrupt 8 
	VEC_DUMMY   		;interrupt 9
	VEC_DUMMY   		;interrupt 10
	VEC_DUMMY   		;interrupt 11
	VEC_DUMMY   		;interrupt 12
	VEC_DUMMY   		;interrupt 13
	VEC_DUMMY   		;interrupt 14
	VEC_DUMMY   		;interrupt 15
	
	.end
	
