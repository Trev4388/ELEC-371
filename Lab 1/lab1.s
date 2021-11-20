.text
.global _start
.org 0x0000

_start:
main:
	
	movi	sp, 0x7FFC
	
	movi	r3, LIST
	movi	r5, 1
	movi	r6, 3
	
	call InRange

	break 

#----------------------------------------------------------------------------------
	
PrintChar:
	subi	sp, sp, 12
	stw 	ra, 8(sp)
	stw	r3, 4(sp)
	stw	r4, 0(sp)
	
	movia	r3, 0x10001000
pc_loop:
	ldwio	r4, 4(r3)
	andhi	r4, r4, 0xFFFF
	beq		r4, r0, pc_loop
	stwio	r2, 0(r3)
	
	ldw 	ra, 8(sp)
	ldw	r3, 4(sp)
	ldw	r4, 0(sp)
	addi	sp, sp, 12
	
	ret


#----------------------------------------------------------------------------------


InRange:


	subi	sp, sp, 24 
	stw 	ra, 20(sp)
	stw	r3, 16(sp) #list_ptr
	stw	r4, 12(sp)#n
	stw	r5, 8(sp)#low
	stw	r6, 4(sp)#high
	stw r7, 0(sp)#element

	ldw r4, N(r0)
	
LOOP:
IF:
	ldw r7, 0(r3)
	bge r7, r5, THEN
	br ELSE
THEN:
	bgt r7, r6, ELSE
	movi r2,'.'
	call PrintChar
	br END_IF
ELSE:
	movi r2, 'X'
	call PrintChar
	br END_IF
END_IF:
	addi r3, r3, 4
	subi r4, r4, 1
	bgt	 r4, r0, LOOP



	ldw ra, 20(sp)
	ldw	r3, 16(sp) #list_ptr
	ldw	r4, 12(sp)#n
	ldw	r5, 8(sp)#low
	ldw	r6, 4(sp)#high
	ldw r7, 0(sp)#element
	addi	sp, sp, 24
	
	ret
#--------------------------------------------------------------------------------
.org	0x1000

N:		.word 6
LIST:	.word 0,1,2,3,4,5
	