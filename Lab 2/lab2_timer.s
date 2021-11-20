#-----------------------------------------------------------------------------
# This template source file for ELEC 371 Lab 2 experimentation with interrupts
# also serves as the template for all assembly-language-level coding for
# Nios II interrupt-based programs in this course. DO NOT USE the approach
# shown in the vendor documentation for the DE0 Basic (or Media) Computer.
# The approach illustrated in this template file is far simpler for learning.
#
# Dr. N. Manjikian, Dept. of Elec. and Comp. Eng., Queen's University
#-----------------------------------------------------------------------------

	.text		# start a code segment 

	.global	_start	# export _start symbol for linker 

#-----------------------------------------------------------------------------
# Define symbols for memory-mapped I/O register addresses and use them in code
#-----------------------------------------------------------------------------

# the symbols below are for the Lab 2 timer experiment

	.equ	TIMER_STATUS, 0x10002000
	.equ	TIMER_CONTROL, 0x10002004
	.equ	TIMER_START_LO, 0x10002008
	.equ	TIMER_START_HI, 0x1000200C
	
	.equ	LEDS, 0x10000010

#-----------------------------------------------------------------------------
# Define two branch instructions in specific locations at the start of memory
#-----------------------------------------------------------------------------

	.org	0x0000	# this is the _reset_ address 
_start:
	br	main	# branch to actual start of main() routine 

	.org	0x0020	# this is the _exception/interrupt_ address
 
	br	isr	# branch to start of interrupt service routine 
			# (rather than placing all of the service code here) 

#-----------------------------------------------------------------------------
# The actual program code (incl. service routine) can be placed immediately
# after the second branch above, or another .org directive could be used
# to place the program code at a desired address (e.g., 0x0080). It does not
# matter because the _start symbol defines where execution begins, and the
# branch at that location simply forces execution to continue where desired.
#-----------------------------------------------------------------------------

main:
	movia sp, 0x007FFFFC		# initialize stack pointer

	call Init		# call hw/sw initialization subroutine

	stw r0, COUNT(r0)	# perform any local initialization before main loop 
				#initialize count register to 0
	
	

main_loop:

			# body of main loop (reflecting typical embedded
			#   software organization where execution does not
			#   terminate)

	ldw r2, COUNT(r0)
	addi r2, r2, 1
	stw r2, COUNT(r0)

	br main_loop

#-----------------------------------------------------------------------------
# This subroutine should encompass preparation of I/O registers as well as
# special processor registers for recognition and processing of interrupt
# requests. Preparation of program data variables can also be included here.
#-----------------------------------------------------------------------------

Init:		
			# make it modular -- save/restore registers
			# body of Init() subroutine

	subi    sp, sp, 8
   	stw    r2, 4(sp)
   	stw    r3, 0(sp)

   	movia    r3, 0x02FAF080
	srli    r3, r3, 1

    	movia    r2, TIMER_START_LO
    	stwio    r3, 0(r2)

    	movia    r2, TIMER_START_HI
    	srli    r3, r3, 16
    	stwio    r3, 0(r2)

   	movia    r2, TIMER_CONTROL
    	movi    r3, 7
    	stwio    r3, 0(r2)

    	movia    r2, TIMER_STATUS
    	stwio    r0, 0(r2)
    	movi     r2, 1
    	wrctl    ienable, r2
    	wrctl    status, r2

    	ldw    r2, 4(sp)
    	ldw    r3, 0(sp)
    	addi     sp, sp, 8

	ret

#-----------------------------------------------------------------------------
# The code for the interrupt service routine is below. Note that the branch
# instruction at 0x0020 is executed first upon recognition of interrupts,
# and that branch brings the flow of execution to the code below. Therefore,
# the actual code for this routine can be anywhere in memory for convenience.
# This template involves only hardware-generated interrupts. Therefore, the
# return-address adjustment on the ea register is performed unconditionally.
# Programs with software-generated interrupts must check for hardware sources
# to conditionally adjust the ea register (no adjustment for s/w interrupts).
#-----------------------------------------------------------------------------

isr:
				# save register values, except ea which
				#  must be modified for hardware interrupts
	subi sp, sp, 8
	stw r2, 4(sp)
	stw r3, 0(sp)

	subi	ea, ea, 4	# ea adjustment required for h/w interrupts

				# body of interrupt service routine
				#   (use the proper approach for checking
				#    the different interrupt sources)
	rdctl r2, ipending

				# restore register values

check_timer:
	andi  r3, r2, 1
	beq r3, r0, exit_isr
	movia r2, TIMER_STATUS
	stwio r0, 0(r2)
	movia r2, LEDS
	ldwio r3, 0(r2)
	xori r3, r3, 1
	stwio r3, 0(r2)

	
exit_isr: 
	

	ldw r2, 4(sp)
	ldw r3, 0(sp)
	addi sp, sp, (8)
	
	eret			# interrupt service routines end _differently_
				# than subroutines; execution must return to
				# to point in main program where interrupt
				# request invoked service routine
				
#----------------------------------------------------------------------------

Switches:
 	subi sp, sp, 16
	stw ra, 12(sp)
 	stw r3, 8(sp) #button address
 	stw r4, 4(sp) #stores button status
	stw r5, 0(sp) #address of switch
	
	movia r3, 0x10000050
	movia r5, 0x10000040
	
Switches_Loop:
	ldwio r4, 0(r3)
	beq r4, r0, Switches_Loop 
	
	ldwio r2, 0(r5)
	
	ldw ra, 12(sp)
 	ldw r3, 8(sp) #button address
 	ldw r4, 4(sp) #stores button status
	ldw r5, 0(sp) #address of switch	
	addi sp, sp, 16
	
	ret
	
#----------------------------------------------------------------------------

SevenSeg:
	subi sp, sp, 16
	stw ra, 12(sp)
	stw r3, 8(sp) #address of sevenseg
	stw r4, 4(sp) #store output of Switches
	stw r5, 0(sp) #stores P/n
	
	movia r3, 0x10000020
	mov r4, r2
	
SevenSeg_If:
	beq r4, r0, SevenSeg_Then
	
	#print n
	movi r5, 0x54
	stwio r5, 0(r3)
	
	br SevenSeg_EndIf
	
SevenSeg_Then:
	#print 0
	movi r5, 0x3F
	stwio r5, 0(r3)
	
SevenSeg_EndIf:

	ldw ra, 12(sp)
	ldw r3, 8(sp) #address of sevenseg
	ldw r4, 4(sp) #store output of Switches
	ldw r5, 0(sp) #stores P/n
	addi sp, sp, 16

ret
	
#-----------------------------------------------------------------------------
# Definitions for program data, incl. anything shared between main/isr code
#-----------------------------------------------------------------------------

	.org	0x1000		# start should be fine for most small programs
				
	COUNT: .skip 4		# define/reserve storage for program data

	.end