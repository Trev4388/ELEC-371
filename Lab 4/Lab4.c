/*===================================================================

   Template single/combined C file for ELEC 371 laboratory activity

   October 2021  Dr. N. Manjikian

   C programs for Nios II systems can be prepared with multiple
   separate files in project defined with the Monitor Program.

   A single C file, however, is necessary for the Web-based simulator
   because the current version of the Web-based tool handles a single
   file in its editor window.

   This single file merges together several .h header files and
   the exception_handler.c file with the assembly-language code
   for saving/restoring registers and handling the ea adjustment.

   The .h header file contents are positioned at the beginning of
   this template file, and the exception_handler.c file contents
   are positioned at the end of this template file. There are
   comments with "----" dividers and original-file identification
   to aid in understanding the different portions.

   In the middle of this template file is the portion that can be
   customized for a particular application. For interrupt support,
   there is an empty interrupt_handler() function. There is an
   empty Init() function and a simple main() function. Other
   functions can be added as necessary in front of main().

==================================================================*/

/*-----------------------------------------------------------------*/

#ifndef _NIOS2_CONTROL_H_
#define _NIOS2_CONTROL_H_

/* 
   This file defines macros that utilize the built-in functions
   in Altera's adaption of the GCC compiler to generate the
   control-register-access instructions for the Nios II processor.

   For ELEC 371 laboratory work, it has been modified by N. Manjikian.

   It should be noted that the manner in which the built-in functions
   are used below _differs_ from the manner in which they are shown in
   the DE0 Basic Computer reference document from Altera. The key
   difference is for _read_ operations. The conventional method of
   defining macros that read a value is to have the value returned,
   like a function. The method used in the original Altera code is
   not conventional. The second main difference is a simplification
   that avoids the use of a do..while loop found in the Altera version.
   The do..while loops may have been introduced for a specific reason (e.g.,
   to perhaps prevent the compiler from optimizing away the calls to
   the built-in functions, or to allow the assignment of the return
   value to a named variable). For the purposes of laboratory work,
   the revision of the macros by the instructor is appropriate, and
   the compiler-generated code is correct.

   The official names for the processor control register are ctl0, ctl1, ...
   These registers are more conveniently identified by their aliases,
   such as status, estatus, ...  Documentation on the Nios II processor
   provides the association between the official names and the aliases.

   Not all of the possible control-register accesses are covered by
   the macros below. The ones most relevant for ELEC 371 laboratory work
   are included. If access to the other control registers is required,
   additional macros could easily be introduced, based on the existing ones.

   The number on the right side of each macro definition refers to the
   corresponding 'ctl_' processor control register. The __builtin_wrctl()
   function passes the ctl register number and a value to the compiler for
   use in generating the appropriate assembly-language instruction.
   The __builtin_rdctl() function passes only the register number, and
   it _returns_ a value. The instructor-revised read macros then return
   this result as the return value of the macro for use by the C program.
*/
	
#define NIOS2_WRITE_STATUS(value)  (__builtin_wrctl (0, value))

#define NIOS2_READ_IENABLE()	   (__builtin_rdctl (3))

#define NIOS2_WRITE_IENABLE(value) (__builtin_wrctl (3, value))

#define NIOS2_READ_IPENDING()	   (__builtin_rdctl (4))


#endif /* _NIOS2_CONTROL_H_ */

	/*-----------------------------------------------------------------*/

#ifndef _TIMER_H_
#define _TIMER_H_
/* define pointer macros for accessing the timer interface registers */
#define TIMER_STATUS	((volatile unsigned int *) 0x10002000)
#define TIMER_CONTROL	((volatile unsigned int *) 0x10002004)
#define TIMER_START_LO	((volatile unsigned int *) 0x10002008)
#define TIMER_START_HI	((volatile unsigned int *) 0x1000200C)
#define TIMER_SNAP_LO	((volatile unsigned int *) 0x10002010)
#define TIMER_SNAP_HI	((volatile unsigned int *) 0x10002014)
/* define a bit pattern reflecting the position of the timeout (TO) bit
   in the timer status register */
#define TIMER_TO_BIT 0x1
#endif /* _TIMER_H_ */

#ifndef _TIMER0_H_
#define _TIMER0_H_
/* define pointer macros for accessing the timer interface registers */
#define TIMER0_STATUS	((volatile unsigned int *) 0x10004000)
#define TIMER0_CONTROL	((volatile unsigned int *) 0x10004004)
#define TIMER0_START_LO	((volatile unsigned int *) 0x10004008)
#define TIMER0_START_HI	((volatile unsigned int *) 0x1000400C)
#define TIMER0_SNAP_LO	((volatile unsigned int *) 0x10004010)
#define TIMER0_SNAP_HI	((volatile unsigned int *) 0x10004014)
/* define a bit pattern reflecting the position of the timeout (TO) bit
   in the timer status register */
#define TIMER0_TO_BIT 0x1
#endif /* _TIMER0_H_ */

#ifndef _TIMER1_H_
#define _TIMER1_H_
/* define pointer macros for accessing the timer interface registers */
#define TIMER1_STATUS	((volatile unsigned int *) 0x10004020)
#define TIMER1_CONTROL	((volatile unsigned int *) 0x10004024)
#define TIMER1_START_LO	((volatile unsigned int *) 0x10004028)
#define TIMER1_START_HI	((volatile unsigned int *) 0x1000402C)
#define TIMER1_SNAP_LO	((volatile unsigned int *) 0x10004030)
#define TIMER1_SNAP_HI	((volatile unsigned int *) 0x10004034)
/* define a bit pattern reflecting the position of the timeout (TO) bit
   in the timer status register */
#define TIMER1_TO_BIT 0x1
#endif /* _TIMER_H_ */

	/*-----------------------------LEDS---------------------------------*/

#ifndef _LEDS_H_
#define _LEDS_H_
/* define pointer macro for accessing the LED interface data register */
#define LEDS	((volatile unsigned int *) 0x10000010)
#endif /* _LEDS_H_ */

/*-----------------------------------------------------------------*/
/*             start of application-specific code                  */
/*-----------------------------------------------------------------*/

/* place additional #define macros here */
	//Seven segment init
#ifndef _SEVENSD_H_
#define _SEVENSD_H_
/* define pointer macro for accessing the seven segment display interface data register */
#define SEVENSD_DATA	((volatile unsigned int *)	0x10000020)
#endif /* _SEVENSD_H_ */
	
	/*-------------------------------JTAG----------------------------------*/
	//JTAG UART init
#ifndef _JTAG_UART_H_
#define _JTAG_UART_H_
/* define pointer macro for accessing the JTAG_UART interface data register */
#define JTAG_UART	((volatile unsigned int *)	0x10001000)
#define JTAG_UART_STATUS	((volatile unsigned int *)	0x10001004)	
#endif /* _JTAG_UART_H_ */	

	/*---------------------------------ADC--------------------------------*/
	
#ifndef _ADC_H_
#define _ADC_H_

/*-------------------------------------------------------------------
  Simplified support code for performing analog-to-digital conversion
  using the Companion Board for the DE0 or DE0-CV

  N. Manjikian, ECE Dept., Queen's University, November 2018
  -------------------------------------------------------------------*/

/* function prototypes for preparing then performing conversion */

extern void         InitADC (int adc_sel, int mux_sel);

extern unsigned int ADConvert (void);

#endif /* _ADC_H_ */
	
	/*--------------------------------ADC.c------------------------------*/
	
	/*-------------------------------------------------------------------
  Simplified support code for performing analog-to-digital conversion
  using the Companion Board for the DE0 or DE0-CV

  N. Manjikian, ECE Dept., Queen's University, November 2018
  -------------------------------------------------------------------*/

/* prepare pointer symbols using appropriate memory-mapped I/O addresses */

#define JP1_DATA ((volatile unsigned int *) 0x10000060)
#define JP1_DIR  ((volatile unsigned int *) 0x10000064)


/*-------------------------------------------------------------------
  This func. sets up data direction reg. and data reg. for JP1 port
  -------------------------------------------------------------------*/

/* NOTE: assumes adc_sel is in the range 0..2 and mux_sel is in the range 0..3.
   More code could be introduced to ensure above ranges are respected. */

void InitADC(int adc_sel, int mux_sel){
  unsigned int bits;

  /* for data direction, bits 31, 28..27, 26..25, 24, 23..22, and 8
     must be set for output direction; the remaining bits are default input */

  bits = (0x1 << 31) | (0x3 << 27) | (0x3 << 25) | (0x1 << 24)
       | (0x3 << 22) | (0x1 << 8);

  /* the compiler generates a constant at compile time for the right-hand side
     of the the statement above -- alternatively, set bits to 0x9FC00100 */


  *JP1_DIR = bits;  /* write to data direction register for parallel port */


  /* for the actual bit output values, build the pattern below */

  bits = 0;                  /* prepare all zero initial bits */
  bits |= (1 << 24);         /* set DAC_write_n to 1 */
  bits |= (1 << 8);          /* set ADC_read_n to 1 */
  bits |= (adc_sel << 25);   /* set 2-bit field to choose active chip */
  bits |= (mux_sel << 27);   /* set 2-bit field to choose among 4 channels */

  *JP1_DATA = bits;  /* write to data register for parallel port */
}

unsigned int hex_table[]=
{
	0x3F, 0x06, 0x5B, 0x4F,
	0x66, 0x6D, 0x7D, 0x07,
	0x7F, 0x6F, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};

/*-------------------------------------------------------------------
  This func. performs conversion for MUX/ADC set with initialization;
  no input arguments; uses data reg. to find which ADC chip selected
  -------------------------------------------------------------------*/

unsigned int ADConvert (void){
  unsigned int bits, done_mask_value, result, adc_sel;
  int loop;

  bits = *JP1_DATA;	/* sets "bits" to current port data register contents */

  adc_sel = (bits >> 25) & 0x3;  /* which chip was selected ? */
								 //Shift bits 25 to the right and mask bit 0 and 1
								 //0xFFFF FFFF FFFF -> 0x0000 

  /* must set ADC_read_n to low and keep it low to perform conversion */

  bits &= ~(1 << 8);  /* force ADC_read_n bit to 0, preserving other bits */
  *JP1_DATA = bits;   /* write new pattern to data register */


  /* now wait until the active-low 'done' bit for selected chip is asserted */

  done_mask_value = (1 << adc_sel); /* 001, 010, or 100 for mask value */

  do
  {
    bits = (*JP1_DATA >> 11);       /* read data register, shift field down */
    bits = bits & done_mask_value;  /* isolate desired bit with mask */
  } while (bits != 0);              /* check if active-low bit is still high */

  /* when loop is exited, extract the digital value from the conversion */

  bits = *JP1_DATA;
  result = bits & 0xFF;  /* isolate the lowest 8 bits */

  /* final step is to set ADC_read_n back to 1 */

  bits |= (1 << 8);    /* bit was previously 0, so OR in 1 to that position */ 
  *JP1_DATA = bits;

  return result;   /* return the converted 8-bit unsigned value */
}

	
	/*-----------------------------------------------------------------*/
	
	
/* define global program variables here */
		//variable used as sw flag when second button is pressed
	volatile int interruptSWFlag1 = 0;
	volatile unsigned int blinky = 1;
	int counter1 = 0;
	int SSD[8] = {0x3F, 0x6, 0x3F00, 0x600, 0x3F0000, 0x60000, 0x3F000000, 0x6000000};
	int counter2 = 0;
	volatile int flag=0;

void interrupt_handler(void)
{
	unsigned int ipending;
	unsigned int ipendingMask;	//init for masking
	
	/* read current value in ipending register */
	ipending = NIOS2_READ_IPENDING();
	
	/* do one or more checks for different sources using ipending value */
		//ipendingMask gets just the value of bit 0
		//to check is timer interrupt request is there
//	ipendingMask = ipending & 1;
	if (ipending & 1){		//if (ipending & 1 != 0)
			//Set timer status to 0
		*TIMER_STATUS = 0;
		
			//set sw flag
		interruptSWFlag1 = 1;
		/*	
		//toggles the LED output
		*LEDS = *LEDS ^ 0b0101;
		*SEVENSD_DATA = *SEVENSD_DATA ^ blinky;
		
		if (blinky >= 0x80000000){	//can't go aboove 0xFFFFFFFF so do fbefore increment
			blinky = 0;	
		}
		if (blinky != 0){
			blinky *= 2;
		}else{
			blinky = 1;	
		}
		*/
	}
	
	if(ipending & 1<<13){
		*TIMER0_STATUS = 0;
		if(flag==0){
			*LEDS=992;
			flag=1;
		}else{
			*LEDS=31;
			flag=0;
		}
	}
	/*	From lab 3. Hardware is necessary for this to work
		//ipendingMask gets just the value of bit 13
		//to check if timer0 interrupt request is there
	ipendingMask = ipending & 0x2000;
		//If timer0 interrupt occurs
	if (ipendingMask){		//timer0 done
		*TIMER0_STATUS = 0;
		if (*LEDS == 0x003){
			*LEDS = 0x300;
		}else {
			//Toggle LEDS
		*LEDS = *LEDS >> 2;	//or >> 2
		}
		
	}
	
		//ipendingMask gets just the value of bit 14
		//to check if timer1 interrupt request is there
	ipendingMask = ipending & 0x4000;
		//If timer1 interrupt occurs
	if (ipendingMask){		//timer1 done
		*TIMER1_STATUS = 0;
		*SEVENSD_DATA = SSD[counter2];
		counter2++;
		if (counter2 == 8){counter2 = 0;}
	}
	*/
}

/*	Lab 2.5 code
void HandleBtn (void)
{
		//if button 1 was pressed
	if (*BTN_EDGECAP_REG & 0b0010){
			//toggle the seven segment display
		*SEVENSD_DATA = *SEVENSD_DATA ^ 0x7F;
	}
		//if button 2 was pressed
	if (*BTN_EDGECAP_REG & 0b0100){
			//set a sw flag to print '!', not print '!' in an interrupt
		BTN2Flag = 1;
		*SEVENSD_DATA = *SEVENSD_DATA ^ 0xFFFFFFFF;
	}
		//To clear the interrupt
	*BTN_EDGECAP_REG = 0b0111;	//writing to edgecap clears it
}
*/

void Init (void)
{
	/* set up each hardware interface */
		//Original Timer
		//017D7840
	*TIMER_START_LO = 0x9680;	//0.2 sec
	*TIMER_START_HI = 0x0098;
	*TIMER_CONTROL = 0x7;
	//	Lab 3 stuff	
		//Timer 0
	*TIMER0_START_LO = 25000000& 0xFFFF;	//25000000 is half a second
	*TIMER0_START_HI = (25000000>>16)&0xFFFF;
	*TIMER0_CONTROL = 0x7;
	
	//Timer 1
	//*TIMER1_START_LO = 6250000 & 0xFFFF;	//25000000 is half a second
	//*TIMER1_START_HI = 6250000 >> 16;
	//*TIMER1_CONTROL = 0x7;
	
	
	/* set up ienable */
	/* for Lab 3
	NIOS2_WRITE_IENABLE(0x6001);		//Enable 3 interrupts. might need bits 0, 13, and 14
	*/
	NIOS2_WRITE_IENABLE(0x1|1<<13);
					//0x6001
					//0b0110 0000 0000 0001
										//bit 1 for Btn and bit 0 for Timer
	NIOS2_WRITE_STATUS(1);		//write status to 0b0001
	*SEVENSD_DATA = 0x3F;
	//PrintChar('\n');		//This is causing an error bc PrintChar is defined below instead of above
	
	//*LEDS = 0x300;
}


/* place additional functions here */

void PrintChar (char x){
		//Poll until JTAG UART is ready
    while(!(*JTAG_UART_STATUS & 0xFFFF0000));	
		//Set the JTAG UART data to the input character										
    *JTAG_UART = x;	
}

void PrintString (char *string){
    
    while (*string){      //While string does not point to a space (NULL)
        PrintChar(*string);    //Print the character string points to
        string++;      //Increment string (going to the next spot in address
    }    
}

void PositionCursor(int row, int col){
		//Moves the cursor to a specified row and column
		//rows and columns must be less than 100 and greater than 0
		
		//<ESC>[row#;column#f  is the sequence needed
	PrintChar(0x1B);		//ASCII code in hex for ESC
	PrintChar('[');
	//Rows
		//Gets digit in tenth position (the 9 in 96) (96 / 10 = 9.6 = 9 in hw)
	int rowTen = row / 10;
		//Gets digit in ones position (the 6 in 96)
		//row - rowTen*10 (96 - 90 = 6)
	int rowOne = row - (rowTen * 10);
		//Makes the numbers ASCII
	char rowTenASCII = rowTen + '0';
	char rowOneASCII = rowOne + '0';
		//prints the ASCII numbers in order
	PrintChar(rowTenASCII);
	PrintChar(rowOneASCII);
	
	PrintChar(';');
	
	//Columns
		//Gets digit in tenth position (the 8 in 85) (85 / 10 = 8.5 = 8 in hw)
	int colTen = col / 10;
		//Gets digit in ones position (the 5 in 85)
		//col - colTen*10 (85 - 80 = 5)
	int colOne = col - (colTen * 10);
		//Makes the numbers ASCII
	char colTenASCII = colTen + '0';
	char colOneASCII = colOne + '0';
		//prints the ASCII numbers in order
	PrintChar(colTenASCII);
	PrintChar(colOneASCII);
	
		//finish command in terminal
	PrintChar('f');
}

void ClearScreen(){
		//uses two VT100 commands that move to the top row and first column
		//move to home command <ESC>[H
	PrintChar(0x1B);		//ASCII code in hex for ESC
	PrintChar('[');
	PrintChar('H');
	
		//clear all command <ESC>[J
	PrintChar(0x1B);		//ASCII code in hex for ESC
	PrintChar('[');
	PrintChar('2');
	PrintChar('J');
	
}

int main (void)	//-------------------------MAIN--------------------------
{
	int value, hundreds, tens, ones;
	
	
	Init ();	/* perform software/hardware initialization */
		//Lab 4 prelab code
	//PrintString("ELEC 371 Lab 4\n");
	InitADC(2, 2);
	
	ClearScreen();
	PositionCursor (3, 6);   /* fourth row, tenth column */
    PrintString("Lab 4 A/D");
	
	PositionCursor (4, 10);
	PrintChar(' ');
	
	while (1)
	{
		/* fill in body of infinite loop */
		//Lab 4 prelab code
      
      //write value to LED parallel port data register
		
		//Spinner code from Lab 3
			//if the sw flag is activated by the second button being pressed
		if (interruptSWFlag1 == 1){
			
			PositionCursor (4, 10);   /* fourth row, tenth column */
			PrintChar('\b');
			value = ADConvert();
			if(value <128){
				PrintChar('-');
			}else{
				PrintChar('+');	
			}
			
			hundreds=value/100;
			value=value-(hundreds*100);
			tens=value/10;
			value=value-(tens*10);
			ones=value;
			
			*SEVENSD_DATA= 
				(hex_table[hundreds] <<16) |
				(hex_table[tens] <<8) |
				(hex_table[ones]); 
			
			
    		

			/*
			
			//Print '\b'
			PrintChar('\b');
				//print sequence character
			if (counter1 == 0){
				PrintChar('/');
				counter1++;
			}else if (counter1 == 1){
				PrintChar('-');
				counter1++;
			}else if (counter1 == 2){
				PrintChar('\\');
				counter1++;
			}else if (counter1 == 3){
				PrintChar('|');
				counter1 = 0;
			}
			
			*/
			//reset flag
			interruptSWFlag1 = 0;
		}	
		//*/
	}
	return 0;	/* never reached, but main() must return a value */
}

/*-----------------------------------------------------------------*/
/*              end of application-specific code                   */
/*-----------------------------------------------------------------*/

/* 
   exception_handler.c

   This file is a portion of the original code supplied by Altera.

   It has been adapted by N. Manjikian for use in ELEC 371 laboratory work.

   Various unnecessary or extraneous elements have been excluded. For
   example, declarations in C for external functions called from asm()
   instructions are not required because any reference to external names
   in asm() instructions is embedded directly in the output written to
   the assembly-language .s file without any other checks by the C compiler.

   There is one particularly important change: on _reset_, the jump must be
   to the >> _start << location in order to properly initialize the stack
   pointer and to perform other crucial initialization tasks that ensure
   proper C semantics for variable initialization are enforced. The Altera
   version of the code jumped to main(), which will _not_ perform these
   crucial initialization tasks correctly.

   Finally, a reference to control register 'ctl4' in the asm() sequence
   has been replaced with the more meaningful alias 'ipending' for clarity.

   Other than the changes described above, the file contents have also been
   reformatted to fit in 80 columns of text, and comments have been edited.
*/


/* The assembly language code below handles processor reset */
void the_reset (void) __attribute__ ((section (".reset")));

/*****************************************************************************
 * Reset code. By giving the code a section attribute with the name ".reset" *
 * we allow the linker program to locate this code at the proper reset vector*
 * address. This code jumps to _startup_ code for C program, _not_ main().   *
 *****************************************************************************/

void the_reset (void)
{
  asm (".set noat");         /* the .set commands are included to prevent */
  asm (".set nobreak");      /* warning messages from the assembler */
  asm ("movia r2, _start");  /* jump to the C language _startup_ code */
  asm ("jmp r2");            /* (_not_ main, as in the original Altera file) */
}

/* The assembly language code below handles exception processing. This
 * code should not be modified; instead, the C language code in the normal
 * function interrupt_handler() [which is called from the code below]
 * can be modified as needed for a given application.
 */

void the_exception (void) __attribute__ ((section (".exceptions")));

/*****************************************************************************
 * Exceptions code. By giving the code a section attribute with the name     *
 * ".exceptions" we allow the linker program to locate this code at the      *
 * proper exceptions vector address. This code calls the interrupt handler   *
 * and later returns from the exception to the main program.                 *
 *****************************************************************************/

void the_exception (void)
{
  asm (".set noat");         /* the .set commands are included to prevent */
  asm (".set nobreak");      /* warning messages from the assembler */
  asm ("subi sp, sp, 128");
  asm ("stw  et, 96(sp)");
  asm ("rdctl et, ipending"); /* changed 'ctl4' to 'ipending' for clarity */
  asm ("beq  et, r0, SKIP_EA_DEC");   /* Not a hardware interrupt, */
  asm ("subi ea, ea, 4");             /* so decrement ea by one instruction */ 
  asm ("SKIP_EA_DEC:");
  asm ("stw	r1,  4(sp)"); /* Save all registers */
  asm ("stw	r2,  8(sp)");
  asm ("stw	r3,  12(sp)");
  asm ("stw	r4,  16(sp)");
  asm ("stw	r5,  20(sp)");
  asm ("stw	r6,  24(sp)");
  asm ("stw	r7,  28(sp)");
  asm ("stw	r8,  32(sp)");
  asm ("stw	r9,  36(sp)");
  asm ("stw	r10, 40(sp)");
  asm ("stw	r11, 44(sp)");
  asm ("stw	r12, 48(sp)");
  asm ("stw	r13, 52(sp)");
  asm ("stw	r14, 56(sp)");
  asm ("stw	r15, 60(sp)");
  asm ("stw	r16, 64(sp)");
  asm ("stw	r17, 68(sp)");
  asm ("stw	r18, 72(sp)");
  asm ("stw	r19, 76(sp)");
  asm ("stw	r20, 80(sp)");
  asm ("stw	r21, 84(sp)");
  asm ("stw	r22, 88(sp)");
  asm ("stw	r23, 92(sp)");
  asm ("stw	r25, 100(sp)"); /* r25 = bt (r24 = et, saved above) */
  asm ("stw	r26, 104(sp)"); /* r26 = gp */
  /* skip saving r27 because it is sp, and there is no point in saving sp */
  asm ("stw	r28, 112(sp)"); /* r28 = fp */
  asm ("stw	r29, 116(sp)"); /* r29 = ea */
  asm ("stw	r30, 120(sp)"); /* r30 = ba */
  asm ("stw	r31, 124(sp)"); /* r31 = ra */
  asm ("addi	fp,  sp, 128"); /* frame pointer adjustment */

  asm ("call	interrupt_handler"); /* call normal function */

  asm ("ldw	r1,  4(sp)"); /* Restore all registers */
  asm ("ldw	r2,  8(sp)");
  asm ("ldw	r3,  12(sp)");
  asm ("ldw	r4,  16(sp)");
  asm ("ldw	r5,  20(sp)");
  asm ("ldw	r6,  24(sp)");
  asm ("ldw	r7,  28(sp)");
  asm ("ldw	r8,  32(sp)");
  asm ("ldw	r9,  36(sp)");
  asm ("ldw	r10, 40(sp)");
  asm ("ldw	r11, 44(sp)");
  asm ("ldw	r12, 48(sp)");
  asm ("ldw	r13, 52(sp)");
  asm ("ldw	r14, 56(sp)");
  asm ("ldw	r15, 60(sp)");
  asm ("ldw	r16, 64(sp)");
  asm ("ldw	r17, 68(sp)");
  asm ("ldw	r18, 72(sp)");
  asm ("ldw	r19, 76(sp)");
  asm ("ldw	r20, 80(sp)");
  asm ("ldw	r21, 84(sp)");
  asm ("ldw	r22, 88(sp)");
  asm ("ldw	r23, 92(sp)");
  asm ("ldw	r24, 96(sp)");
  asm ("ldw	r25, 100(sp)");
  asm ("ldw	r26, 104(sp)");
  /* skip r27 because it is sp, and we did not save this on the stack */
  asm ("ldw	r28, 112(sp)");
  asm ("ldw	r29, 116(sp)");
  asm ("ldw	r30, 120(sp)");
  asm ("ldw	r31, 124(sp)");

  asm ("addi	sp,  sp, 128");

  asm ("eret"); /* return from exception */

  /* Note that the C compiler will still generate the 'standard'
     end-of-normal-function code with a normal return-from-subroutine
     instruction. But with the above eret instruction embedded
     in the final output from the compiler, that end-of-function code
     will never be executed.
   */ 
}