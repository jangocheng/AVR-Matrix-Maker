/****************************************************************************\
 File:			globals.h
 Author:		
 Date:			
 
 Description:	Defines and variables global to project
 
 Known bugs/missing features:
 
 Modifications:
 Date                Comment            
 ----    ------------------------------------------------
\****************************************************************************/

/* Commonly used comment templates */

/*--------------------------------------------------------------------------*\
 
 Function:		
 Author:		
 
 Description:	
 
 Parameters:	void
 Returns:		void
 
\*--------------------------------------------------------------------------*/

/***********************                            *************************/
/*********************** defines                    *************************/
/*********************** global variables           *************************/
/*********************** include files              *************************/
/*********************** function prototypes        *************************/
/*********************** function definitions       *************************/
/*********************** ISR definitions            *************************/


#ifndef _GLOBALS_H_
#define _GLOBALS_H_ 1


/*---------------------- avr pin macros				------------------------*\

 The following macros are used to make PIN/PORT management easier.
 
 http://www.starlino.com/port_macro.html
 
 Usage Example:
 ———————————————–
 #define pinLED B,5  //define pins like this
 
 OUTPUT(pinLED);     //compiles as DDRB |= (1<<5);
 HIGH(pinLED);       //compiles as PORTB |= (1<<5);

\*--------------------------------------------------------------------------*/
 
 
// macros used indirectly by the main macros
#define _SET(type,name,bit)			type ## name  |= _BV(bit)
#define _CLEAR(type,name,bit)		type ## name  &= ~ _BV(bit)
#define _TOGGLE(type,name,bit)		type ## name  ^= _BV(bit)
#define _GET(type,name,bit)			((type ## name >> bit) &  1)

// main macros used within the code
#define OUTPUT(pin)					_SET(DDR,pin)
#define INPUT(pin)					_CLEAR(DDR,pin)
#define HIGH(pin)					_SET(PORT,pin)
#define LOW(pin)					_CLEAR(PORT,pin)
#define TOGGLE(pin)					_TOGGLE(PORT,pin)
#define READ(pin)					_GET(PIN,pin)

/*********************** defines                    *************************/

#define PROJECT_VERSION_MAJOR	0x00
#define PROJECT_VERSION_MINOR	0x01

/*********************** global variables           *************************/

#endif	/* _GLOBALS_H_ */
