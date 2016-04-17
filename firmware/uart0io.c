/*-----------------------------------------------------------------------*\
 File:			uart0io.c
 
 Description:	UART serial routines using USART0.  Functions are
				provided to be called directly, and functions are provided
				for the avr-libc fdev system.  Theses will be noted
				in the function description.
 
 Known bugs/missing features:
 
 Modifications:
 Date                Comment            
 ----    ------------------------------------------------
\*----------------------------------------------------------------------*/

/*********************** include files              *********************/

#include <stdio.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart0io.h"
#include "globals.h"

/*********************** global variables           *********************/

volatile rx_buffer_t uart0_buf = { {0}, 0, 0 };		// ring buffer for RX


/*********************** extern variables           *********************/


/*********************** ISR definitions            *********************/

/*----------------------------------------------------------------------*\
 
 Function:		ISR(USART0_RX_vect)
 
 Description:	USART0 RX data ready interrupt. receives char from 
				usart register.
 
 Parameters:	none
 Returns:		none
 
\*----------------------------------------------------------------------*/

ISR(USART_RX_vect)
{
	uint8_t c = UDR0;					// get character from data reg

	uart0_bufferchar(c, &uart0_buf);
	
}

/*********************** function definitions       *********************/

/*----------------------------------------------------------------------*\
 
 Function:		uart_init()
 
 Description:	initalizes USART interface
 
 Parameters:	uint32_t baud		- baud rate to use

 Returns:		none
 
\*----------------------------------------------------------------------*/

void uart0_init(uint32_t baud)
{

	uint16_t _ubrr;
	
	// calulate baud rate
	_ubrr	= (F_CPU/4/baud - 1) / 2;
	UBRR0 = _ubrr;


	
	UCSR0A = (1 << U2X0);				// double speed to handle 230400
	UCSR0B |=							// config UCSR0B
		(1 << TXEN0) |					// enable transmit
		(1 << RXEN0) |					// enable receive
		(1 << RXCIE0);					// enable rx interrupt
	
	fprintf(F_UART0, "\n\nInitalizing hardware subsystems:\n\n");
	fprintf(F_UART0, "Initalizing UART I/O............\tDone\n");

}

/*----------------------------------------------------------------------*\
 
 Function:		uart0_bufferadd()
 
 Description:	called from RX ISR with received character.  Puts char
				into ring buffer.  If buffer full, thows away character
 
 Parameters:	char c	-	character recived from data register
				rx_buffer_t *usart_buf - ptr to ring buffer
 
 Returns:		none
 
\*----------------------------------------------------------------------*/

inline void uart0_bufferchar(unsigned char c, volatile rx_buffer_t *uart_buf)
{
	int i = (uart_buf->h_pos + 1) % RX_BUFFER_SIZE;
	
	// if the next head position 'i' == the tail position
	// we will overrun the buffer, esentially making it
	// appear empty.  Instead, quietly discard character
	//
	// if not, then add received character to buffer.
	// and update head position pointer (index)
	
	if (i != uart_buf->t_pos)
	{
		uart_buf->buffer[uart_buf->h_pos] = c;
		uart_buf->h_pos = i;
	}
}

/*----------------------------------------------------------------------*\
 
 Function:		uart0_putchar()
 
 Description:	avr-libc fdev function.  Puts single char in data reg
				and waits for end of transmission.  Translates '\n' to
				'\r\n'
 
 Parameters:	char c			-	character to output
				FILE *stream	-	stream associated with this UART
 
 Returns:		0 - success
 
\*----------------------------------------------------------------------*/

int uart0_putchar(char c, FILE *stream)
{
    if (c == '\n') uart0_putchar('\r', stream);
	
	while(!(UCSR0A & (1 << UDRE0)) ) { }
    
    UDR0 = c;
    
    return 0;
}

/*----------------------------------------------------------------------*\
 
 Function:		uart0_getchar()
 
 Description:	avr-libc fdev function.  Retreives single char from 
				ring buffer and returns char.
 
 Parameters:	FILE *stream	-	stream associated with this UART
 
 Returns:		int				-	char from ring buffer
 
\*----------------------------------------------------------------------*/


int uart0_getchar(FILE *stream)
{
	unsigned char c;
	
	// if h_pos = t_pos, there are no characters in buffer
	// wait until a character comes in
	
	if(uart0_buf.h_pos == uart0_buf.t_pos)
		return -1;
	
	c = uart0_buf.buffer[uart0_buf.t_pos];
	uart0_buf.t_pos = (uart0_buf.t_pos + 1) % RX_BUFFER_SIZE;
	
	return c;
}

/*----------------------------------------------------------------------*\
 
 Function:		uart0_putc()
 
 Description:	direct call function.  Puts single char in data reg
				and waits for end of transmission. Does not do any
				translations and it 8 bit safe
 
				This differs from putchar() in that it does not do any
				manipulation of the outgoing data where putchar() 
				translates \n -> \r\n
 
 Parameters:	uint8_t c	-	character to output
 
 Returns:		none
 
\*----------------------------------------------------------------------*/

void uart0_putc(uint8_t c)
{
	while(!(UCSR0A & (1 << UDRE0)) ) { }
    UDR0 = c;
}

/*----------------------------------------------------------------------*\
 
 Function:		uart0_getc()
 
 Description:	direct call function.  Retreives single char from 
				ring buffer and puts char in passed in variable
 
				This differs from getchar() as it does not return
				the char, but instead returns 0 if no characters are
				in buffer, otherwise return one if a char was retreived
				from buffer.  This method will not clobber date bytes 
				with a value of 255 (getchar() returns 255 on error)
 
 Parameters:	uint8_t * c	-	pointer to char location to store char
 
 Returns:		uint8_t		-	0 - failed (no char in buffer)
								1 - success (char retreived from buffer)
 
\*----------------------------------------------------------------------*/

uint8_t uart0_getc(uint8_t *c)
{
	// if h_pos = t_pos, there are no characters in buffer
	if(uart0_buf.h_pos == uart0_buf.t_pos)
		return 0;
	
	// put char from buffer into var pointed to by c
	*c = uart0_buf.buffer[uart0_buf.t_pos];
	uart0_buf.t_pos = (uart0_buf.t_pos + 1) % RX_BUFFER_SIZE;
	
	return 1;
}

/*----------------------------------------------------------------------*\
 
 Function:		uart0_clear_buffer()
 
 Description:	resets ring buffer head and tail pointers and zero's
				out buffer memory
 
 Parameters:	none
 Returns:		none
 
\*----------------------------------------------------------------------*/


inline void uart0_clear_buffer(void)
{
	int i;									// counter
	
	cli();									// make function atomic
	
	uart0_buf.t_pos = uart0_buf.h_pos = 0;	// reset head and tail ptrs
	for (i = 0; i < RX_BUFFER_SIZE; i ++)	// clear all data from buffer
		uart0_buf.buffer[i] = 0;
	
	sei();									// turn interrupts back on	
}
