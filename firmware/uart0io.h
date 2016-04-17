/*-----------------------------------------------------------------------*\
 File:			uart0io.c
 Author:		Justin England
 Date:			Nov 2015
 
 Description:	UART serial routines using USART0.  Functions are
				provided to be called directly, and functions are provided
				for the avr-libc fdev system.  Theses will be noted
				in the function description.
 
 Known bugs/missing features:
 
 Modifications:
 Date                Comment
 ----    ------------------------------------------------
 \*----------------------------------------------------------------------*/

#ifndef _UARTIO_H_
#define _UARTIO_H_

/*********************** defines                    *************************/

#define F_UART0 &uart0                      // Map file descriptor
#define RX_BUFFER_SIZE 256                  // receive buffer size

/*********************** typedefs                   *************************/

// receive buffers for uart;
typedef struct {
	uint8_t buffer[RX_BUFFER_SIZE];
	uint8_t h_pos;
	uint8_t t_pos;
} rx_buffer_t;

/*********************** function prototypes        *************************/

void	uart0_init(uint32_t);				// init uart: args = UARTn, baudrate
int		uart0_putchar(char, FILE *);		// put_char function for uart0
int		uart0_getchar(FILE *);				// get_char for uart0
void	uart0_clear_buffer(void);			// clear UART buffer
uint8_t uart0_getc(uint8_t *c);				// get single char from buffer
void	uart0_putc(uint8_t);				// puts raw byte in uart

inline	void uart0_bufferchar(uint8_t, volatile rx_buffer_t *);

//// file handles using avr-libc fdev method
static FILE uart0 = FDEV_SETUP_STREAM(uart0_putchar, uart0_getchar, _FDEV_SETUP_RW);
//
#endif /* _UARTIO_H_ */
