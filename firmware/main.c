/****************************************************************************\
 File:			main.c
 
 Description:	
 
 Known bugs/missing features:
 
 Modifications:
 Date                Comment            
 ----    ------------------------------------------------
\****************************************************************************/

/*********************** include files              *************************/

#include <stdint.h>
#include <stdio.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "main.h"
#include "globals.h"
#include "uart0io.h"
#include "max7219.h"

/*********************** pin defines                *************************/

#define MATRIX_DATA	 	B,3		// data line:			PB3 / MOSI	/ D11
#define MATRIX_RCLK		B,2		// '595 latch line:		PB2 / SS	/ D10
#define MATRIX_SCK		B,5		// clock line pin:		PB5 / SCK	/ D13

/*********************** defines                    *************************/

#define LED_OFF		0
#define LED_RED		1
#define LED_GREEN	2
#define LED_ORANGE	3

#define MAX_ROWS	8

// RX Data States
#define STATE_IDLE	0
#define STATE_CMD	1
#define STATE_XPOS	2
#define STATE_YPOS	3
#define STATE_COLOR	4

// characters used for serial comms
//#define CHAR_NUL		0x00	// NUL character
#define CHAR_STX		0x02	// STX character (cntl-B)
//#define CHAR_ETX		0x03	// ETX character (cntl-C)
//#define CHAR_EOT		0x04	// EOT character (cntl-D)
#define CHAR_ENQ		0x05	// ENQ character (cntl-E)
#define CHAR_ACK		0x06	// ACK character (cntl-F)
//#define CHAR_BS			0x08	// BS  character (cntl-H)
//#define CHAR_CR			0x0d	// CR  character (cntl-M)
//#define CHAR_CAN		0x18	// CAN character (cntl-X)

/*********************** global variables           *************************/

pixel_struct_t	pixel;				// pixel struct
uint8_t			state_current;		// current state
const char		*colors[]		= {"OFF", "RED", "GREEN", "ORANGE"};
uint8_t			matrix[8][8];
uint8_t			color_toggle_en = 0;


//uint8_t			packet_checksum;
uint8_t			packet_command;

/*********************** volatile variables         *************************/

volatile uint8_t color_toggle = MAX7219_SHUTDOWN_MODE_ON;

/*--------------------------------------------------------------------------*\
 
 Function:      TIMER2_COMPA_vect
 
 Description:   Timer interrupt used to toggle one MAX led driver off
                and toggle the other MAX driver on.  This allows the use
                of the common row lines for each of the two MAX drivers.
 
 Parameters:	void
 Returns:
 
 \*--------------------------------------------------------------------------*/


ISR(TIMER2_COMPA_vect)
{
    // check if we are active (and toggling colors)
	if(color_toggle_en == 1)
	{
		// the high 7 bits of the MAX_SHUTDOWN_[ON,OFF] are don't care bits,
		// therefore, just toggle the entire byte to change the command
		// from xxxxxxx0 to xxxxxxx1
		send_to_matrix(MAX7219_REG_SHUTDOWN, color_toggle, ~(color_toggle));
		color_toggle = ~(color_toggle);
	}
}

/*--------------------------------------------------------------------------*\
 
 Function:		main.c
 
 Description:	Waits for char from UART and sends char to be processed
 
 Parameters:	void
 Returns:		
 
\*--------------------------------------------------------------------------*/

int main(void)
{
	uint8_t c;
	
	state_current = STATE_IDLE;
	
	uart0_init(230400);
	init_spi();
	init_matrix();
	init_timers();
	fprintf(F_UART0, "matrix: waiting for ENQ\n");
	sei();
	
    while (1)
    {
		if(uart0_getc(&c)) 			// wait until byte received
		{
			process_char(c);		// run through state machine
		}

    }
    return 0;
}

/*--------------------------------------------------------------------------*\
 
 Function:      process_char
 
 Description:   processes the current character received from USART
 
 Parameters:	uint8_t c       - character received
 Returns:		void
 
\*--------------------------------------------------------------------------*/

void process_char(uint8_t c)
{
	switch (state_current)
	{
        // wait for inital ENQ char to start
		case STATE_IDLE:
			if(c == CHAR_ENQ) {
				uart0_putc(CHAR_ACK);
				color_toggle_en = 1;
				state_current = STATE_XPOS;
			}
			break;
		
//		case STATE_CMD:
//			packet_command	= c;
//			packet_checksum = packet_command;
			
		case STATE_XPOS:
			pixel.x_pos		= c;
			state_current	= STATE_YPOS;
			break;
			
		case STATE_YPOS:
			pixel.y_pos		= c;
			state_current	= STATE_COLOR;
			break;
		
		case STATE_COLOR:
			pixel.color		= c;
			state_current	= STATE_XPOS;
			process_pixel();
			break;
	}
}


/*--------------------------------------------------------------------------*\
 
 Function:      process_pixel
 
 Description:   uses global pixel struct to update Max chips for the Matrix
 
 Parameters:	void
 Returns:		void
 
 \*--------------------------------------------------------------------------*/

void process_pixel(void)
{
	uint8_t x_pos;
	
    // move bit into pixel's x position
	x_pos = (0x80 >> pixel.x_pos);

	switch (pixel.color)
	{
		case LED_RED:
			matrix[LED_RED][pixel.y_pos]	|=  (x_pos);
			matrix[LED_GREEN][pixel.y_pos]	&= ~(x_pos);
			break;
			
		case LED_GREEN:
			matrix[LED_RED][pixel.y_pos]	&= ~(x_pos);
			matrix[LED_GREEN][pixel.y_pos]	|=  (x_pos);
			break;
		
		case LED_ORANGE:
			matrix[LED_RED][pixel.y_pos]	|= (x_pos);
			matrix[LED_GREEN][pixel.y_pos]	|= (x_pos);
			break;
			
		case LED_OFF:
			matrix[LED_RED][pixel.y_pos]	&= ~(x_pos);
			matrix[LED_GREEN][pixel.y_pos]	&= ~(x_pos);
			break;
	}
	
    // the matrix data array is now set accordingly, send to matrix
    
	send_to_matrix( (MAX7219_REG_ROW0 + pixel.y_pos),
				   matrix[LED_RED][pixel.y_pos],
				   matrix[LED_GREEN][pixel.y_pos]);
	
}

/*--------------------------------------------------------------------------*\
 
 Function:		void send_to_matrix(
						uint8_t reg, uint8_t red_byte, uint8_t green_byte)
 
 
 Description:	This is a convenience function that will write to the two
				max chips on the square.
				This can ONLY be used when writing to the same register
				address on both max drivers.
 
 Parameters:	uint8_t reg         - the register address in max
				uint8_t red_byte	- the byte to write to red max
				uint8_t green_byte	- the byte to write to green max
 
 Returns:		void
 
\*--------------------------------------------------------------------------*/

void send_to_matrix(uint8_t reg, uint8_t red_byte, uint8_t green_byte)
{

	color_toggle_en = 0;

	LOW(MATRIX_RCLK);
	
	// send data to red chip first
	SPDR = reg;                         // send the register address first
	while(!(SPSR & (1 << SPIF)));       // wait for SPI Interrupt flag
	
	SPDR = red_byte;                    // send data to wrtie to register
	while(!(SPSR & (1 << SPIF)));       // wait for SPI interrupt flag
	
	// send data to green chip next
	SPDR = reg;                         // send the register address first
	while(!(SPSR & (1 << SPIF)));       // wait for SPI Interrupt flag
	
	SPDR = green_byte;					// send data to wrtie to register
	while(!(SPSR & (1 << SPIF)));       // wait for SPI interrupt flag
	
	HIGH(MATRIX_RCLK);

	color_toggle_en = 1;
	
}


/*--------------------------------------------------------------------------*\
 
 Function:		void refresh_matrix(uint8_t square)
 Author:		Justin England
 
 Description:   this will write all rows bytes in memory to the max chips
 
 Parameters:	void
 Returns:		void
 
\*--------------------------------------------------------------------------*/

void refresh_matrix(void)
{

	uint8_t row;			// loop counters per row
	uint8_t red_byte,		// data byte for red max
			green_byte,		// data byte for green max
	
	
	// turn off max toggle
	color_toggle_en = 0;
	
	
	// interate through all rows character to display
	for(row = 0; row < MAX_ROWS; row++)
	{
		
		red_byte	= matrix[LED_RED][row];
		green_byte	= matrix[LED_GREEN][row];
		
		// the bytes for the red and green drivers are ready to SPI out
		send_to_matrix(MAX7219_REG_ROW0 + row, red_byte, green_byte);
		
	}
	
	// the square has been refreshed, turn on toggle to display
	color_toggle_en = 1;
}


/*--------------------------------------------------------------------------*\
 
 Function:		void init_timers(void)
 
 Description:	Initalizes avr timer used for the max chip toggling
 coin switch.
 
 Parameters:	void
 Returns:		void
 
\*--------------------------------------------------------------------------*/


static void init_timers(void)
{
	fprintf(F_UART0, "Initalizing Timers..............\t");

	// set up timer 2
	TCCR2A = (1 << WGM21);		// CTC mode
	TCCR2B = (1 << CS22);       // /64 prescale =  4us / tick
	OCR2A  = 249;				// 4us * 250 ticks = 1ms / int (1kHz)
	TIMSK2 = (1 << OCIE2A);		// enable COMPA interrupt
	
	fprintf(F_UART0, "Done\n");
	
}


/*--------------------------------------------------------------------------*\
 
 Function:		void init_spi(void)
 
 Description:	Initalizes the SPI hardware used for comms to MAX drivers
 
 Parameters:	void
 Returns:		void
 
\*--------------------------------------------------------------------------*/

static void init_spi(void)
{
	
	fprintf(F_UART0, "Initalizing SPI I/O.............\t");
    
	// set port directions for the pins used to communicate via spi
	OUTPUT(MATRIX_DATA);
	OUTPUT(MATRIX_RCLK);
	OUTPUT(MATRIX_SCK);
	
	
	// set start-up states for max7219 connected pins
	//
	// the LATCH/LOAD/CS (LATCH) pin is set high first.  Because the max7219
	// loads the last 16 bits clocked in the data line, and since the max
	// should be in its default state, no stray data will show ip on the
	// square's leds. (The max7221 works like a normal CS line)
	//
	// The DATA and SCK lines will be controlled by the AVR's SPI hardware
	
	HIGH(MATRIX_RCLK);
	HIGH(MATRIX_SCK);
	HIGH(MATRIX_DATA);
	
	// set up the AVR's SPI hardware
	//
	// The max7219/21 requires a minimum clock period of 100 ns
	// which gives a max frequency of (1/100ns) = 10Mhz
	
	SPCR = (
			//(1 << SPR0)	|		// /8 prescale (used with SPI2X bit)
			(1 << SPE)  |		// SPI Enable
			(1 << MSTR)			// SPI Master mode
			
			);
	
	SPSR = (1 << SPI2X);		// /8 prescale Fosc / 8 = 16Mhz/8 = 2Mhz
	
	fprintf(F_UART0, "Done\n");
	
	return;
	
}

/*--------------------------------------------------------------------------*\
 
 Function:		void init_matrix(void)
 
 Description:	Initalizes the square's max7219 drivers and
                clears the display matrix
 
 
 Parameters:	void
 Returns:		void
 
 \*--------------------------------------------------------------------------*/

static void init_matrix(void)
{
	fprintf(F_UART0, "Initalizing Matrix..............\t");
    
	//
	// first turn off display-test mode.  According to the max7219 datasheet,
	// the max will default into display-test mode which will light all leds.
	// This overides the shutdown mode so display-test mode should be turned
	// off first.
	//
	
	send_to_matrix(MAX7219_REG_DISPLAY_TEST,
				   MAX7219_DISPLAY_TEST_OFF, MAX7219_DISPLAY_TEST_OFF);
	
	
	//
	// set all max drivers to Shutdown Mode = on; this will put the
	// max chips in shutdown mode where all leds are turned off
	//
	
	send_to_matrix(MAX7219_REG_SHUTDOWN,
				   MAX7219_SHUTDOWN_MODE_ON, MAX7219_SHUTDOWN_MODE_ON);
	
	//
	// set the max drivers to NO DECODE mode (no need to decode BCD chars)
	//
	
	send_to_matrix(MAX7219_REG_DECODE_MODE,
				   MAX7219_DECODE_MODE_OFF, MAX7219_DECODE_MODE_OFF);
	
	//
	// set to maximum segment intensity (drive current / pwm duty cycle)
	//
	
	send_to_matrix(MAX7219_REG_INTENSITY,
				   MAX7219_DUTY_CYCLE_31_32, MAX7219_DUTY_CYCLE_31_32);
	
	//
	// set to scan all 8 digits (digits on the max are rows on the 8x8 matrix)
	//
	
	send_to_matrix(MAX7219_REG_SCAN_LIMIT,
				   MAX7219_DISPLAY_DIGIT_0_7, MAX7219_DISPLAY_DIGIT_0_7);
	
	// All init commands have been pushed through, now clear the register
	// contents of the max chips
	
	refresh_matrix();
	fprintf(F_UART0, "Done\n");
}




