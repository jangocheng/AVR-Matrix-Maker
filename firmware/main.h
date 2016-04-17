/****************************************************************************\
 File:			main.h
 Author:		
 Date:			
 
 Description:	Header file for main.c
 
 Known bugs/missing features:
 
 Modifications:
 Date                Comment            
 ----    ------------------------------------------------
\****************************************************************************/

#ifndef _MAIN_H_
#define _MAIN_H_

/*********************** typedefs                   *************************/

// struct to hold all data needed to display a pixel; x,y coordinate + color
typedef struct
{
	uint8_t		x_pos;
	uint8_t		y_pos;
	uint8_t		color;
} pixel_struct_t;

/*********************** function prototypes        *************************/

static void init_spi(void);
static void init_matrix(void);
static void init_timers(void);

void process_char(uint8_t c);
void process_pixel(void);
void send_to_matrix(uint8_t row_reg, uint8_t red, uint8_t green);
void refresh_matrix(void);

#endif	/* _MAIN_H_  */
