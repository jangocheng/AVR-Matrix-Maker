
#########  AVR Project Makefile Template   #########

PROJECTNAME = avr-matrix-maker
PROJECTTYPE = Project

# TODO rename to SRCS
PRJSRC = main.c uart0io.c

INC =
LIBS =

USE_MATH_LIB	=
USE_PRINTF		=
USE_SCANF		=

#USE_PRINTF_FLT	= true
#USE_SCANF_FLT	= true

OPTLEVEL = s

# Select the Arduino board to be programmed
ARDUINO = atmega328

# If using bare metal AVR chip, leave above ARDUINO empty and fill in
# the following vars as needed for your chip and programmer
#MCU					= atmega328p
#CLOCK					= 16000000
#BAUD					= 57600
#PROGRAMMER_MCU			= m328p
#AVRDUDE_PROGRAMMERID	= arduino
#AVRDUDE_PORT			= /dev/tty.usbserial*

###############################################################################
#																			
# Arduino Types
#
# These types of Arduinos are pulled from boards.txt
# from the Arduino software environment.
#
# uno			- Uno							Atmega328
# atmega328		- Duemilanove					ATmega328
# diecimila		- Diecimila or Duemilanove		ATmega168
# nano328		- Nano							ATmega328
# nano			- Nano							ATmega168
# mega2560		- Mega 2560 or Mega ADK			ATmega2560
# mega			- Mega 							ATmega1280
# leonardo		- Leonardo						ATmega32u4
# mini328		- Mini							ATmega328
# mini			- Mini							ATmega168
# ethernet		- Ethernet						ATmega328
# fio			- Fio							ATmega328
# bt328			- BT							ATmega328
# bt			- BT							ATmega168
# lilypad328	- LilyPad						ATmega328
# lilypad		- LilyPad						ATmega168
# pro5v328		- Pro or Pro Mini (5V, 16 Mhz)	ATmega328
# pro5v			- Pro or Pro Mini (5V, 16 MHz)	ATmega168
# pro328		- Pro or Pro Mini (3.3V, 8 MHz)	ATmega328
# pro			- Pro or Pro Mini (3.3V, 8 MHz)	ATmega168
# atmega168		- Arduino NG or older			ATmega168
# atmega8		- Arduino NG or older			ATmega8
#
###############################################################################

#####         Target Specific Details          #####
#####     Customize these for your project     #####


ifeq ($(ARDUINO), uno)
	MCU						= atmega328p
	CLOCK					= 16000000
	BAUD					= 115200
	PROGRAMMER_MCU			= m328p
	AVRDUDE_PROGRAMMERID	= arduino
	AVRDUDE_PORT			= /dev/tty.usbmodem*
endif

ifeq ($(ARDUINO), atmega328)
	MCU						= atmega328p
	CLOCK					= 16000000
	BAUD					= 57600
	PROGRAMMER_MCU			= m328p
	AVRDUDE_PROGRAMMERID	= arduino
	AVRDUDE_PORT			= /dev/tty.usbserial*
endif

ifeq ($(ARDUINO), diecimila)
	MCU						= atmega168
	CLOCK					= 16000000
	BAUD					= 19200
	PROGRAMMER_MCU			= m168
	AVRDUDE_PROGRAMMERID	= arduino
	AVRDUDE_PORT			= /dev/tty.usbserial*
endif

ifeq ($(ARDUINO), nano328)
	MCU						= atmega328p
	CLOCK					= 16000000
	BAUD					= 57600
	PROGRAMMER_MCU			= m328p
	AVRDUDE_PROGRAMMERID	= arduino
	AVRDUDE_PORT			= /dev/tty.usbserial*
endif

# nano (ATmega168)

ifeq ($(ARDUINO), mega2560)
	MCU						= atmega2560
	CLOCK					= 16000000
	BAUD					= 115200
	PROGRAMMER_MCU			= m2560
	AVRDUDE_PROGRAMMERID	= stk500v2
	AVRDUDE_PORT			= /dev/tty.usbmodem*
endif

ifeq ($(ARDUINO), mega)
	MCU						= atmega1280
	CLOCK					= 16000000
	BAUD					= 57600
	PROGRAMMER_MCU			= m1280
	AVRDUDE_PROGRAMMERID	= arduino
	AVRDUDE_PORT			= /dev/tty.usbserial*
endif

# AVR fuse calculator
# http://www.engbedded.com/fusecalc/
# Specify fuses
FUSES = -U hfuse:w:0x11:m -U lfuse:w:0xE4:m 





####################################################
#####                Config Done               #####
#####                                          #####
##### You shouldn't need to edit anything      #####
##### below to use the makefile but may wish   #####
##### to override a few of the flags           #####
##### nonetheless                              #####
#####                                          #####
####################################################

printf_min	= -Wl,-u,vfprintf -lprintf_min
printf_flt	= -Wl,-u,vfprintf -lprintf_flt -lm

scanf_min	= -Wl,-u,vfscanf -lscanf_flt -lm
scanf_flt	= -Wl,-u,vfscanf -lscanf_min -lm

math_lib	= -lm

##### Flags #####

ifeq ($(USE_MATH_LIB), true)
	LIBS += $(math_lib)
endif

ifeq ($(USE_PRINTF), true)
	ifeq ($(USE_PRINTF_FLT), true)
		LIBS += $(printf_flt)
	else
		LIBS += $(printf_min)
	endif
endif

ifeq ($(USE_SCANF), true)
	ifeq ($(USE_SCAN_FLT), true)
		LIBS += $(scanf_flt)
	else
		LIBS += $(scanf_min)
	endif
endif

# HEXFORMAT -- format for .hex file output
HEXFORMAT=ihex

# compiler
CFLAGS	= -DF_CPU=$(CLOCK)
CFLAGS += -I. $(INC) 
CFLAGS += -g -mmcu=$(MCU)
CFLAGS += -O$(OPTLEVEL)
CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
CFLAGS += -funsigned-bitfields
CFLAGS += -funsigned-char
CFLAGS += -Wall -Wstrict-prototypes
CFLAGS += -Wa,-ahlms=$(firstword $(filter %.lst, $(<:.c=.lst)))

# c++ specific flags
CPPFLAGS	= -DF_CPU=$(CLOCK)
CPPFLAGS   += -fno-exceptions
CPPFLAGS   += -Wa,-ahlms=$(firstword \
	$(filter %.lst, $(<:.cpp=.lst))  \
	$(filter %.lst, $(<:.cc=.lst))   \
	$(filter %.lst, $(<:.C=.lst)))

# assembler
ASMFLAGS	=-I. $(INC)
ASMFLAGS   += -mmcu=$(MCU)
ASMFLAGS   += -DF_CPU=$(CLOCK)
ASMFLAGS   += -x assembler-with-cpp
ASMFLAGS   += -Wa,-gstabs,-ahlms=$(firstword $(<:.S=.lst) $(<.s=.lst))


# linker
LDFLAGS		=-Wl,-Map,$(TRG).map
LDFLAGS	   += -mmcu=$(MCU)
LDFLAGS	   += $(LIBS)
	

##### executables ####
CC		= avr-gcc
OBJCOPY	= avr-objcopy
OBJDUMP	= avr-objdump
SIZE	= avr-size
AVRDUDE	= avrdude
REMOVE	= rm -f

##### automatic target names ####
TRG=$(PROJECTNAME).out
DUMPTRG=$(PROJECTNAME).s

HEXROMTRG=$(PROJECTNAME).hex 
HEXTRG=$(HEXROMTRG) $(PROJECTNAME).ee.hex
GDBINITFILE=gdbinit-$(PROJECTNAME)

# Define all object files.

# Start by splitting source files by type
#  C++
CPPFILES	= $(filter %.cpp, $(PRJSRC))
CCFILES		= $(filter %.cc, $(PRJSRC))
BIGCFILES	= $(filter %.C, $(PRJSRC))

#  C
CFILES		= $(filter %.c, $(PRJSRC))

#  Assembly
ASMFILES	= $(filter %.S, $(PRJSRC))


# List all object files we need to create
OBJDEPS		= $(CFILES:.c=.o)
OBJDEPS	   += $(CPPFILES:.cpp=.o)
OBJDEPS	   += $(BIGCFILES:.C=.o)
OBJDEPS	   += $(CCFILES:.cc=.o)
OBJDEPS    += $(ASMFILES:.S=.o)

# Define all lst files.
LST	= $(filter %.lst, $(OBJDEPS:.o=.lst))

# All the possible generated assembly 
# files (.s files)
GENASMFILES	= $(filter %.s, $(OBJDEPS:.o=.s)) 


.SUFFIXES : .c .cc .cpp .C .o .out .s .S \
	.hex .ee.hex .h .hh .hpp


.PHONY: writeflash clean stats gdbinit

# Make targets:
# all, disasm, stats, hex, writeflash/install, writefuse, gdbinit, clean
all: $(TRG)

disasm: $(DUMPTRG) stats

stats: $(TRG)
	$(OBJDUMP) -h $(TRG)
	$(SIZE) $(TRG) 

hex: $(HEXTRG)


writeflash: hex
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMMERID)   \
	 -p $(PROGRAMMER_MCU) -P $(AVRDUDE_PORT) -e -b $(BAUD) -D       \
	 -U flash:w:$(HEXROMTRG)

install: writeflash

writefuse:
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMMERID)   \
	 -p $(PROGRAMMER_MCU) -P $(AVRDUDE_PORT)        \
	 $(FUSES)

$(DUMPTRG): $(TRG) 
	$(OBJDUMP) -S  $< > $@


$(TRG): $(OBJDEPS) 
	$(CC) $(LDFLAGS) -o $(TRG) $(OBJDEPS)


#### Generating assembly ####
# asm from C
%.s: %.c
	$(CC) -S $(CFLAGS) $< -o $@

# asm from (hand coded) asm
%.s: %.S
	$(CC) -S $(ASMFLAGS) $< > $@


# asm from C++
.cpp.s .cc.s .C.s :
	$(CC) -S $(CFLAGS) $(CPPFLAGS) $< -o $@



#### Generating object files ####
# object from C
.c.o: 
	$(CC) $(CFLAGS) -c $< -o $@


# object from C++ (.cc, .cpp, .C files)
.cc.o .cpp.o .C.o :
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

# object from asm
.S.o :
	$(CC) $(ASMFLAGS) -c $< -o $@


#### Generating hex files ####
# hex files from elf
#####  Generating a gdb initialisation file    #####
.out.hex:
	$(OBJCOPY) -j .text                    \
		-j .data                       \
		-O $(HEXFORMAT) $< $@

.out.ee.hex:
	$(OBJCOPY) -j .eeprom                  \
		--change-section-lma .eeprom=0 \
		-O $(HEXFORMAT) $< $@


#####  Generating a gdb initialisation file    #####
##### Use by launching simulavr and avr-gdb:   #####
#####   avr-gdb -x gdbinit-myproject           #####
gdbinit: $(GDBINITFILE)

$(GDBINITFILE): $(TRG)
	@echo "file $(TRG)" > $(GDBINITFILE)
	
	@echo "target remote localhost:1212" \
		                >> $(GDBINITFILE)
	
	@echo "load"        >> $(GDBINITFILE) 
	@echo "break main"  >> $(GDBINITFILE)
	@echo "continue"    >> $(GDBINITFILE)
	@echo
	@echo "Use 'avr-gdb -x $(GDBINITFILE)'"


#### Cleanup ####
clean:
	$(REMOVE) $(TRG) $(TRG).map $(DUMPTRG)
	$(REMOVE) $(OBJDEPS)
	$(REMOVE) $(LST) $(GDBINITFILE)
	$(REMOVE) $(GENASMFILES)
	$(REMOVE) $(HEXTRG)
	


#####                    EOF                   #####

