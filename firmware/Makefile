# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# SRCS ......... The source files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

GCC_MCU        = atmega328p
AVRDUDE_DEVICE = m328p
CLOCK          = 8000000
PROGRAMMER     = -C avrdude.conf -c pi_spi -P /dev/spidev0.0 -b 200000
SRCS           = kamikaze.cc
FUSES          = -U lfuse:w:0x64:m -U hfuse:w:0xdd:m -U efuse:w:0xff:m


######################################################################
######################################################################

# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude $(PROGRAMMER) -p $(AVRDUDE_DEVICE)
CXX = avr-g++
CPPFLAGS = -Wall -Werror -pedantic -Wextra -Os
CPPFLAGS += -DF_CPU=$(CLOCK) -mmcu=$(GCC_MCU) -I /usr/lib/avr/include
CXXFLAGS = -std=c++11
COMPILE = $(CXX) $(CPPFLAGS) $(CXXFLAGS)

# symbolic targets:
all:	main.hex

flash:	main.hex
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

install: flash

clean:
	rm -f main.hex main.elf

main.elf: $(SRCS) Makefile
	$(COMPILE) -o main.elf $(SRCS)

main.hex: main.elf
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E $(SRCS)
