###############################################################################
# Makefile for the project stk500_2_boot
###############################################################################

## General Flags
PROJECT = stk500_2_boot
MCU = atmega128
TARGET = stk500_2_boot.elf
CC = avr-gcc.exe

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -mno-interrupts -Wstrict-prototypes  -gstabs                                -DF_CPU=7372800UL -Os -funsigned-char -funsigned-bitfields
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -nostartfiles  -Wl,--section-start=.text=0x1F000 -Wl,-Map=stk500_2_boot.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"C:\WinAVR-20080512\avr\include\avr" 

## Objects that must be built in order to link
OBJECTS = stk500_2_boot.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) stk500_2_boot.hex stk500_2_boot.eep stk500_2_boot.lss size

## Compile
stk500_2_boot.o: ../stk500_2_boot.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) stk500_2_boot.elf dep/* stk500_2_boot.hex stk500_2_boot.eep stk500_2_boot.lss stk500_2_boot.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

