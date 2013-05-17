################################################################################
################################################################################
##                                                                            ##
## NOTE:  This makefile uses GNU make syntax.                                 ##
## =====  It may not work when called by other make utilities                 ##
##                                                                            ##
################################################################################
################################################################################

################################################################################
##     #########################################################################
## DEF #########################################################################
##     #########################################################################
################################################################################
# enter the product name for which you want to build the bootloader/monitor
# currently MB_128,MB_128_USBCAN are supported
#PRODUCT    = MB_128_USBCAN
PRODUCT    = MB_128

ifeq ($(PRODUCT),MB_128)
CPU = atmega128
#F_CPU=14745600UL
F_CPU=7372800UL
LDSECTION  = --section-start=.text=0x1F000
endif

ifeq ($(PRODUCT),MB_128_USBCAN)
CPU = at90can128
F_CPU=11095200UL 
LDSECTION  = --section-start=.text=0x1E000
endif

#PROJECT = bootloader_$(CPU)
PROJECT = stk500_2_boot

################################################################################
###         ####################################################################
### SOURCES ####################################################################
###         ####################################################################
################################################################################

OBJ  = stk500_2_boot.o

################################################################################
##       #######################################################################
## TOOLS #######################################################################
##       #######################################################################
################################################################################

COMPILER  = avr-gcc
ASSEMBLER = avr-gcc
LINKER    = avr-gcc
ROMIZER   = avr-objcopy
DUMPER    = avr-objdump

################################################################################
##       #######################################################################
## FLAGS #######################################################################
##       #######################################################################
################################################################################

COMMON_FLAGS  = -mmcu=$(CPU)
COMMON_FLAGS += -Os
COMMON_FLAGS += -mno-interrupts
COMMON_FLAGS += -funsigned-char
COMMON_FLAGS += -funsigned-bitfields
COMMON_FLAGS += -Wall
COMMON_FLAGS += -Wstrict-prototypes

LINK_FLAGS  = $(COMMON_FLAGS) -gstabs 

COMPILE_FLAGS  = $(LINK_FLAGS) -c -DF_CPU=$(F_CPU) -D$(PRODUCT) 
COMPILE_FLAGS += -Wa,-acdhlmns=$(<:.c=.lst)

ASSEMBLE_FLAGS  = $(COMMON_FLAGS) -c -I. -x assembler-with-cpp
ASSEMBLE_FLAGS += -Wa,-gstabs,-acdhlmns=$(<:.s=.lst)

EEPROM_FLAGS  = -j .eeprom
EEPROM_FLAGS += --change-section-lma .eeprom=0

FLASH_FLAGS  = -j .text
FLASH_FLAGS += -j .data

ELF_FLAGS  = -Wl,-Map=$(PROJECT).map
ELF_FLAGS += -Wl,$(LDSECTION)
ELF_FLAGS += --cref
ELF_FLAGS += -lm

################################################################################
##         #####################################################################
## ACTIONS #####################################################################
##         #####################################################################
################################################################################

COMPILE      = $(COMPILER) $(COMPILE_FLAGS)
ASSEMBLE     = $(ASSEMBLER) $(ASSEMBLE_FLAGS)
LINK         = $(LINKER) $(LINK_FLAGS)
REMOVE       = rm -f
COFFCONVERT  = $(ROMIZER) --debugging
COFFCONVERT += --change-section-address .data-0x800000
COFFCONVERT += --change-section-address .bss-0x800000
COFFCONVERT += --change-section-address .noinit-0x800000
COFFCONVERT += --change-section-address .eeprom-0x810000

################################################################################
##       #######################################################################
## BUILD #######################################################################
##       #######################################################################
################################################################################

all: elf flash eep list coff

rebuild: clean all

clean:
	$(REMOVE) $(PROJECT).hex
	$(REMOVE) $(PROJECT)_eeprom.hex
	$(REMOVE) $(PROJECT).elf
	$(REMOVE) $(PROJECT).map
	$(REMOVE) $(PROJECT).cof
	$(REMOVE) $(PROJECT).lst
	$(REMOVE) $(OBJ:.o=.lst)
	$(REMOVE) $(OBJ)

eep:     $(PROJECT)_eeprom.hex
flash:   $(PROJECT).hex
elf:     $(PROJECT).elf
list:    $(PROJECT).lst
coff:    $(PROJECT).cof

################################################################################
##      ########################################################################
## LINK ########################################################################
##      ########################################################################
################################################################################

$(PROJECT)_eeprom.hex: $(PROJECT).elf
	$(ROMIZER) $(EEPROM_FLAGS) -O ihex $(PROJECT).elf $(PROJECT)_eeprom.hex 

$(PROJECT).hex: $(PROJECT).elf
	$(ROMIZER) $(FLASH_FLAGS) -O ihex $(PROJECT).elf $(PROJECT).hex
#	cp $(PROJECT).hex c:\tmp

$(PROJECT).cof: $(PROJECT).elf
	$(COFFCONVERT) -O coff-avr $< $(PROJECT).cof

$(PROJECT).lst: $(PROJECT).elf
	$(DUMPER) -d -S $(PROJECT).elf > $(PROJECT).lst

$(PROJECT).elf: $(OBJ)
	$(LINK) $(OBJ) -o $(PROJECT).elf $(ELF_FLAGS)

################################################################################
##         #####################################################################
## COMPILE #####################################################################
##         #####################################################################
################################################################################

%.o : %.c
	$(COMPILE) $< -o $@

%.o : %.s
	$(ASSEMBLE) $< -o $@

