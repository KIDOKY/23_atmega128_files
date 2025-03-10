###############################################################################
# Makefile for the project CDS_sensor_ADC
###############################################################################

## General Flags
PROJECT = CDS_sensor_ADC
MCU = atmega128
TARGET = TMP_sensor_ADC_HW.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99         -DF_CPU=16000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=TMP_sensor_ADC_HW.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = lcd.o TMP_sensor_ADC_HW.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) TMP_sensor_ADC_HW.hex TMP_sensor_ADC_HW.eep TMP_sensor_ADC_HW.lss size

## Compile
lcd.o: ../lcd.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

TMP_sensor_ADC_HW.o: ../TMP_sensor_ADC_HW.c
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
	-rm -rf $(OBJECTS) TMP_sensor_ADC_HW.elf dep/* TMP_sensor_ADC_HW.hex TMP_sensor_ADC_HW.eep TMP_sensor_ADC_HW.lss TMP_sensor_ADC_HW.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

