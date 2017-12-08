CC = avr-gcc
OBJCOPY = avr-objcopy

TARGET = i2c_slave
OBJECTS = main.o usitwislave/usitwislave.o

## Chip and project-specific global definitions
MCU = attiny84
F_CPU = 8000000UL
BAUD = 9600UL
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I.

## Compiler/linker options
CFLAGS = -Os -g -std=gnu99 -Wall
CFLAGS += -ffunction-sections -fdata-sections

TARGET_ARCH = -mmcu=$(MCU)

LDFLAGS = -Wl,-Map,$(TARGET).map
LDFLAGS += -Wl,--gc-sections

AVR_PROGRAMMER = c232hm

## Targets and rules
all: $(TARGET).hex

flash: $(TARGET).hex
	avrdude -c $(AVR_PROGRAMMER) -p $(MCU) -U flash:w:$(TARGET).hex

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@

clean:
	rm -f $(TARGET).elf $(TARGET).hex $(TARGET).map $(OBJECTS)
