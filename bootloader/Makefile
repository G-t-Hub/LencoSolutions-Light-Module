MCU = atmega328p
F_CPU = 16000000UL
BOOT_START = 0x7800
AVR_TOOLS = C:/Users/gareth.timm/AppData/Local/Arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/bin

CC = $(AVR_TOOLS)/avr-gcc
OBJCOPY = $(AVR_TOOLS)/avr-objcopy
SIZE = $(AVR_TOOLS)/avr-size

CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu99 -ffunction-sections -fdata-sections -fno-inline-small-functions -Wall -Wextra
LDFLAGS = -mmcu=$(MCU) -Wl,--gc-sections -Wl,--section-start=.text=$(BOOT_START)

all: lencoled_can_bootloader.hex

lencoled_can_bootloader.elf: lencoled_can_bootloader.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<
	$(SIZE) $@

lencoled_can_bootloader.hex: lencoled_can_bootloader.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

clean:
	del /Q lencoled_can_bootloader.elf lencoled_can_bootloader.hex 2>NUL || exit 0
