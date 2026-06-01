# LencoLED CAN Bootloader

Experimental ATmega328P CAN bootloader for the LencoLED light module.

This is not a normal Arduino sketch. Build/link it for the boot flash section and install it with ISP before enabling `BOOTRST`.

## Target

- MCU: ATmega328P
- Clock: 16 MHz
- CAN controller: MCP2515, 8 MHz crystal, CS on D10/PB2
- CAN bitrate: 500 kbps
- Node ID: 36
- OTA command extended CAN ID: `(241 << 8) | 36`
- OTA status extended CAN ID: `(242 << 8) | 36`
- Boot section: 2 KB, start byte address `0x7800`

## Fuses

Current no-bootloader test state:

```text
lfuse = 0xFF
hfuse = 0xDB
efuse = 0xFD
```

For a 2 KB bootloader with reset into bootloader:

```text
lfuse = 0xFF
hfuse = 0xDA
efuse = 0xFD
```

Do not write fuses until the bootloader image is built and installed.

## Build Check

The optimized prototype was compiled with Arduino's bundled `avr-gcc`, linked at `0x7800`, and measured:

```text
text = 1874 bytes
bss  = 142 bytes
```

This fits the 2 KB boot section.
