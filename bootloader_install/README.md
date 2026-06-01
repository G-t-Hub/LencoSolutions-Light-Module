# LencoLED Arduino Bootloader Installer

This folder is for the Arduino light module only. It is not the VESC Express `.vescpkg` package.

It installs the LencoLED CAN OTA bootloader on an ATmega328P light module using a USBASP/ISP programmer.

It includes two install paths:

- `install_current_app_and_bootloader.ps1` writes the included current Arduino light-module firmware plus the CAN OTA bootloader.
- `install_bootloader_preserve_app.ps1` preserves the firmware already on the module and only overlays the bootloader.

Included firmware files:

- `arduino_light_module_firmware_v3.0.hex` is used by the ISP installer script.
- `lencoled_can_bootloader.hex` is the custom CAN OTA bootloader.

## Requirements

- USBASP/ISP programmer connected to the light module ICSP pins.
- Arduino AVR tools installed by Arduino IDE.
- Target MCU: ATmega328P.

## Recommended Install

Use this for modules that do not already have the OTA-capable app:

```powershell
.\install_current_app_and_bootloader.ps1
```

The installer creates a timestamped backup under `backups/`, writes `arduino_light_module_firmware_v3.0.hex` plus `lencoled_can_bootloader.hex` as one merged flash image, restores EEPROM, and sets:

```text
lfuse = 0xFF
hfuse = 0xDA
efuse = 0xFD
```

`hfuse=0xDA` enables reset into the 2 KB bootloader section.

## Preserve Existing App

Use this only when the module already has a compatible OTA-capable Arduino light-module firmware and you want to add or replace only the bootloader:

Run:

```powershell
.\install_bootloader_preserve_app.ps1
```

The installer creates a timestamped backup under `backups/`, preserves the existing app flash, overlays `lencoled_can_bootloader.hex` at boot address `0x7800`, restores EEPROM, and sets:

```text
lfuse = 0xFF
hfuse = 0xDA
efuse = 0xFD
```

## Verify

Run:

```powershell
.\verify_bootloader.ps1
```

Verification reads the chip and checks that the bootloader bytes in flash match `lencoled_can_bootloader.hex`.

## Notes

Do not use Arduino IDE "Upload Using Programmer" or `arduino-cli upload -P usbasp` after installing this bootloader unless you intentionally want to replace the whole firmware. ISP uploads can erase the bootloader section. Normal future firmware updates should use the LencoLED OTA update UI.
