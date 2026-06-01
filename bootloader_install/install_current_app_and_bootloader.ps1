$ErrorActionPreference = "Stop"

$ToolsRoot = Join-Path $env:LOCALAPPDATA "Arduino15\packages\arduino\tools"
$Avrdude = Get-ChildItem -Path $ToolsRoot -Recurse -Filter avrdude.exe | Sort-Object FullName -Descending | Select-Object -First 1
$Objcopy = Get-ChildItem -Path $ToolsRoot -Recurse -Filter avr-objcopy.exe | Sort-Object FullName -Descending | Select-Object -First 1

if (-not $Avrdude) { throw "avrdude.exe not found under $ToolsRoot" }
if (-not $Objcopy) { throw "avr-objcopy.exe not found under $ToolsRoot" }

$AvrdudeConf = Join-Path $Avrdude.Directory.Parent.FullName "etc\avrdude.conf"
if (-not (Test-Path $AvrdudeConf)) { throw "avrdude.conf not found at $AvrdudeConf" }

$FirmwareHex = Join-Path $PSScriptRoot "arduino_light_module_firmware_v3.0.hex"
$BootHex = Join-Path $PSScriptRoot "lencoled_can_bootloader.hex"
if (-not (Test-Path $FirmwareHex)) { throw "Firmware hex not found at $FirmwareHex" }
if (-not (Test-Path $BootHex)) { throw "Bootloader hex not found at $BootHex" }

$Stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$BackupDir = Join-Path $PSScriptRoot "backups\install_app_bootloader_$Stamp"
New-Item -ItemType Directory -Force -Path $BackupDir | Out-Null

function Run-Checked($File, [string[]] $ArgumentList, $Message) {
    & $File @ArgumentList
    if ($LASTEXITCODE -ne 0) {
        throw "$Message failed with exit code $LASTEXITCODE"
    }
}

Write-Host "Reading current EEPROM and fuses for backup..."
Run-Checked $Avrdude.FullName @(
    "-C", $AvrdudeConf,
    "-p", "m328p",
    "-c", "usbasp",
    "-U", "flash:r:$BackupDir\before_flash.hex:i",
    "-U", "eeprom:r:$BackupDir\before_eeprom.hex:i",
    "-U", "lfuse:r:$BackupDir\before_lfuse.hex:i",
    "-U", "hfuse:r:$BackupDir\before_hfuse.hex:i",
    "-U", "efuse:r:$BackupDir\before_efuse.hex:i",
    "-U", "lock:r:$BackupDir\before_lock.hex:i"
) "Initial chip read"

Run-Checked $Objcopy.FullName @("-I", "ihex", "-O", "binary", $FirmwareHex, "$BackupDir\firmware.bin") "Firmware conversion"
Run-Checked $Objcopy.FullName @("-I", "ihex", "-O", "binary", $BootHex, "$BackupDir\bootloader.bin") "Bootloader conversion"

$Flash = New-Object byte[] 32768
for ($i = 0; $i -lt $Flash.Length; $i++) { $Flash[$i] = 0xFF }

$Firmware = [System.IO.File]::ReadAllBytes("$BackupDir\firmware.bin")
$Boot = [System.IO.File]::ReadAllBytes("$BackupDir\bootloader.bin")

if ($Firmware.Length -gt 0x7800) {
    throw "Firmware is too large and would overlap the bootloader section: $($Firmware.Length) bytes"
}

if ($Boot.Length -gt 2048) {
    throw "Bootloader is too large for 2 KB boot section: $($Boot.Length) bytes"
}

[Array]::Copy($Firmware, 0, $Flash, 0, $Firmware.Length)
[Array]::Copy($Boot, 0, $Flash, 0x7800, $Boot.Length)
[System.IO.File]::WriteAllBytes("$BackupDir\merged_firmware_bootloader.bin", $Flash)
Run-Checked $Objcopy.FullName @("-I", "binary", "-O", "ihex", "--change-addresses", "0", "$BackupDir\merged_firmware_bootloader.bin", "$BackupDir\merged_firmware_bootloader.hex") "Merged flash conversion"

Write-Host "Writing current firmware + bootloader, restoring EEPROM, and setting fuses..."
Run-Checked $Avrdude.FullName @(
    "-C", $AvrdudeConf,
    "-p", "m328p",
    "-c", "usbasp",
    "-U", "flash:w:$BackupDir\merged_firmware_bootloader.hex:i",
    "-U", "eeprom:w:$BackupDir\before_eeprom.hex:i",
    "-U", "lfuse:w:0xFF:m",
    "-U", "hfuse:w:0xDA:m",
    "-U", "efuse:w:0xFD:m"
) "Firmware and bootloader install"

Write-Host "Install complete. Backup directory:"
Write-Host $BackupDir
