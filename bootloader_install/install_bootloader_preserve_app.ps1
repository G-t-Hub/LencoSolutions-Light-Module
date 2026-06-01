$ErrorActionPreference = "Stop"

$ToolsRoot = Join-Path $env:LOCALAPPDATA "Arduino15\packages\arduino\tools"
$Avrdude = Get-ChildItem -Path $ToolsRoot -Recurse -Filter avrdude.exe | Sort-Object FullName -Descending | Select-Object -First 1
$Objcopy = Get-ChildItem -Path $ToolsRoot -Recurse -Filter avr-objcopy.exe | Sort-Object FullName -Descending | Select-Object -First 1

if (-not $Avrdude) { throw "avrdude.exe not found under $ToolsRoot" }
if (-not $Objcopy) { throw "avr-objcopy.exe not found under $ToolsRoot" }

$AvrdudeConf = Join-Path $Avrdude.Directory.Parent.FullName "etc\avrdude.conf"
if (-not (Test-Path $AvrdudeConf)) { throw "avrdude.conf not found at $AvrdudeConf" }

$BootHex = Join-Path $PSScriptRoot "lencoled_can_bootloader.hex"
if (-not (Test-Path $BootHex)) { throw "Bootloader hex not found at $BootHex" }

$Stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$BackupDir = Join-Path $PSScriptRoot "backups\install_$Stamp"
New-Item -ItemType Directory -Force -Path $BackupDir | Out-Null

function Run-Checked($File, [string[]] $ArgumentList, $Message) {
    & $File @ArgumentList
    if ($LASTEXITCODE -ne 0) {
        throw "$Message failed with exit code $LASTEXITCODE"
    }
}

Write-Host "Reading current flash, EEPROM, and fuses..."
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

Run-Checked $Objcopy.FullName @("-I", "ihex", "-O", "binary", "$BackupDir\before_flash.hex", "$BackupDir\before_flash.bin") "Flash conversion"
Run-Checked $Objcopy.FullName @("-I", "ihex", "-O", "binary", $BootHex, "$BackupDir\bootloader.bin") "Bootloader conversion"

$Flash = [System.IO.File]::ReadAllBytes("$BackupDir\before_flash.bin")
if ($Flash.Length -lt 32768) {
    $Padded = New-Object byte[] 32768
    for ($i = 0; $i -lt $Padded.Length; $i++) { $Padded[$i] = 0xFF }
    [Array]::Copy($Flash, $Padded, $Flash.Length)
    $Flash = $Padded
}

$Boot = [System.IO.File]::ReadAllBytes("$BackupDir\bootloader.bin")
if ($Boot.Length -gt 2048) {
    throw "Bootloader is too large for 2 KB boot section: $($Boot.Length) bytes"
}

[Array]::Copy($Boot, 0, $Flash, 0x7800, $Boot.Length)
[System.IO.File]::WriteAllBytes("$BackupDir\merged_flash.bin", $Flash)
Run-Checked $Objcopy.FullName @("-I", "binary", "-O", "ihex", "--change-addresses", "0", "$BackupDir\merged_flash.bin", "$BackupDir\merged_flash.hex") "Merged flash conversion"

Write-Host "Writing merged flash, restoring EEPROM, and setting fuses..."
Run-Checked $Avrdude.FullName @(
    "-C", $AvrdudeConf,
    "-p", "m328p",
    "-c", "usbasp",
    "-U", "flash:w:$BackupDir\merged_flash.hex:i",
    "-U", "eeprom:w:$BackupDir\before_eeprom.hex:i",
    "-U", "lfuse:w:0xFF:m",
    "-U", "hfuse:w:0xDA:m",
    "-U", "efuse:w:0xFD:m"
) "Bootloader install"

Write-Host "Install complete. Backup directory:"
Write-Host $BackupDir
