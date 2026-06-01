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
$VerifyDir = Join-Path $PSScriptRoot "backups\verify_$Stamp"
New-Item -ItemType Directory -Force -Path $VerifyDir | Out-Null

function Run-Checked($File, [string[]] $ArgumentList, $Message) {
    & $File @ArgumentList
    if ($LASTEXITCODE -ne 0) {
        throw "$Message failed with exit code $LASTEXITCODE"
    }
}

Write-Host "Reading flash and fuses..."
Run-Checked $Avrdude.FullName @(
    "-C", $AvrdudeConf,
    "-p", "m328p",
    "-c", "usbasp",
    "-U", "flash:r:$VerifyDir\flash.hex:i",
    "-U", "lfuse:r:$VerifyDir\lfuse.hex:i",
    "-U", "hfuse:r:$VerifyDir\hfuse.hex:i",
    "-U", "efuse:r:$VerifyDir\efuse.hex:i"
) "Chip read"

Run-Checked $Objcopy.FullName @("-I", "ihex", "-O", "binary", "$VerifyDir\flash.hex", "$VerifyDir\flash.bin") "Flash conversion"
Run-Checked $Objcopy.FullName @("-I", "ihex", "-O", "binary", $BootHex, "$VerifyDir\bootloader.bin") "Bootloader conversion"

$Flash = [System.IO.File]::ReadAllBytes("$VerifyDir\flash.bin")
$Boot = [System.IO.File]::ReadAllBytes("$VerifyDir\bootloader.bin")
$Match = 0
for ($i = 0; $i -lt $Boot.Length; $i++) {
    if ($Flash[0x7800 + $i] -eq $Boot[$i]) { $Match++ }
}

Write-Host "Bootloader match: $Match/$($Boot.Length)"
Write-Host "Verification files:"
Write-Host $VerifyDir

if ($Match -ne $Boot.Length) {
    throw "Bootloader verification failed"
}

Write-Host "Bootloader verification passed."
