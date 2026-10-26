<#
.SYNOPSIS
Clears, builds, and flashes the ESP32-C3 BLE fixture with ESP-IDF v5.5.1.

.PARAMETER Port
Serial port used by the ESP32-C3 board.

.PARAMETER IdfPath
Root directory of the ESP-IDF v5.5.1 installation.

.PARAMETER Variant
Compile-time BLE fixture profile: 0 for Ray7, 1 for Netgate-C3-Device1.

.OUTPUTS
None. Stops with a non-zero PowerShell error when any ESP-IDF command fails.
#>
[CmdletBinding()]
param(
    [ValidatePattern('^COM[0-9]+$')]
    [string]$Port = 'COM9',

    [ValidateScript({ Test-Path -LiteralPath $_ -PathType Container })]
    [string]$IdfPath = 'C:\ESPIDF\ESP551\Espressif\frameworks\esp-idf-v5.5.1',

    [ValidateSet(0, 1)]
    [int]$Variant = 0
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repositoryRoot = Split-Path -Parent $PSScriptRoot
$fixtureRoot = Join-Path $repositoryRoot 'test_devices\esp32c3_ble_fixture'
$exportScript = Join-Path $IdfPath 'export.ps1'
$idfInstallRoot = Split-Path -Parent (Split-Path -Parent $IdfPath)
$idfToolsRoot = Join-Path $idfInstallRoot 'tools'
$idfPythonEnv = Join-Path $idfInstallRoot 'python_env\idf5.5_py3.11_env'

if (-not (Test-Path -LiteralPath $fixtureRoot -PathType Container)) {
    throw "ESP32-C3 fixture project was not found: $fixtureRoot"
}
if (-not (Test-Path -LiteralPath $exportScript -PathType Leaf)) {
    throw "ESP-IDF export script was not found: $exportScript"
}
if (-not (Test-Path -LiteralPath $idfToolsRoot -PathType Container)) {
    throw "ESP-IDF tools directory was not found: $idfToolsRoot"
}
if (-not (Test-Path -LiteralPath $idfPythonEnv -PathType Container)) {
    throw "ESP-IDF Python environment was not found: $idfPythonEnv"
}

Push-Location -LiteralPath $fixtureRoot
try {
    # Keep this build independent from an ESP-IDF installation left in the caller environment.
    $env:IDF_PATH = $IdfPath
    $env:IDF_TOOLS_PATH = $idfInstallRoot
    $env:IDF_PYTHON_ENV_PATH = $idfPythonEnv
    . $exportScript

    if ($LASTEXITCODE -ne 0) {
        throw "ESP-IDF environment activation failed with exit code $LASTEXITCODE"
    }

    Write-Host "[NETGATE] Project: $fixtureRoot"
    Write-Host "[NETGATE] ESP-IDF: $IdfPath"
    Write-Host "[NETGATE] Target: esp32c3, port: $Port, fixture variant: $Variant"

    $cmakeVariantArgs = @("-DNETGATE_C3_FIXTURE_VARIANT=$Variant")

    & idf.py @cmakeVariantArgs set-target esp32c3
    if ($LASTEXITCODE -ne 0) {
        throw "idf.py set-target esp32c3 failed with exit code $LASTEXITCODE"
    }

    & idf.py @cmakeVariantArgs fullclean
    if ($LASTEXITCODE -ne 0) {
        throw "idf.py fullclean failed with exit code $LASTEXITCODE"
    }

    & idf.py @cmakeVariantArgs build
    if ($LASTEXITCODE -ne 0) {
        throw "idf.py build failed with exit code $LASTEXITCODE"
    }

    & idf.py @cmakeVariantArgs -p $Port flash
    if ($LASTEXITCODE -ne 0) {
        throw "idf.py flash failed with exit code $LASTEXITCODE"
    }

    Write-Host '[NETGATE] ESP32-C3 fixture build and flash completed successfully.'
}
finally {
    Pop-Location
}
