param([string[]]$IdfArgs = @('build'))
$ErrorActionPreference = 'Stop'
# Use the installed 6.1 toolchain. Environment changes affect this process only.
$env:PROCESSOR_ARCHITECTURE = 'AMD64'
$env:PYTHONUTF8 = '1'
$env:IDF_PATH = 'G:\Software\Espressif6\.espressif\v6.1\esp-idf'
$env:IDF_TOOLS_PATH = 'C:\Espressif\tools'
$env:IDF_PYTHON_ENV_PATH = 'C:\Espressif\tools\python\v6.1\venv'
$env:ESP_IDF_VERSION = '6.1'
$env:IDF_VERSION = '6.1.0'
$env:IDF_CCACHE_ENABLE = '0'
$env:ESP_ROM_ELF_DIR = 'C:\Espressif\tools\esp-rom-elfs\20241011'
$env:IDF_COMPONENT_LOCAL_STORAGE_URL = 'file://C:/Espressif/tools'
$env:PATH = @(
    'C:\Espressif\tools\python\v6.1\venv\Scripts',
    'C:\Espressif\tools\cmake\4.0.3\bin',
    'C:\Espressif\tools\ninja\1.12.1',
    'C:\Espressif\tools\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin',
    'C:\Program Files\Git\cmd', 'C:\Windows\System32', 'C:\Windows'
) -join ';'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    & "$env:IDF_PYTHON_ENV_PATH\Scripts\python.exe" "$env:IDF_PATH\tools\idf.py" @IdfArgs
    exit $LASTEXITCODE
} finally { Pop-Location }
