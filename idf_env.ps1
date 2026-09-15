param([string]$IdfAction = "build", [string]$Port = "")
$env:IDF_PATH='C:\Users\SM\.espressif\v6.0.2\esp-idf'
$env:IDF_TOOLS_PATH='C:\Espressif\tools'
$env:IDF_PYTHON_ENV_PATH='C:\Espressif\tools\python_env\idf6.0_py3.11_env'
$env:ESP_ROM_ELF_DIR='C:\Espressif\tools\esp-rom-elfs\20241011'
$env:ESP_IDF_VERSION='6.0.2'
$env:PYTHONDONTWRITEBYTECODE='1'
$env:PATH='C:\Espressif\tools\cmake\4.0.3\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\xtensa-esp-elf\esp-15.2.0_20251204\xtensa-esp-elf\bin;' + $env:PATH
$py = 'C:\Espressif\tools\python_env\idf6.0_py3.11_env\Scripts\python.exe'
if ($Port -ne "") {
    & $py "$env:IDF_PATH\tools\idf.py" -p $Port ($IdfAction -split ' ')
} else {
    & $py "$env:IDF_PATH\tools\idf.py" ($IdfAction -split ' ')
}
