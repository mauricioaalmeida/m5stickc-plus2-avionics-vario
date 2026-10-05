$port = "COM5"
$bootloader = ".pio\build\esp32_wroom_oled\bootloader.bin"
$partitions = ".pio\build\esp32_wroom_oled\partitions.bin"
$boot_app0 = "C:\Users\Mauricio\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin"
$firmware = ".pio\build\esp32_wroom_oled\firmware.bin"
$littlefs = ".pio\build\esp32_wroom_oled\littlefs.bin"

Write-Host "=========================================================="
Write-Host " Gravador ESP32 - Vario BMP280 OLED                       "
Write-Host " Aguardando ESP32 entrar em Download Mode na porta $port... "
Write-Host " (Mantenha o botao BOOT pressionado e dê 1 clique em RST) "
Write-Host "=========================================================="

$connected = $false
for ($i = 0; $i -lt 120; $i++) {
    $out = py -3.11 -m esptool --chip esp32 --port $port chip-id 2>&1 | Out-String
    if ($out -match "Chip is ESP32" -or $out -match "Detecting chip... ESP32") {
        Write-Host ""
        Write-Host ">>> ESP32 detectado em modo de gravacao!" -ForegroundColor Green
        $connected = $true
        break
    }
    Write-Host "." -NoNewline
    Start-Sleep -Milliseconds 600
}

Write-Host ""
if (-not $connected) {
    Write-Host "Timeout aguardando modo de download." -ForegroundColor Yellow
    exit 1
}

Write-Host "Gravando firmware (app0 + app1)..." -ForegroundColor Cyan
py -3.11 -m esptool --chip esp32 --port $port --baud 115200 write-flash -z --flash-mode dio --flash-freq 40m `
    0x10000 $firmware `
    0x1f0000 $firmware

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "==========================================================" -ForegroundColor Green
    Write-Host " GRAVACAO CONCLUIDA COM SUCESSO!                         " -ForegroundColor Green
    Write-Host " Agora solte o BOOT e dê 1 clique em RST para iniciar!    " -ForegroundColor Green
    Write-Host "==========================================================" -ForegroundColor Green
} else {
    Write-Host "Falha na gravacao." -ForegroundColor Red
}
