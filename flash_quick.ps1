Write-Host "=========================================================="
Write-Host " Gravando nova tabela de particoes (3KB) na COM5...       "
Write-Host " Segure BOOT e de 1 clique em RST para gravar!            "
Write-Host "=========================================================="

for ($i = 0; $i -lt 80; $i++) {
    $out = py -3.11 -m esptool --chip esp32 --port COM5 --baud 115200 write-flash 0x8000 .pio\build\esp32_wroom_oled\partitions.bin 2>&1 | Out-String
    if ($out -match "Hash of data verified") {
        Write-Host ""
        Write-Host "==========================================================" -ForegroundColor Green
        Write-Host ">>> PARTICOES GRAVADAS COM SUCESSO!                     <<<" -ForegroundColor Green
        Write-Host "Agora solte o BOOT e dê 1 clique em RST para iniciar!   <<<" -ForegroundColor Green
        Write-Host "==========================================================" -ForegroundColor Green
        exit 0
    }
    Write-Host "." -NoNewline
    Start-Sleep -Milliseconds 400
}

Write-Host ""
Write-Host "Tempo limite esgotado." -ForegroundColor Yellow
exit 1
