param (
    [string]$port = "COM8",
    [int]$baud = 1500000
)

$binFile = Join-Path $PSScriptRoot "m5stickc_plus2_vario_m5burner.bin"

if (-not (Test-Path $binFile)) {
    Write-Host "Arquivo $binFile nao encontrado. Gerando..." -ForegroundColor Yellow
    py -3.11 (Join-Path $PSScriptRoot "generate_m5burner_bin.py")
}

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host " Gravador M5StickC Plus2 - Avionics Vario (M5Burner Image) " -ForegroundColor Cyan
Write-Host " Porta: $port | Baud: $baud" -ForegroundColor Cyan
Write-Host " Arquivo: $binFile (Offset: 0x0)" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

py -3.11 -m esptool --chip esp32 --port $port --baud $baud write-flash -z --flash-mode dio --flash-freq 40m --flash-size 8MB 0x0 $binFile

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "==========================================================" -ForegroundColor Green
    Write-Host " GRAVACAO CONCLUIDA COM SUCESSO!                          " -ForegroundColor Green
    Write-Host " O M5StickC Plus2 reiniciara com o Vario Avionics!        " -ForegroundColor Green
    Write-Host "==========================================================" -ForegroundColor Green
} else {
    Write-Host ""
    Write-Host "Falha na gravacao. Verifique a porta COM e a conexao USB." -ForegroundColor Red
}
