Set-Location $PSScriptRoot

$RepoWsl = (& wsl.exe -d Ubuntu-22.04 wslpath -u ($PSScriptRoot -replace '\\','/') 2>$null | Out-String).Trim()
if (-not $RepoWsl) {
    Write-Error "Не удалось определить путь к репозиторию в WSL. Проверьте WSL2 + Ubuntu-22.04."
    exit 1
}

# wsl.exe не умеет стартовать из UNC-каталога — уходим в локальный TEMP.
Set-Location $env:TEMP

Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "       КОМПЛЕКС «ОКО МЧС» — ЕДИНЫЙ ЗАПУСК              " -ForegroundColor Green
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "1. Очистка старых процессов..." -ForegroundColor Yellow
wsl.exe -d Ubuntu-22.04 bash -c "pkill -9 -x gzserver 2>/dev/null || true; pkill -9 -f '[m]chs_vision_node.py' 2>/dev/null || true; pkill -9 -f '[m]chs_pilot_route.py' 2>/dev/null || true"

Write-Host "2. Запуск симуляции, зрения и автопилота в WSL..." -ForegroundColor Yellow
$WslArgs = "-d Ubuntu-22.04 --cd `"$RepoWsl`" -- bash -lc `"MCHS_NO_UI=1 bash scripts/run_all.sh`""
Start-Process wsl.exe -ArgumentList $WslArgs

Write-Host "3. Ожидание старта узлов (8 сек)..." -ForegroundColor Yellow
Start-Sleep -Seconds 8

Write-Host "4. Запуск АРМ Оператора ЦУКС на Windows..." -ForegroundColor Yellow
Start-Process python -ArgumentList (Join-Path $PSScriptRoot "src\mchs_ui\app.py")

Write-Host "========================================================" -ForegroundColor Cyan
Write-Host ">>> ВСЕ СИСТЕМЫ АКТИВНЫ!" -ForegroundColor Green
Write-Host "- 3D-Арена Gazebo: в окне WSLg"
Write-Host "- Пульт оператора МЧС: открыт на Windows"
Write-Host "- Прямой веб-эфир камеры БПЛА: http://localhost:8080"
Write-Host "========================================================" -ForegroundColor Cyan
