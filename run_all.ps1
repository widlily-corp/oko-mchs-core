$PSScriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $PSScriptRoot
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "       КОМПЛЕКС «ОКО МЧС» — ЕДИНЫЙ ЗАПУСК              " -ForegroundColor Green
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "1. Очистка старых процессов..." -ForegroundColor Yellow
wsl.exe -d Ubuntu-22.04 bash -c "killall -9 gazebo gzserver gzclient python3 2>/dev/null || true"

Write-Host "2. Запуск симуляции, зрения и автопилота в WSL..." -ForegroundColor Yellow
Start-Process wsl.exe -ArgumentList "-d Ubuntu-22.04 bash -c 'MCHS_NO_UI=1 bash /root/oko-mchs-core/scripts/run_all.sh'"

Write-Host "3. Ожидание старта узлов (5 сек)..." -ForegroundColor Yellow
Start-Sleep -Seconds 5

Write-Host "4. Запуск АРМ Оператора ЦУКС на Windows..." -ForegroundColor Yellow
Start-Process python -ArgumentList '\\wsl.localhost\Ubuntu-22.04\root\oko-mchs-core\src\mchs_ui\app.py'

Write-Host "========================================================" -ForegroundColor Cyan
Write-Host ">>> ВСЕ СИСТЕМЫ АКТИВНЫ!" -ForegroundColor Green
Write-Host "- 3D-Арена Gazebo: в окне WSLg"
Write-Host "- Пульт оператора МЧС: открыт на Windows"
Write-Host "- Прямой веб-эфир камеры БПЛА: http://localhost:8080"
Write-Host "========================================================" -ForegroundColor Cyan
