Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "   ОКО МЧС: ЗАПУСК 3D-СИМУЛЯЦИИ GAZEBO 11               " -ForegroundColor Green
Write-Host "========================================================" -ForegroundColor Cyan
wsl.exe -d Ubuntu-22.04 bash -c "killall -9 gazebo gzserver gzclient 2>/dev/null || true"
wsl.exe -d Ubuntu-22.04 /usr/local/bin/gazebo-mchs
