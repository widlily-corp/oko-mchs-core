@echo off
chcp 65001 >nul
echo ========================================================
echo    ОКО МЧС: ЗАПУСК 3D-СИМУЛЯЦИИ GAZEBO 11 С АРЕНОЙ ЧС
echo ========================================================
echo 1. Очистка старых процессов...
wsl.exe -d Ubuntu-22.04 bash -c "killall -9 gazebo gzserver gzclient 2>/dev/null || true"

echo 2. Запуск 3D-арены Gazebo с дроном и сенсорами...
wsl.exe -d Ubuntu-22.04 /usr/local/bin/gazebo-mchs
pause
