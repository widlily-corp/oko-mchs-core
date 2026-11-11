@echo off
chcp 65001 >nul
echo ========================================================
echo       КОМПЛЕКС «ОКО МЧС» — ЕДИНЫЙ ЗАПУСК СИСТЕМЫ
echo ========================================================
echo 1. Очистка старых процессов...
wsl.exe -d Ubuntu-22.04 bash -c "killall -9 gzserver gzclient gazebo python3 2>/dev/null || true"

echo 2. Запуск симуляции и нод ROS 2 в WSL...
start "ОКО МЧС - СИМУЛЯЦИЯ И АВТОПИЛОТ (WSL)" wsl.exe -d Ubuntu-22.04 bash -c "cd /mnt/c/Users/Widlily/.gemini/antigravity/scratch/oko-mchs-core && bash scripts/run_all.sh"

echo 3. Ожидание инициализации узлов (5 сек)...
timeout /t 5 /nobreak >nul

echo 4. Запуск АРМ Оператора ЦУКС на Windows...
start "ОКО МЧС - АРМ ОПЕРАТОРА" python src\mchs_ui\app.py

echo ========================================================
echo >>> КОМПЛЕКС ЗАПУЩЕН!
echo - 3D-Арена Gazebo: выводится на экран (WSLg)
echo - АРМ Оператора МЧС: активен на Windows
echo - Веб-эфир камеры БПЛА: http://localhost:8080
echo ========================================================
