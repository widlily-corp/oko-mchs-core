@echo off
chcp 65001 >nul
pushd "%~dp0"
echo ========================================================
echo       КОМПЛЕКС «ОКО МЧС» — ЕДИНЫЙ ЗАПУСК СИСТЕМЫ
echo ========================================================
echo 1. Очистка старых процессов...
wsl.exe -d Ubuntu-22.04 bash -c "killall -9 gazebo gzserver gzclient python3 2>/dev/null || true"

echo 2. Запуск симуляции, зрения и автопилота в WSL...
start "ОКО МЧС - СИМУЛЯЦИЯ И АВТОПИЛОТ (WSL)" wsl.exe -d Ubuntu-22.04 bash -c "bash /root/oko-mchs-core/scripts/run_all.sh"

echo 3. Ожидание инициализации узлов (5 сек)...
timeout /t 5 /nobreak >nul

echo 4. Запуск АРМ Оператора ЦУКС на Windows...
start "ОКО МЧС - АРМ ОПЕРАТОРА" python src\mchs_ui\app.py

echo ========================================================
echo >>> КОМПЛЕКС ЗАПУЩЕН!
echo - 3D-Арена Gazebo: запущена в WSLg (1024x768)
echo - Нода зрения: анализирует 4 типа ЧС + веб-эфир :8080
echo - Автопилот: FSM патрулирование, зависание и протокол
echo - АРМ Оператора МЧС: активен на Windows
echo - Веб-эфир камеры БПЛА: http://localhost:8080
echo ========================================================
popd
