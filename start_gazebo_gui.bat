@echo off
chcp 65001 >nul
echo ========================================================
echo    ОКО МЧС: ПРЯМОЙ ЗАПУСК 3D-СИМУЛЯЦИИ GAZEBO 11
echo ========================================================
echo [1/3] Очистка предыдущих процессов...
wsl.exe -d Ubuntu-22.04 bash -c "killall -9 gzserver gzclient gazebo 2>/dev/null || true"

echo [2/3] Подготовка конфигурации графики и моделей...
wsl.exe -d Ubuntu-22.04 bash -c "mkdir -p /root/.gazebo; cat << 'EOF' > /root/.gazebo/gui.ini
[geometry]
x=50
y=50
width=1024
height=768

[camera]
x=0
y=-25
z=15
roll=0
pitch=0.6
yaw=1.57
EOF"

echo [3/3] Запуск 3D-симулятора Gazebo 11...
wsl.exe -d Ubuntu-22.04 bash -c "source /opt/ros/humble/setup.bash; export DISPLAY=:0; export LIBGL_ALWAYS_SOFTWARE=1; export GALLIUM_DRIVER=llvmpipe; export GAZEBO_MODEL_DATABASE_URI=''; export GAZEBO_MODEL_PATH='/root/oko-mchs-core/src/mchs_simulation/models:/mnt/c/Users/Widlily/.gemini/antigravity/scratch/oko-mchs-core/src/mchs_simulation/models:/usr/share/gazebo-11/models'; gazebo --verbose /mnt/c/Users/Widlily/.gemini/antigravity/scratch/oko-mchs-core/src/mchs_simulation/worlds/mchs_emergency_arena.world -s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
pause
