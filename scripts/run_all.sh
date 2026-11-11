#!/usr/bin/env bash
# Единый запуск полного демонстрационного комплекса «Око МЧС» с графическими окнами
set -e

echo "=========================================================="
echo "   ЗАПУСК АВТОНОМНОГО КОМПЛЕКСА МОНИТОРИНГА «ОКО МЧС»     "
echo "=========================================================="

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
REPO_DIR=$(dirname "$SCRIPT_DIR")

# 0. Настройка графической подсистемы WSLg для вывода 3D-окна Gazebo и камеры на экран Windows
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
export PULSE_SERVER="${PULSE_SERVER:-/mnt/wslg/PulseServer}"
export LIBGL_ALWAYS_INDIRECT=0

# Пути к 3D-моделям
export GAZEBO_MODEL_PATH="$REPO_DIR/src/mchs_simulation/models:/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH"

# Подключение среды ROS 2
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "[INFO] Подключение ROS 2 Humble..."
        source /opt/ros/humble/setup.bash
    fi
fi

# 1. Запуск симуляции Gazebo 11 (Сервер физики + 3D-окно gzclient)
echo "[1/3] Запуск 3D-симулятора Gazebo 11 (gzserver + 3D GUI gzclient)..."
if command -v ros2 &> /dev/null; then
    ros2 launch "$REPO_DIR/src/mchs_simulation/launch/simulation.launch.py" &
    SIM_PID=$!
else
    python3 "$REPO_DIR/src/mchs_simulation/launch/simulation.launch.py" &
    SIM_PID=$!
fi

# Даем время инициализироваться 3D-окну Gazebo и камере
echo "[INFO] Ожидание инициализации 3D-мира Gazebo (6 сек)..."
sleep 6

# 2. Запуск ноды компьютерного зрения (Окно видеопотока с камеры БПЛА)
echo "[2/3] Запуск ноды компьютерного зрения (Окно тактической камеры БПЛА)..."
python3 "$REPO_DIR/src/mchs_vision/scripts/mchs_vision_node.py" &
VISION_PID=$!

sleep 1

# 3. Запуск полетного автопилота БПЛА
echo "[3/3] Старт программы патрулирования БПЛА..."
python3 "$REPO_DIR/src/mchs_control/scripts/mchs_pilot_node.py" &
PILOT_PID=$!

echo "=========================================================="
echo ">>> ВСЕ СИСТЕМЫ АКТИВНЫ!"
echo ">>> Открыты 2 графических окна:"
echo "    1) Gazebo 11 — 3D-вид полигона и полет дрона"
echo "    2) OpenCV — живая картинка с камеры БПЛА и ИИ-детекция"
echo ">>> Нажмите Ctrl+C для завершения работы."
echo "=========================================================="

trap "kill -9 $SIM_PID $VISION_PID $PILOT_PID 2>/dev/null || true; exit 0" SIGINT SIGTERM
wait
