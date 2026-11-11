#!/usr/bin/env bash
# Единый запуск полного демонстрационного комплекса «Око МЧС»
set -e

echo "=========================================================="
echo "   ЗАПУСК АВТОНОМНОГО КОМПЛЕКСА МОНИТОРИНГА «ОКО МЧС»     "
echo "=========================================================="

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
REPO_DIR=$(dirname "$SCRIPT_DIR")

# Подключение среды ROS 2, если она не загружена
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "[INFO] Подключение ROS 2 Humble..."
        source /opt/ros/humble/setup.bash
    fi
fi

# 1. Запуск симуляции Gazebo 11
echo "[1/3] Запуск 3D-симулятора Gazebo 11 с ареной ЧС..."
if command -v ros2 &> /dev/null; then
    ros2 launch "$REPO_DIR/src/mchs_simulation/launch/simulation.launch.py" &
    SIM_PID=$!
else
    python3 "$REPO_DIR/src/mchs_simulation/launch/simulation.launch.py" &
    SIM_PID=$!
fi

# Ожидание старта gzserver
echo "[INFO] Ожидание инициализации физического мира Gazebo (5 сек)..."
sleep 5

# 2. Запуск ноды компьютерного зрения
echo "[2/3] Запуск бортовой ноды компьютерного зрения..."
python3 "$REPO_DIR/src/mchs_vision/scripts/mchs_vision_node.py" &
VISION_PID=$!

sleep 1

# 3. Запуск полетного автопилота БПЛА
echo "[3/3] Старт программы патрулирования БПЛА..."
python3 "$REPO_DIR/src/mchs_control/scripts/mchs_pilot_node.py" &
PILOT_PID=$!

echo "=========================================================="
echo ">>> ВСЕ СИСТЕМЫ АКТИВНЫ!"
echo ">>> Нажмите Ctrl+C для штатной остановки всех процессов."
echo "=========================================================="

trap "kill -9 $SIM_PID $VISION_PID $PILOT_PID 2>/dev/null || true; exit 0" SIGINT SIGTERM
wait
