#!/usr/bin/env bash
# Единый запуск полного демонстрационного комплекса «Око МЧС»
set -e

echo "=========================================================="
echo "   ЗАПУСК АВТОНОМНОГО КОМПЛЕКСА МОНИТОРИНГА «ОКО МЧС»     "
echo "=========================================================="

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
REPO_DIR=$(dirname "$SCRIPT_DIR")

# 1. Запуск мира Gazebo с очагом огня
echo "[1/3] Запуск 3D-симулятора Gazebo 11..."
python3 "$REPO_DIR/src/mchs_simulation/launch/simulation.launch.py" &
SIM_PID=$!

sleep 3

# 2. Запуск ноды компьютерного зрения
echo "[2/3] Запуск бортовой ноды компьютерного зрения..."
python3 "$REPO_DIR/src/mchs_vision/scripts/mchs_vision_node.py" &
VISION_PID=$!

sleep 1

# 3. Запуск полетного контроллера БПЛА
echo "[3/3] Старт программы патрулирования БПЛА..."
python3 "$REPO_DIR/src/mchs_control/scripts/mchs_pilot_node.py" &
PILOT_PID=$!

echo ">>> ВСЕ СИСТЕМЫ АКТИВНЫ! Нажмите Ctrl+C для остановки всех процессов."

trap "kill -9 $SIM_PID $VISION_PID $PILOT_PID 2>/dev/null || true; exit 0" SIGINT SIGTERM
wait
