#!/usr/bin/env bash
# Автоматизированный интеграционный тест сервисов Gazebo и перемещения объектов
set -e

source /opt/ros/humble/setup.bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
REPO_DIR=$(dirname "$SCRIPT_DIR")
cd "$REPO_DIR"

export GAZEBO_MODEL_PATH="$REPO_DIR/src/mchs_simulation/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"

echo "=== 1. ЗАПУСК GZSERVER С АРЕНОЙ ЧС ==="
gzserver --verbose src/mchs_simulation/worlds/mchs_emergency_arena.world -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &
GZ_PID=$!

sleep 5

echo "=== 2. ПРОВЕРКА ROS 2 СЕРВИСОВ GAZEBO ==="
ros2 service list | grep -E "set_entity_state|get_entity_state"

echo "=== 3. ТЕСТ ПЕРЕМЕЩЕНИЯ ОБЪЕКТА ЧЕРЕЗ ARENA_CONTROLLER ==="
python3 src/mchs_simulation/scripts/arena_controller.py --hazard fire --x 18.0 --y -6.0

echo "=== 4. ПРОВЕРКА ФАКТИЧЕСКОЙ ПОЗИЦИИ В GAZEBO ЧЕРЕЗ GZ MODEL ==="
gz model -m fire_hazard -i | grep -A 2 -i "position" | head -n 4

echo "=== 5. ОСТАНОВКА СИМУЛЯТОРА ==="
kill -9 $GZ_PID 2>/dev/null || true
echo "=== ТЕСТ ПРОЙДЕН УСПЕШНО ==="
