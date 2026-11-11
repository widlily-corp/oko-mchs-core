#!/usr/bin/env bash
# Полная проверка модуля зрения «Око МЧС»: юнит-тесты, офлайн-демо и ROS 2.
#
# Запуск:
#   bash src/mchs_vision/scripts/run_vision_tests.sh
#
# Требуется ROS 2 Humble (/opt/ros/humble). Все временные файлы — в /tmp.
set -euo pipefail

REPO_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." &> /dev/null && pwd)
VISION_DIR="$REPO_DIR/src/mchs_vision"
TMP_DIR="${TMPDIR:-/tmp}/oko_mchs_vision_check"
NODE_LOG="$TMP_DIR/vision_node.log"

echo "=================================================================="
echo "  ОКО МЧС: ПРОВЕРКА МОДУЛЯ КОМПЬЮТЕРНОГО ЗРЕНИЯ"
echo "  Репозиторий: $REPO_DIR"
echo "  Временные файлы: $TMP_DIR"
echo "=================================================================="

if [[ -f /opt/ros/humble/setup.bash ]]; then
    # setup.bash ROS 2 обращается к неинициализированным переменным,
    # поэтому на время source отключаем set -u.
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
else
    echo "[ERROR] не найден /opt/ros/humble/setup.bash" >&2
    exit 1
fi

export PYTHONPATH="$VISION_DIR:${PYTHONPATH:-}"
# Отдельный домен, чтобы тест не смешивался с запущенной демонстрацией.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-44}"

rm -rf "$TMP_DIR"
mkdir -p "$TMP_DIR/dataset" "$TMP_DIR/offline"

echo
echo "[1/4] Юнит-тесты (pytest)..."
python3 -m pytest "$VISION_DIR/tests" -q

echo
echo "[2/4] Генерация синтетического датасета..."
python3 "$VISION_DIR/scripts/make_test_scenes.py" --output "$TMP_DIR/dataset"

echo
echo "[3/4] Офлайн-демонстрация и метрики..."
python3 "$VISION_DIR/scripts/vision_demo.py" \
    --input "$TMP_DIR/dataset" --output "$TMP_DIR/offline" --json --csv

echo
echo "[4/4] ROS 2-интеграция (реальные топики)..."
python3 "$VISION_DIR/scripts/mchs_vision_node.py" \
    --ros-args \
    -p show_window:=false \
    -p enable_stream:=false \
    -p camera_pitch_down_deg:=90.0 \
    -p track_min_hits:=2 \
    -p event_cooldown_s:=1.0 \
    -p event_min_interval_s:=0.5 \
    >"$NODE_LOG" 2>&1 &
NODE_PID=$!

cleanup() {
    kill "$NODE_PID" 2>/dev/null || true
    wait "$NODE_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 3
if ! kill -0 "$NODE_PID" 2>/dev/null; then
    echo "[ERROR] нода зрения завершилась при старте. Лог:" >&2
    cat "$NODE_LOG" >&2
    exit 1
fi

if ! python3 "$VISION_DIR/scripts/ros_integration_check.py" \
    --timeout 20 --save-annotated "$TMP_DIR/ros_annotated.png"; then
    echo "[ERROR] ROS-интеграция не прошла. Лог ноды:" >&2
    cat "$NODE_LOG" >&2
    exit 1
fi

echo
echo "=================================================================="
echo "  ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ"
echo "  Результаты: $TMP_DIR"
echo "=================================================================="
