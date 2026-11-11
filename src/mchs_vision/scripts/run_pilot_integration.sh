#!/usr/bin/env bash
# Сквозная проверка «зрение → автопилот → протокол ЦУКС» без Gazebo.
#
# Поднимает ноду зрения и FSM-автопилот, публикует синтетический пожар,
# ждёт HOVER_INSPECTION и файл incident_protocol.json.
set -euo pipefail

REPO_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." &> /dev/null && pwd)
VISION_DIR="$REPO_DIR/src/mchs_vision"
CONTROL_DIR="$REPO_DIR/src/mchs_control"
TMP_DIR="${TMPDIR:-/tmp}/oko_mchs_pilot_check"
REPORT_DIR="$TMP_DIR/reports"
VISION_LOG="$TMP_DIR/vision.log"
PILOT_LOG="$TMP_DIR/pilot.log"

echo "=================================================================="
echo "  СКВОЗНАЯ ИНТЕГРАЦИЯ: ЗРЕНИЕ → АВТОПИЛОТ → ПРОТОКОЛ"
echo "  Временные файлы: $TMP_DIR"
echo "=================================================================="

if [[ -f /opt/ros/humble/setup.bash ]]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
else
    echo "[ERROR] не найден /opt/ros/humble/setup.bash" >&2
    exit 1
fi

export PYTHONPATH="$VISION_DIR:$CONTROL_DIR:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"

rm -rf "$TMP_DIR"
mkdir -p "$REPORT_DIR"

python3 "$VISION_DIR/scripts/mchs_vision_node.py" \
    --ros-args \
    -p enable_stream:=false \
    -p show_window:=false \
    -p camera_pitch_down_deg:=90.0 \
    -p image_width:=640 \
    -p image_height:=480 \
    -p track_min_hits:=1 \
    -p event_min_interval_s:=0.5 \
    >"$VISION_LOG" 2>&1 &
VISION_PID=$!

python3 "$CONTROL_DIR/scripts/mchs_pilot_node.py" \
    --report-dir "$REPORT_DIR" \
    --takeoff-altitude 8.0 \
    --control-rate 10.0 \
    >"$PILOT_LOG" 2>&1 &
PILOT_PID=$!

cleanup() {
    kill "$VISION_PID" "$PILOT_PID" 2>/dev/null || true
    wait "$VISION_PID" "$PILOT_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 2
if ! kill -0 "$VISION_PID" 2>/dev/null; then
    echo "[ERROR] нода зрения завершилась при старте:" >&2
    cat "$VISION_LOG" >&2
    exit 1
fi
if ! kill -0 "$PILOT_PID" 2>/dev/null; then
    echo "[ERROR] автопилот завершился при старте:" >&2
    cat "$PILOT_LOG" >&2
    exit 1
fi

if ! python3 "$VISION_DIR/scripts/ros_pilot_integration_check.py" \
    --timeout 45 --report-dir "$REPORT_DIR"; then
    echo "[ERROR] сквозная интеграция не прошла. Логи:" >&2
    echo "--- vision ---" >&2
    tail -40 "$VISION_LOG" >&2 || true
    echo "--- pilot ---" >&2
    tail -60 "$PILOT_LOG" >&2 || true
    exit 1
fi

echo "[OK] Протоколы: $REPORT_DIR"
