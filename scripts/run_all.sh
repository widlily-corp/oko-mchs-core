#!/usr/bin/env bash
# =============================================================================
#  «ОКО МЧС» — единый запуск комплекса (большая арена 200×200 м).
#
#  WSL: gzserver (арена + камера + ROS 2 плагины)
#       → нода зрения: аннотированный кадр + HTTP-стрим :8080
#       → маршрутный автопилот: облёт 4 очагов ЧС / возврат на базу
#  Windows: АРМ оператора (src/mchs_ui/app.py) забирает поток с :8080
#
#  Остановка: Ctrl+C.  Логи: /tmp/mchs_demo/
#  Флаги: MCHS_NO_UI=1 — не открывать окно АРМ автоматически.
# =============================================================================
set -e

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
REPO_DIR=$(dirname "$SCRIPT_DIR")
WORLD="$REPO_DIR/src/mchs_simulation/worlds/mchs_emergency_arena.world"
LOG_DIR="${MCHS_LOG_DIR:-/tmp/mchs_demo}"
mkdir -p "$LOG_DIR"

echo "=========================================================="
echo "   ЗАПУСК КОМПЛЕКСА «ОКО МЧС» — АРЕНА 200×200 М            "
echo "=========================================================="

if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
if ! command -v ros2 >/dev/null 2>&1; then
    echo "[ОШИБКА] ROS 2 Humble не найден (source /opt/ros/humble/setup.bash)"
    exit 1
fi

export GAZEBO_MODEL_PATH="$REPO_DIR/src/mchs_simulation/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe

PIDS=()
cleanup() {
    echo ""
    echo "[INFO] Остановка комплекса «Око МЧС»..."
    for pid in "${PIDS[@]:-}"; do
        kill -9 "$pid" 2>/dev/null || true
    done
    pkill -9 -f "[m]chs_vision_node.py" 2>/dev/null || true
    pkill -9 -f "[m]chs_pilot_route.py" 2>/dev/null || true
    pkill -9 -x gzserver 2>/dev/null || true
    echo "[INFO] Готово."
}
trap cleanup EXIT INT TERM

echo "[1/3] Запуск Gazebo 11: большая арена, дрон, камера, ROS 2 плагины..."
gzserver --verbose "$WORLD" \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so \
    -s libgazebo_ros_state.so \
    > "$LOG_DIR/gzserver.log" 2>&1 &
PIDS+=($!)

CAMERA_READY=0
for _ in $(seq 1 45); do
    if ros2 topic list 2>/dev/null | grep -q "/mchs/camera/image_raw"; then
        CAMERA_READY=1
        break
    fi
    sleep 1
done
if [ "$CAMERA_READY" -eq 1 ]; then
    echo "       Камера БПЛА активна: /mchs/camera/image_raw"
else
    echo "       [ВНИМАНИЕ] Камера не поднялась за 45 c — см. $LOG_DIR/gzserver.log"
fi

echo "[2/3] Запуск бортовой ноды зрения (детекция + веб-стрим :8080)..."
python3 "$REPO_DIR/src/mchs_vision/scripts/mchs_vision_node.py" \
    --ros-args \
    -p enable_stream:=true \
    -p show_window:=false \
    -p stream_port:=8080 \
    > "$LOG_DIR/vision.log" 2>&1 &
PIDS+=($!)

echo "[3/3] Запуск автопилота: БАЗА → ПОЖАР → НЕФТЕБАЗА → ЗАТОПЛЕНИЕ → СВАЛКА..."
python3 "$REPO_DIR/src/mchs_control/scripts/mchs_pilot_route.py" \
    > "$LOG_DIR/pilot.log" 2>&1 &
PIDS+=($!)

sleep 2

if [ "${MCHS_NO_UI:-0}" != "1" ] && command -v powershell.exe >/dev/null 2>&1; then
    UI_UNC='\\wsl.localhost\Ubuntu-22.04\root\oko-mchs-core\src\mchs_ui\app.py'
    echo "[4/4] Открываю АРМ оператора на Windows..."
    powershell.exe -NoProfile -Command \
        "Start-Process -FilePath python -ArgumentList '$UI_UNC'" \
        >/dev/null 2>&1 || echo "       Не удалось автозапустить АРМ (python/UNC)."
fi

cat <<EOF
==========================================================
>>> ДЕМОНСТРАЦИЯ АКТИВНА — АРЕНА 200×200 М
    АРМ оператора (Windows):  живая камера БПЛА + детекция ЧС
    Веб-стрим:                http://localhost:8080
    Маршрут дрона:            база → пожар → нефтебаза → затопление → свалка
    Переместить очаг:         python3 src/mchs_simulation/scripts/arena_controller.py --hazard fire --x 10 --y 5

    Логи: $LOG_DIR/
    Остановка: Ctrl+C в этом окне.
==========================================================
EOF

wait
