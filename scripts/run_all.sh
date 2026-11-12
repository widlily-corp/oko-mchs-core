#!/usr/bin/env bash
# =============================================================================
#  «ОКО МЧС» — единый запуск комплекса (большая арена 200×200 м).
#
#  WSL: gzserver (арена + камера + ROS 2 плагины)
#       → нода зрения: аннотированный кадр + HTTP-стрим :8080
#       → автопилот-сканер: змейка по всей арене 200×200 / возврат на базу
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
STDOUT_LOG_DIR="$REPO_DIR/logs/stdout"
mkdir -p "$LOG_DIR" "$STDOUT_LOG_DIR"

# Чистое демо: убираем артефакты прошлых запусков, чтобы АРМ не подхватил
# старый протокол/команду и не показал ложную тревогу при старте.
rm -f "$REPO_DIR/reports/incident_protocol.json" \
      "$REPO_DIR/reports/incident_protocol.json.tmp" \
      "$REPO_DIR/reports/mission.cmd"
mkdir -p "$REPO_DIR/reports/incidents" "$REPO_DIR/reports/photos" "$REPO_DIR/reports/video"

> "$STDOUT_LOG_DIR/gazebo.log"
> "$STDOUT_LOG_DIR/vision.log"
> "$STDOUT_LOG_DIR/pilot.log"

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
export PYTHONPATH="$REPO_DIR/src:$REPO_DIR/src/mchs_vision:$REPO_DIR/src/mchs_core:$REPO_DIR/src/mchs_control:$REPO_DIR/src/mchs_ui:${PYTHONPATH:-}"
# Prefer the WSLg GPU (D3D12/OpenGL) when Mesa reports real hardware
# acceleration; keep an explicit software mode for headless machines.
MCHS_RENDERER=${MCHS_RENDERER:-auto}
case "$MCHS_RENDERER" in
    auto)
        unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER MESA_LOADER_DRIVER_OVERRIDE
        if [ -e /dev/dxg ] && command -v glxinfo >/dev/null 2>&1 \
                && glxinfo -B 2>/dev/null | grep -q 'Accelerated: yes'; then
            echo "[INFO] WSLg GPU renderer: $(glxinfo -B 2>/dev/null | sed -n 's/^[[:space:]]*OpenGL renderer string: //p' | head -n 1)"
        else
            export LIBGL_ALWAYS_SOFTWARE=1
            export GALLIUM_DRIVER=llvmpipe
            export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
            echo "[INFO] Аппаратный OpenGL недоступен; используется программный llvmpipe."
        fi
        ;;
    software)
        export LIBGL_ALWAYS_SOFTWARE=1
        export GALLIUM_DRIVER=llvmpipe
        export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
        echo "[INFO] Принудительно включен программный рендеринг (MCHS_RENDERER=software)."
        ;;
    *)
        echo "[ОШИБКА] Неизвестный MCHS_RENDERER='$MCHS_RENDERER' (auto/software)." >&2
        exit 2
        ;;
esac

PIDS=()
cleanup() {
    echo ""
    echo "[INFO] Остановка комплекса «Око МЧС»..."
    for pid in "${PIDS[@]:-}"; do
        kill -9 "$pid" 2>/dev/null || true
    done
    pkill -9 -f "[m]chs_vision_node.py" 2>/dev/null || true
    pkill -9 -f "[m]chs_pilot_node.py" 2>/dev/null || true
    pkill -9 -f "[m]chs_pilot_route.py" 2>/dev/null || true
    pkill -9 -x gzserver 2>/dev/null || true
    echo "[INFO] Готово."
}
trap cleanup EXIT INT TERM

echo "[0/3] Генерация мира арены: очаги из config/hazards.json..."
GEN_WORLD="$REPO_DIR/src/mchs_simulation/scripts/generate_arena_world.py"
if python3 "$GEN_WORLD"; then
    echo "       Мир обновлён, очаги из config/hazards.json"
else
    echo "       [ВНИМАНИЕ] Мир не перегенерирован — использую существующий $WORLD"
fi

echo "[1/3] Запуск Gazebo 11: большая арена, дрон, камера, ROS 2 плагины..."
gzserver --verbose "$WORLD" \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so \
    > >(tee -a "$STDOUT_LOG_DIR/gazebo.log") 2>&1 &
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
    echo "       [ВНИМАНИЕ] Камера не поднялась за 45 c — см. $STDOUT_LOG_DIR/gazebo.log"
fi

echo "[2/3] Запуск бортовой ноды зрения (детекция + веб-стрим :8080)..."
python3 "$REPO_DIR/src/mchs_vision/scripts/mchs_vision_node.py" \
    --ros-args \
    -p enable_stream:=true \
    -p show_window:=false \
    -p stream_port:=8080 \
    > >(tee -a "$STDOUT_LOG_DIR/vision.log") 2>&1 &
PIDS+=($!)

echo "[3/3] Запуск автопилота-сканера: змейка по всей арене (план — config/mission.json)..."
PILOT_EXEC="${MCHS_PILOT_SCRIPT:-$REPO_DIR/src/mchs_control/scripts/mchs_pilot_route.py}"
if [ ! -f "$PILOT_EXEC" ]; then
    PILOT_EXEC="$REPO_DIR/src/mchs_control/scripts/mchs_pilot_node.py"
fi
python3 "$PILOT_EXEC" \
    > >(tee -a "$STDOUT_LOG_DIR/pilot.log") 2>&1 &
PIDS+=($!)

sleep 2

if [ "${MCHS_NO_UI:-0}" != "1" ] && command -v powershell.exe >/dev/null 2>&1; then
    UI_LOCAL="$REPO_DIR/src/mchs_ui/app.py"
    # Путь для Windows считаем от фактического корня репозитория, а не хардкодом:
    # иначе открывается устаревшая копия АРМ из другого чекаута.
    UI_WIN=$(wslpath -w "$UI_LOCAL" 2>/dev/null || true)
    [ -n "$UI_WIN" ] || UI_WIN="$UI_LOCAL"
    echo "[4/4] Открываю АРМ оператора на Windows: $UI_WIN"
    powershell.exe -NoProfile -Command \
        "Start-Process -FilePath python -ArgumentList '$UI_WIN'" \
        >/dev/null 2>&1 || echo "       Не удалось автозапустить АРМ (python/UNC)."
fi

cat <<EOF
==========================================================
>>> ДЕМОНСТРАЦИЯ АКТИВНА — АРЕНА 200×200 М
    Сканирование:             вся арена змейкой (ряды/шаг — config/mission.json)
    АРМ оператора (Windows):  живая камера БПЛА + детекция ЧС + карта очагов
    Веб-стрим:                http://localhost:8080
    Очаги ЧС:                 перетащите очаг прямо на карте АРМ — координаты
                              сохранятся в config/hazards.json и применятся в Gazebo
    Отчёты:                   reports/incidents (фото очагов с борта),
                              reports/photos (снимки оператора),
                              reports/video (запись видео),
                              reports/incident_protocol.json (протокол ЦУКС)
    Переместить очаг вручную: python3 src/mchs_simulation/scripts/arena_controller.py --hazard fire --x 10 --y 5
    Логи:                     logs/system.log, logs/events.jsonl, logs/gcs_ui.log,
                              logs/stdout/ (gazebo.log, vision.log, pilot.log)
    Остановка: Ctrl+C в этом окне.
==========================================================
EOF

wait
