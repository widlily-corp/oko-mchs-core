#!/usr/bin/env python3
"""
Контроллер динамического перемещения объектов полигона ЧС 'Око МЧС'.
Позволяет в реальном времени (на лету, без перезапуска Gazebo) перемещать
любой из 4 очагов ЧС по координатам для проверки работы БПЛА и компьютерного зрения.
"""
import sys
import os
import argparse
import json
import math
import random
import subprocess

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
SRC_DIR = os.path.join(REPO_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

try:
    from mchs_core import get_logger, install_crash_handler, log_event
except ImportError:
    get_logger = None  # type: ignore[assignment]
    install_crash_handler = None  # type: ignore[assignment]
    log_event = None  # type: ignore[assignment]

if install_crash_handler is not None:
    install_crash_handler("SIM")

logger = get_logger("arena_controller", subsystem="SIM") if get_logger is not None else None

HAZARD_MODELS = {
    'fire': {
        'name': 'fire_hazard',
        'default_z': 0.2,
        'label': 'Очаг лесного пожара'
    },
    'oil': {
        'name': 'oil_spill_hazard',
        'default_z': 0.02,
        'label': 'Разлив нефтепродуктов'
    },
    'flood': {
        'name': 'flood_hazard',
        'default_z': 0.02,
        'label': 'Зона затопления / паводка'
    },
    'waste': {
        'name': 'waste_hazard',
        'default_z': 0.4,
        'label': 'Токсичные отходы / бочки'
    },
    'human': {
        'name': 'person_standing',
        'default_z': 0.0,
        'label': 'Пострадавший (человек в сигнальном жилете)'
    },
    'victim': {
        'name': 'person_standing',
        'default_z': 0.0,
        'label': 'Пострадавший (человек в сигнальном жилете)'
    }
}

#: Соответствие ключа --hazard и записи в config/hazards.json.
CONFIG_KEYS = {
    'fire': 'FIRE',
    'oil': 'OIL',
    'flood': 'FLOOD',
    'waste': 'WASTE',
    'human': 'HUMAN',
    'victim': 'HUMAN',
}
HAZARDS_CONFIG_PATH = os.path.join(REPO_ROOT, "config", "hazards.json")


def update_hazards_config(hazard_key: str, x: float, y: float,
                          z: float) -> bool:
    """Записывает координаты перемещённого очага в config/hazards.json.

    Файл — единый источник истины для АРМ и ноды зрения. Без этой записи
    перемещённый человек оставался «не там» для фото-улик и протокола ЦУКС.
    """
    config_key = CONFIG_KEYS.get(hazard_key)
    if config_key is None:
        return False
    try:
        with open(HAZARDS_CONFIG_PATH, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, ValueError) as exc:
        if logger:
            logger.warning(f"config/hazards.json не прочитан: {exc}")
        return False
    entry = data.get(config_key)
    if not isinstance(entry, dict):
        return False
    entry["x"] = round(float(x), 2)
    entry["y"] = round(float(y), 2)
    entry["z"] = round(float(z), 2)
    tmp_path = HAZARDS_CONFIG_PATH + ".tmp"
    try:
        with open(tmp_path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
        os.replace(tmp_path, HAZARDS_CONFIG_PATH)
    except OSError as exc:
        if logger:
            logger.warning(f"config/hazards.json не обновлён: {exc}")
        return False
    if logger:
        logger.info(
            f"config/hazards.json обновлён: {config_key} -> "
            f"({x:.2f}, {y:.2f}, {z:.2f})")
    return True


# Рандомизация по всей арене 200×200 м (для стресс-теста жюри).
RANDOM_ARENA_LIMIT_M = 85.0        # x, y ∈ [-85, 85] м
RANDOM_MIN_HAZARD_SPACING_M = 30.0  # минимум 30 м между очагами
RANDOM_MIN_BASE_DISTANCE_M = 20.0   # не ближе 20 м к базе (0, 0)


def is_gazebo_running() -> bool:
    """Проверяет, запущен ли реальный процесс gzserver / gazebo."""
    try:
        # Быстрая проверка через pgrep
        res = subprocess.run(["pgrep", "-x", "gzserver"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=1)
        if res.returncode == 0 and res.stdout.strip():
            return True
        # Альтернативная проверка наличия топиков Gazebo
        res2 = subprocess.run(["gz", "topic", "-l"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=1.5)
        return res2.returncode == 0 and bool(res2.stdout.strip())
    except Exception:
        return False

def move_hazard_ros2(model_name: str, x: float, y: float, z: float) -> bool:
    """Перемещение через ROS 2 сервисы /gazebo/set_entity_state или /gazebo/set_model_state с проверкой результата."""
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        if logger:
            logger.warning(f"ROS 2 (rclpy) недоступен для перемещения {model_name}: {exc}")
        return False

    try:
        rclpy.init(args=None)
        node = Node('mchs_arena_controller')
        
        # 1. Сначала пробуем стандартный сервис SetEntityState
        from gazebo_msgs.srv import SetEntityState
        client = node.create_client(SetEntityState, '/gazebo/set_entity_state')

        if client.wait_for_service(timeout_sec=1.5):
            req = SetEntityState.Request()
            req.state.name = model_name
            req.state.pose.position.x = float(x)
            req.state.pose.position.y = float(y)
            req.state.pose.position.z = float(z)
            req.state.reference_frame = 'world'

            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            res = future.result()
            node.destroy_node()
            rclpy.shutdown()
            if res is not None and getattr(res, 'success', False):
                if logger:
                    logger.info(f"ROS 2 сервис /gazebo/set_entity_state успешно переместил {model_name} -> ({x:.2f}, {y:.2f}, {z:.2f})")
                return True
            if logger:
                logger.warning(f"ROS 2 сервис /gazebo/set_entity_state вернул статус 'неуспешно' для {model_name}")
            return False

        # 2. Если SetEntityState нет, пробуем SetModelState
        from gazebo_msgs.srv import SetModelState
        client_legacy = node.create_client(SetModelState, '/gazebo/set_model_state')
        if client_legacy.wait_for_service(timeout_sec=1.0):
            req = SetModelState.Request()
            req.model_state.model_name = model_name
            req.model_state.pose.position.x = float(x)
            req.model_state.pose.position.y = float(y)
            req.model_state.pose.position.z = float(z)
            req.model_state.reference_frame = 'world'

            future = client_legacy.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            res = future.result()
            node.destroy_node()
            rclpy.shutdown()
            if res is not None and getattr(res, 'success', False):
                if logger:
                    logger.info(f"ROS 2 legacy-сервис /gazebo/set_model_state успешно переместил {model_name} -> ({x:.2f}, {y:.2f}, {z:.2f})")
                return True
            if logger:
                logger.warning(f"ROS 2 legacy-сервис /gazebo/set_model_state вернул статус 'неуспешно' для {model_name}")
            return False

        if logger:
            logger.warning(f"Сервисы /gazebo/set_entity_state и /gazebo/set_model_state не ответили за таймаут для {model_name}")
        node.destroy_node()
        rclpy.shutdown()
        return False
    except Exception as exc:
        if logger:
            logger.error(f"Исключение при вызове ROS 2 сервиса Gazebo для {model_name}: {exc}")
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return False

def move_hazard_gz_cli(model_name: str, x: float, y: float, z: float) -> bool:
    """Фоллбэк перемещения через утилиту gz model с валидацией активности симулятора."""
    if not is_gazebo_running():
        if logger:
            logger.error(f"gz model CLI отменён: Gazebo 11 не запущен (для {model_name})")
        return False

    try:
        cmd = ["gz", "model", "-m", model_name, "-x", str(x), "-y", str(y), "-z", str(z)]
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3)
        if res.returncode != 0:
            if logger:
                logger.warning(f"gz model CLI вернул код ошибки {res.returncode}: {res.stderr.strip()}")
            return False
            
        # Проверяем, что модель реально присутствует в мире
        info_cmd = ["gz", "model", "-m", model_name, "-i"]
        info_res = subprocess.run(info_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2)
        success = (info_res.returncode == 0)
        if success and logger:
            logger.info(f"gz model CLI успешно переместил {model_name} -> ({x:.2f}, {y:.2f}, {z:.2f})")
        elif not success and logger:
            logger.warning(f"Модель {model_name} отсутствует в мире Gazebo по данным gz model -i")
        return success
    except Exception as exc:
        if logger:
            logger.error(f"Исключение при выполнении gz model CLI для {model_name}: {exc}")
        return False

def move_hazard(hazard_key: str, x: float, y: float, z: float = None) -> bool:
    cfg = HAZARD_MODELS[hazard_key]
    model_name = cfg['name']
    target_z = z if z is not None else cfg['default_z']

    move_msg = f"Перемещение '{cfg['label']}' ({model_name}) -> X={x:.2f}, Y={y:.2f}, Z={target_z:.2f}..."
    print(f"[ARENA CONTROLLER] {move_msg}")
    if logger:
        logger.info(move_msg)

    # Проверяем, запущена ли симуляция
    if not is_gazebo_running():
        err_msg = "Симулятор Gazebo 11 не запущен (процесс gzserver не найден)."
        print(f"[ARENA CONTROLLER] ❌ Ошибка: {err_msg}")
        print(f"                   Сначала запустите: ros2 launch src/mchs_simulation/launch/simulation.launch.py")
        if logger:
            logger.error(f"Ошибка связи с Gazebo: {err_msg}")
        if log_event is not None:
            log_event("SIM", "GAZEBO_CONNECTION_ERROR", {
                "error": err_msg,
                "target_model": model_name,
            }, level="ERROR")
        return False

    # 1. Попытка через ROS 2 сервис
    success = False
    try:
        success = move_hazard_ros2(model_name, x, y, target_z)
    except Exception as exc:
        if logger:
            logger.error(f"Ошибка перемещения через ROS 2 сервис: {exc}")
        success = False

    # 2. Фоллбэк через gz model CLI
    if not success:
        if logger:
            logger.info(f"Переход на CLI фоллбэк gz model для {model_name}")
        success = move_hazard_gz_cli(model_name, x, y, target_z)

    if success:
        update_hazards_config(hazard_key, x, y, target_z)
        succ_msg = f"Успешно: {model_name} перемещен в координаты ({x:.2f}, {y:.2f})"
        print(f"[ARENA CONTROLLER] ✅ {succ_msg}")
        if logger:
            logger.info(succ_msg)
        if log_event is not None:
            log_event("SIM", "HAZARD_RELOCATED", {
                "hazard_key": hazard_key,
                "model_name": model_name,
                "label": cfg['label'],
                "x": float(x),
                "y": float(y),
                "z": float(target_z),
            })
    else:
        fail_msg = f"Не удалось переместить {model_name}. Убедитесь, что модель присутствует в мире."
        print(f"[ARENA CONTROLLER] ⚠️ {fail_msg}")
        if logger:
            logger.warning(fail_msg)
        if log_event is not None:
            log_event("SIM", "HAZARD_RELOCATION_FAILED", {
                "hazard_key": hazard_key,
                "model_name": model_name,
                "x": float(x),
                "y": float(y),
            }, level="WARNING")
    return success

def randomize_arena():
    """Случайное распределение 4 очагов ЧС по всей арене 200×200 м (стресс-тест жюри)."""
    if not is_gazebo_running():
        err_msg = "Gazebo не запущен. Невозможно перетасовать объекты."
        print(f"[ARENA CONTROLLER] ❌ Ошибка: {err_msg}")
        if logger:
            logger.error(f"Ошибка связи с Gazebo: {err_msg}")
        if log_event is not None:
            log_event("SIM", "GAZEBO_CONNECTION_ERROR", {"error": err_msg}, level="ERROR")
        return

    rand_msg = "🎲 Случайная перетасовка объектов арены ЧС..."
    print(f"[ARENA CONTROLLER] {rand_msg}")
    if logger:
        logger.info(rand_msg)
    if log_event is not None:
        log_event("SIM", "ARENA_RANDOMIZE_STARTED", {})
    used_positions = []

    for key, cfg in HAZARD_MODELS.items():
        placed = False
        # До 500 попыток: арена большая, ограничения по расстояниям мягкие.
        for _ in range(500):
            rx = round(random.uniform(-RANDOM_ARENA_LIMIT_M, RANDOM_ARENA_LIMIT_M), 1)
            ry = round(random.uniform(-RANDOM_ARENA_LIMIT_M, RANDOM_ARENA_LIMIT_M), 1)

            if math.hypot(rx, ry) < RANDOM_MIN_BASE_DISTANCE_M:
                continue  # не ближе 20 м к базе (0, 0)
            if any(
                math.hypot(rx - ux, ry - uy) < RANDOM_MIN_HAZARD_SPACING_M
                for ux, uy in used_positions
            ):
                continue  # не ближе 30 м между очагами
            used_positions.append((rx, ry))
            move_hazard(key, rx, ry)
            placed = True
            break
        if not placed:
            msg = f"Не удалось подобрать позицию для очага {cfg['name']} за 500 попыток"
            print(f"[ARENA CONTROLLER] ⚠️ {msg}")
            if logger:
                logger.warning(msg)

def print_hazard_list():
    print("\n=== СПИСОК ДОСТУПНЫХ ОБЪЕКТОВ ЧС НА АРЕНЕ ===")
    for key, cfg in HAZARD_MODELS.items():
        print(f"  • --hazard {key:<6} -> {cfg['label']} (Model: {cfg['name']})")
    print("=============================================\n")

def apply_positions(raw: str) -> int:
    """Перемещает несколько очагов за один запуск (одна инициализация rclpy).

    Используется кнопкой «🎲 СЛУЧАЙНО» в АРМ: все четыре очага переезжают
    одной командой и без лишних задержек на старт ROS 2.

    Формат: ``{"fire": [x, y], "oil": [x, y], ...}`` (лишние ключи
    игнорируются, z берётся по умолчанию для типа очага).
    """
    try:
        payload = json.loads(raw)
    except ValueError as exc:
        print(f"[ARENA CONTROLLER] ❌ --positions: некорректный JSON: {exc}")
        return 2
    if not isinstance(payload, dict):
        print("[ARENA CONTROLLER] ❌ --positions: ожидается объект {ключ: [x, y]}")
        return 2

    ok = True
    for key, cfg in HAZARD_MODELS.items():
        if key not in payload:
            continue
        value = payload[key]
        try:
            if isinstance(value, dict):
                x = float(value["x"])
                y = float(value["y"])
                z = float(value.get("z", cfg["default_z"]))
            else:
                x = float(value[0])
                y = float(value[1])
                z = float(value[2]) if len(value) > 2 else float(cfg["default_z"])
        except (KeyError, IndexError, TypeError, ValueError):
            print(f"[ARENA CONTROLLER] ❌ {key}: некорректные координаты {value!r}")
            ok = False
            continue
        ok = move_hazard(key, x, y, z) and ok
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description="MCHS Dynamic Arena Hazard Controller")
    parser.add_argument('--hazard', choices=['fire', 'oil', 'flood', 'waste', 'human', 'victim'], help="Тип ЧС для перемещения")
    parser.add_argument('--x', type=float, help="Координата X (вперед от базы)")
    parser.add_argument('--y', type=float, help="Координата Y (влево/вправо)")
    parser.add_argument('--z', type=float, default=None, help="Высота Z")
    parser.add_argument('--random', action='store_true', help="Случайно перемешать все объекты")
    parser.add_argument('--list', action='store_true', help="Показать список объектов")
    parser.add_argument('--positions',
                        help="JSON с координатами всех очагов: {\"fire\": [x, y], ...}")

    args = parser.parse_args()

    if args.positions:
        sys.exit(apply_positions(args.positions))

    if args.list:
        print_hazard_list()
        return

    if args.random:
        randomize_arena()
        return

    if args.hazard:
        if args.x is None or args.y is None:
            print("Ошибка: Для перемещения укажите координаты --x <X> --y <Y>")
            return
        move_hazard(args.hazard, args.x, args.y, args.z)
    else:
        parser.print_help()

if __name__ == '__main__':
    main()
