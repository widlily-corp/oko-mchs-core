#!/usr/bin/env python3
"""
Контроллер динамического перемещения объектов полигона ЧС 'Око МЧС'.
Позволяет в реальном времени (на лету, без перезапуска Gazebo) перемещать
любой из 4 очагов ЧС по координатам для проверки работы БПЛА и компьютерного зрения.
"""
import sys
import argparse
import random
import subprocess
import json

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
    }
}

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
    except ImportError:
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
                return True
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
                return True
            return False

        node.destroy_node()
        rclpy.shutdown()
        return False
    except Exception:
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return False

def move_hazard_gz_cli(model_name: str, x: float, y: float, z: float) -> bool:
    """Фоллбэк перемещения через утилиту gz model с валидацией активности симулятора."""
    if not is_gazebo_running():
        return False

    try:
        cmd = ["gz", "model", "-m", model_name, "-x", str(x), "-y", str(y), "-z", str(z)]
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3)
        if res.returncode != 0:
            return False
            
        # Проверяем, что модель реально присутствует в мире
        info_cmd = ["gz", "model", "-m", model_name, "-i"]
        info_res = subprocess.run(info_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2)
        return info_res.returncode == 0
    except Exception:
        return False

def move_hazard(hazard_key: str, x: float, y: float, z: float = None) -> bool:
    cfg = HAZARD_MODELS[hazard_key]
    model_name = cfg['name']
    target_z = z if z is not None else cfg['default_z']

    print(f"[ARENA CONTROLLER] Перемещение '{cfg['label']}' ({model_name}) -> X={x:.2f}, Y={y:.2f}, Z={target_z:.2f}...")

    # Проверяем, запущена ли симуляция
    if not is_gazebo_running():
        print(f"[ARENA CONTROLLER] ❌ Ошибка: Симулятор Gazebo 11 не запущен (процесс gzserver не найден).")
        print(f"                   Сначала запустите: ros2 launch src/mchs_simulation/launch/simulation.launch.py")
        return False

    # 1. Попытка через ROS 2 сервис
    success = False
    try:
        success = move_hazard_ros2(model_name, x, y, target_z)
    except Exception:
        success = False

    # 2. Фоллбэк через gz model CLI
    if not success:
        success = move_hazard_gz_cli(model_name, x, y, target_z)

    if success:
        print(f"[ARENA CONTROLLER] ✅ Успешно: {model_name} перемещен в координаты ({x:.2f}, {y:.2f})")
    else:
        print(f"[ARENA CONTROLLER] ⚠️ Не удалось переместить {model_name}. Убедитесь, что модель присутствует в мире.")
    return success

def randomize_arena():
    """Случайное распределение 4 очагов ЧС в зоне патрулирования (для стресс-теста жюри)."""
    if not is_gazebo_running():
        print("[ARENA CONTROLLER] ❌ Ошибка: Gazebo не запущен. Невозможно перетасовать объекты.")
        return

    print("[ARENA CONTROLLER] 🎲 Случайная перетасовка объектов арены ЧС...")
    used_positions = []

    for key, cfg in HAZARD_MODELS.items():
        while True:
            # Сектор поиска: X от +6 до +20 м (впереди от базы), Y от -14 до +14 м
            rx = round(random.uniform(7.0, 19.0), 1)
            ry = round(random.uniform(-13.0, 13.0), 1)

            too_close = False
            for ux, uy in used_positions:
                dist = ((rx - ux)**2 + (ry - uy)**2)**0.5
                if dist < 5.0:
                    too_close = True
                    break
            if not too_close:
                used_positions.append((rx, ry))
                move_hazard(key, rx, ry)
                break

def print_hazard_list():
    print("\n=== СПИСОК ДОСТУПНЫХ ОБЪЕКТОВ ЧС НА АРЕНЕ ===")
    for key, cfg in HAZARD_MODELS.items():
        print(f"  • --hazard {key:<6} -> {cfg['label']} (Model: {cfg['name']})")
    print("=============================================\n")

def main():
    parser = argparse.ArgumentParser(description="MCHS Dynamic Arena Hazard Controller")
    parser.add_argument('--hazard', choices=['fire', 'oil', 'flood', 'waste'], help="Тип ЧС для перемещения")
    parser.add_argument('--x', type=float, help="Координата X (вперед от базы)")
    parser.add_argument('--y', type=float, help="Координата Y (влево/вправо)")
    parser.add_argument('--z', type=float, default=None, help="Высота Z")
    parser.add_argument('--random', action='store_true', help="Случайно перемешать все объекты")
    parser.add_argument('--list', action='store_true', help="Показать список объектов")

    args = parser.parse_args()

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
