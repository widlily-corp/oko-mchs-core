#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Автопилот-сканер БПЛА «ОКО МЧС»: змейка (boustrophedon) по всей арене 200×200 м.

Сценарий полёта
---------------
Дрон МЧС стартует с базы (0, 0) и непрерывно сканирует всю арену:

* ряды идут вдоль оси X с шагом ``scan.row_step_m`` по оси Y
  (от ``scan.y_min`` до ``scan.y_max``);
* внутри ряда — проход от ``scan.x_min`` до ``scan.x_max``, следующий ряд
  проходится в обратном направлении (чередование +X / -X);
* после последнего ряда цикл повторяется с первого (маршрут замкнут).

Высота 22 м задаётся моделью мира / кинематическим плагином, автопилот
ось Z не трогает. Управление — ``libgazebo_ros_planar_move`` через
``geometry_msgs/Twist`` (``linear.x``/``linear.y`` — скорости в мировой СК).

Входы/выходы
------------
подписки:  ``/mchs/odom``         (nav_msgs/Odometry)     — фактическая позиция
           ``/mchs/detection_event`` (std_msgs/String)     — тревоги зрения
публикует: ``/mchs/cmd_vel``      (geometry_msgs/Twist)   — уставки скорости
           ``/mchs/scan_status``  (std_msgs/String, JSON) — статус для АРМ.

JSON статуса (раз в 1 с)::

    {"row": k, "rows_total": N, "progress": 0.0..1.0,
     "phase": "ОЖИДАНИЕ ОДОМЕТРИИ" | "ВЫХОД В ЗОНУ" | "СКАНИРОВАНИЕ" |
              "ИНСПЕКЦИЯ ОЧАГА" | "ВОЗВРАТ НА БАЗУ" | "ОЖИДАНИЕ",
     "x": .., "y": .., "target_x": .., "target_y": .., "ts": ..}

``row`` — номер текущего ряда, нумерация с 1 (``1..rows_total``).

Безопасность на старте: до первого сообщения ``/mchs/odom`` дрон не двигается
(нулевая уставка, фаза "ОЖИДАНИЕ ОДОМЕТРИИ"), чтобы кинематический плагин не
улетел вслепую. Если одометрия пропала позже — резервная оценка позиции
интегрируется из опубликованных команд (таймаут 2 с).

Файл-команды оператора: ``reports/mission.cmd`` (путь из ``config/mission.json``,
поле ``mission_cmd_file``): ``rtl`` — возврат на базу (0, 0) с удержанием,
``scan`` — продолжить сканирование. Файл удаляется после чтения.

Конфиги: ``config/mission.json`` (план/скорости/топики) и
``config/hazards.json`` (очаги для лога наблюдений). При отсутствии файлов
используются fallback-константы ниже.

Запуск::

    source /opt/ros/humble/setup.bash
    python3 src/mchs_control/scripts/mchs_pilot_route.py
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
SRC_DIR = os.path.join(REPO_ROOT, "src")
if SRC_DIR not in sys.path:  # импорт mchs_core работает из любой папки
    sys.path.insert(0, SRC_DIR)
CONTROL_PKG_DIR = os.path.join(SRC_DIR, "mchs_control")
if CONTROL_PKG_DIR not in sys.path:  # пакет mchs_control (чистая логика)
    sys.path.insert(0, CONTROL_PKG_DIR)

from mchs_control.route_inspection import (  # noqa: E402
    DEFAULT_DETECTION_TOPIC,
    HOVER_CRITICAL_S,
    HOVER_WARNING_S,
    INSPECTION_COOLDOWN_S,
    LINK_LOSS_RTL_S,
    InspectionController,
    InspectionManeuver,
    link_loss_needs_rtl,
    parse_detection_event,
)

try:
    from mchs_core import get_logger, install_crash_handler
except ImportError:  # ядро логов необязательно: узел продолжит работу
    get_logger = None  # type: ignore[assignment]
    install_crash_handler = None  # type: ignore[assignment]

# ---------------------------------------------------------------------------
# Пути и fallback-константы (если config/mission.json недоступен)
# ---------------------------------------------------------------------------

CONFIG_DIR = os.path.join(REPO_ROOT, "config")
MISSION_PATH = os.path.join(CONFIG_DIR, "mission.json")
HAZARDS_PATH = os.path.join(CONFIG_DIR, "hazards.json")

DEFAULT_X_MIN = -92.0
DEFAULT_X_MAX = 92.0
DEFAULT_Y_MIN = -84.0
DEFAULT_Y_MAX = 84.0
DEFAULT_ROW_STEP_M = 28.0
DEFAULT_CRUISE_SPEED_MPS = 12.0
DEFAULT_ARRIVE_RADIUS_M = 3.0
DEFAULT_ALTITUDE_M = 22.0

DEFAULT_CMD_VEL_TOPIC = "/mchs/cmd_vel"
DEFAULT_ODOM_TOPIC = "/mchs/odom"
DEFAULT_SCAN_STATUS_TOPIC = "/mchs/scan_status"
DEFAULT_REPORTS_DIR = "reports"
DEFAULT_MISSION_CMD_FILE = "reports/mission.cmd"
DEFAULT_HAZARDS_FILE = "config/hazards.json"

# Очаги из config/hazards.json; fallback — координаты демо-арены.
DEFAULT_HAZARDS = (
    {"key": "FIRE", "title": "Очаг лесного пожара", "model": "fire_hazard", "x": 60.0, "y": 28.0, "z": 0.2},
    {"key": "OIL", "title": "Разлив нефтепродуктов", "model": "oil_spill_hazard", "x": -58.0, "y": 30.0, "z": 0.02},
    {"key": "FLOOD", "title": "Зона затопления", "model": "flood_hazard", "x": 45.0, "y": -56.0, "z": 0.02},
    {"key": "WASTE", "title": "Токсичные отходы", "model": "waste_hazard", "x": 68.0, "y": -72.0, "z": 0.4},
)

# Фазы полёта для /mchs/scan_status.
PHASE_WAIT_ODOM = "ОЖИДАНИЕ ОДОМЕТРИИ"
PHASE_OUTBOUND = "ВЫХОД В ЗОНУ"
PHASE_SCAN = "СКАНИРОВАНИЕ"
PHASE_RTL = "ВОЗВРАТ НА БАЗУ"
PHASE_WAIT = "ОЖИДАНИЕ"
PHASE_INSPECT = "ИНСПЕКЦИЯ ОЧАГА"

# Периоды (с).
CONTROL_PERIOD_S = 0.1
STATUS_PERIOD_S = 1.0
LOG_PERIOD_S = 3.0
CMD_CHECK_PERIOD_S = 1.0
ODOM_TIMEOUT_S = 2.0
HAZARD_OBSERVE_RADIUS_M = 35.0


# ---------------------------------------------------------------------------
# Конфигурация
# ---------------------------------------------------------------------------


@dataclass
class MissionConfig:
    """План полёта и параметры сканирования из config/mission.json."""

    x_min: float = DEFAULT_X_MIN
    x_max: float = DEFAULT_X_MAX
    y_min: float = DEFAULT_Y_MIN
    y_max: float = DEFAULT_Y_MAX
    row_step_m: float = DEFAULT_ROW_STEP_M
    cruise_speed: float = DEFAULT_CRUISE_SPEED_MPS
    arrive_radius: float = DEFAULT_ARRIVE_RADIUS_M
    altitude: float = DEFAULT_ALTITUDE_M
    cmd_vel_topic: str = DEFAULT_CMD_VEL_TOPIC
    odom_topic: str = DEFAULT_ODOM_TOPIC
    scan_status_topic: str = DEFAULT_SCAN_STATUS_TOPIC
    reports_dir: str = ""
    mission_cmd_file: str = ""
    hazards: list = field(default_factory=list)


def _resolve_path(path: str, base: str = REPO_ROOT) -> str:
    """Превращает путь из конфига в абсолютный (относительно корня репозитория)."""
    if not isinstance(path, str) or not path:
        return path
    return path if os.path.isabs(path) else os.path.abspath(os.path.join(base, path))


def _load_json(path: str) -> dict:
    """Читает JSON-файл, возвращая {} при любой ошибке (конфиг необязателен)."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def _as_float(mapping: dict, key: str, default: float) -> float:
    """Безопасно достаёт float из словаря конфигурации."""
    try:
        value = float(mapping.get(key, default))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _as_topic(mapping: dict, key: str, default: str) -> str:
    """Безопасно достаёт имя топика из конфигурации."""
    value = mapping.get(key)
    return value.strip() if isinstance(value, str) and value.strip() else default


def load_hazards(path: str = HAZARDS_PATH) -> list:
    """Загружает очаги из config/hazards.json (FIRE/OIL/FLOOD/WASTE)."""
    data = _load_json(path)
    hazards = []
    for key, item in data.items():
        if not isinstance(item, dict):
            continue
        try:
            x = float(item["x"])
            y = float(item["y"])
        except (KeyError, TypeError, ValueError):
            continue
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        hazards.append(
            {
                "key": str(key),
                "title": str(item.get("title") or key),
                "model": str(item.get("model") or key.lower()),
                "x": x,
                "y": y,
                "z": float(item.get("z", 0.0) or 0.0),
            }
        )
    if not hazards:
        hazards = [dict(item) for item in DEFAULT_HAZARDS]
    return hazards


def load_config(mission_path: str = MISSION_PATH) -> MissionConfig:
    """Собирает MissionConfig из config/mission.json, подставляя fallback-значения."""
    data = _load_json(mission_path)
    scan = data.get("scan") if isinstance(data.get("scan"), dict) else {}
    drone = data.get("drone") if isinstance(data.get("drone"), dict) else {}
    topics = data.get("topics") if isinstance(data.get("topics"), dict) else {}

    row_step = _as_float(scan, "row_step_m", DEFAULT_ROW_STEP_M)
    if row_step <= 0.0:
        row_step = DEFAULT_ROW_STEP_M
    cruise = _as_float(drone, "cruise_speed_mps", DEFAULT_CRUISE_SPEED_MPS)
    if cruise <= 0.0:
        cruise = DEFAULT_CRUISE_SPEED_MPS
    arrive = _as_float(drone, "arrive_radius_m", DEFAULT_ARRIVE_RADIUS_M)
    if arrive <= 0.0:
        arrive = DEFAULT_ARRIVE_RADIUS_M

    hazards_file = _resolve_path(data.get("hazards_file") or DEFAULT_HAZARDS_FILE)
    config = MissionConfig(
        x_min=_as_float(scan, "x_min", DEFAULT_X_MIN),
        x_max=_as_float(scan, "x_max", DEFAULT_X_MAX),
        y_min=_as_float(scan, "y_min", DEFAULT_Y_MIN),
        y_max=_as_float(scan, "y_max", DEFAULT_Y_MAX),
        row_step_m=row_step,
        cruise_speed=cruise,
        arrive_radius=arrive,
        altitude=_as_float(drone, "altitude_m", DEFAULT_ALTITUDE_M),
        cmd_vel_topic=_as_topic(topics, "cmd_vel", DEFAULT_CMD_VEL_TOPIC),
        odom_topic=_as_topic(topics, "odom", DEFAULT_ODOM_TOPIC),
        scan_status_topic=_as_topic(topics, "scan_status", DEFAULT_SCAN_STATUS_TOPIC),
        reports_dir=_resolve_path(data.get("reports_dir") or DEFAULT_REPORTS_DIR),
        mission_cmd_file=_resolve_path(data.get("mission_cmd_file") or DEFAULT_MISSION_CMD_FILE),
        hazards=load_hazards(hazards_file),
    )
    return config


# ---------------------------------------------------------------------------
# Чистая геометрия маршрута (тестируется без ROS/Gazebo)
# ---------------------------------------------------------------------------


def build_scan_waypoints(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    row_step_m: float,
) -> tuple[list, int]:
    """Строит путевые точки змейки по всей арене.

    Возвращает ``(waypoints, rows_total)``, где каждая точка — словарь
    ``{"x": float, "y": float, "row": int}``. На каждый ряд две точки:
    начало и конец прохода. Чётные ряды идут в +X, нечётные — в -X,
    поэтому переход между рядами — короткий вертикальный отрезок
    длиной ``row_step_m`` у края арены.

    После последнего ряда маршрут замыкается на первую точку
    (в цикле автопилота — «непрерывно по кругу»).
    """
    if not math.isfinite(row_step_m) or row_step_m <= 0.0:
        row_step_m = DEFAULT_ROW_STEP_M
    if y_max < y_min:
        y_min, y_max = y_max, y_min
    if x_max < x_min:
        x_min, x_max = x_max, x_min

    span = y_max - y_min
    rows_total = int(round(span / row_step_m)) + 1
    rows_total = max(1, rows_total)

    waypoints = []
    for row in range(rows_total):
        y = min(y_min + row * row_step_m, y_max)
        if row % 2 == 0:
            start_x, end_x = x_min, x_max
        else:
            start_x, end_x = x_max, x_min
        waypoints.append({"x": float(start_x), "y": float(y), "row": row})
        waypoints.append({"x": float(end_x), "y": float(y), "row": row})
    return waypoints, rows_total


def build_scan_status(
    row_index: int,
    rows_total: int,
    progress: float,
    phase: str,
    x: float,
    y: float,
    target_x: float,
    target_y: float,
    ts: float | None = None,
) -> dict:
    """Формирует JSON-совместимый статус для /mchs/scan_status (row — с 1)."""
    try:
        row_number = int(row_index) + 1
    except (TypeError, ValueError):
        row_number = 0
    return {
        "row": row_number,
        "rows_total": int(rows_total),
        "progress": round(max(0.0, min(1.0, float(progress))), 3),
        "phase": str(phase),
        "x": round(float(x), 2),
        "y": round(float(y), 2),
        "target_x": round(float(target_x), 2),
        "target_y": round(float(target_y), 2),
        "ts": round(float(time.time() if ts is None else ts), 3),
    }


# ---------------------------------------------------------------------------
# ROS 2 узел
# ---------------------------------------------------------------------------


class MchsScanPilot(Node):
    """Автопилот сканирования всей арены змейкой + файл-команды оператора."""

    def __init__(self, mission: MissionConfig | None = None) -> None:
        super().__init__("mchs_scan_pilot")
        self.mission = mission or load_config()
        self.logger = get_logger("route_pilot", subsystem="PILOT") if get_logger is not None else None

        self.waypoints, self.rows_total = build_scan_waypoints(
            self.mission.x_min,
            self.mission.x_max,
            self.mission.y_min,
            self.mission.y_max,
            self.mission.row_step_m,
        )
        self.hazards = list(self.mission.hazards)

        # Позиция: фактическая из одометрии, fallback — интеграция команд.
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.have_odom = False
        self.last_odom_time = float("-inf")
        self.est_x = 0.0
        self.est_y = 0.0

        # Состояние миссии.
        self.wp_index = 0
        self.mode = "OUTBOUND"  # OUTBOUND | SCAN | RTL
        self.rtl_arrived = False
        self.last_row_seen = -1
        self.observed_hazards: set = set()
        self.last_progress = 0.0
        self.target_x = self.waypoints[0]["x"]
        self.target_y = self.waypoints[0]["y"]
        self.odom_warned = False

        # Инспекция очага по тревоге зрения: подлёт, доворот камеры, съёмка.
        self.inspector = InspectionController()
        self.maneuver = InspectionManeuver()
        self.inspecting = False
        self.odom_yaw = 0.0
        self._world_vx = 0.0
        self._world_vy = 0.0

        # Таймеры логирования/статуса/команд.
        self.start_time = time.monotonic()
        self.last_control_time = self.start_time
        self.last_status_time = 0.0
        self.last_log_time = 0.0
        self.last_cmd_check_time = 0.0

        try:
            os.makedirs(self.mission.reports_dir, exist_ok=True)
        except OSError:
            pass

        self.cmd_publisher = self.create_publisher(Twist, self.mission.cmd_vel_topic, 10)
        self.status_publisher = self.create_publisher(String, self.mission.scan_status_topic, 10)
        self.create_subscription(Odometry, self.mission.odom_topic, self._on_odom, 10)
        self.create_subscription(String, DEFAULT_DETECTION_TOPIC,
                                 self._on_detection, 10)
        self.create_timer(CONTROL_PERIOD_S, self._control_loop)

        self._log(
            ">>> Автопилот-сканер МЧС запущен: змейка X[%.0f..%.0f] × Y[%.0f..%.0f] м, "
            "шаг %.0f м, рядов: %d, скорость %.0f м/с, высота %.0f м (задаёт плагин)"
            % (
                self.mission.x_min,
                self.mission.x_max,
                self.mission.y_min,
                self.mission.y_max,
                self.mission.row_step_m,
                self.rows_total,
                self.mission.cruise_speed,
                self.mission.altitude,
            )
        )
        self._log(
            "    Очагов для наблюдения: %d; команды оператора: %s"
            % (len(self.hazards), self.mission.mission_cmd_file)
        )

    # -- логирование --------------------------------------------------------

    def _log(self, message: str) -> None:
        """Пишет в get_logger (mchs_core), иначе — в штатный ROS-логгер."""
        if self.logger is not None:
            self.logger.info(message)
        else:
            self.get_logger().info(message)

    def _warn(self, message: str) -> None:
        if self.logger is not None:
            self.logger.warning(message)
        else:
            self.get_logger().warning(message)

    # -- одометрия ----------------------------------------------------------

    def _on_odom(self, message: Odometry) -> None:
        """Запоминает фактическую позицию из /mchs/odom (z не используется)."""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (position.x, position.y, orientation.x, orientation.y,
                  orientation.z, orientation.w)
        if not all(math.isfinite(float(value)) for value in values):
            self._warn("Пропущена некорректная одометрия: координаты или кватернион не finite")
            return
        quaternion_norm = math.sqrt(
            float(orientation.x) ** 2 + float(orientation.y) ** 2
            + float(orientation.z) ** 2 + float(orientation.w) ** 2
        )
        if quaternion_norm < 1e-9:
            self._warn("Пропущена некорректная одометрия: нулевой кватернион")
            return
        self.odom_x = float(position.x)
        self.odom_y = float(position.y)
        self.odom_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        self.have_odom = True
        self.last_odom_time = time.monotonic()
        # резервная оценка стартует с последней фактической точки
        self.est_x = self.odom_x
        self.est_y = self.odom_y

    def _odom_fresh(self, now: float) -> bool:
        return self.have_odom and (now - self.last_odom_time) <= ODOM_TIMEOUT_S

    def _position(self, now: float) -> tuple[float, float]:
        """Фактическая позиция; при отсутствии одометрии >2 с — интеграл команд."""
        if self._odom_fresh(now):
            return self.odom_x, self.odom_y
        return self.est_x, self.est_y

    # -- события зрения -----------------------------------------------------

    def _on_detection(self, message: String) -> None:
        # Тревога зрения: подлёт к очагу, доворот камеры и съёмка крупным планом.
        event = parse_detection_event(message.data)
        if event is None:
            return
        if self.mode != "SCAN":
            return  # на выходе в зону и на RTL инспекция не начинается
        now = time.monotonic()
        if not self.inspector.on_event(event, now):
            return
        hazard_x = event.get("x")
        hazard_y = event.get("y")
        if hazard_x is None or hazard_y is None:
            hazard_x, hazard_y = self.target_x, self.target_y
        hold_s = self.inspector.duration_for(event.get("threat_level"))
        pos_x, pos_y = self._position(now)
        self.maneuver.start(float(hazard_x), float(hazard_y), pos_x, pos_y,
                            hold_s=hold_s,
                            label=str(event.get("hazard_label_ru")
                                      or event.get("hazard_type") or "ЧС"))
        self.target_x, self.target_y = float(hazard_x), float(hazard_y)
        self._log(
            "[ТРЕВОГА] %s %s: подлёт к очагу и съёмка %.0f с "
            "(дистанция %.0f м, X=%.1f Y=%.1f)"
            % (event.get("hazard_type") or "ЧС",
               event.get("threat_level") or "",
               hold_s, self.maneuver.standoff_m,
               float(hazard_x), float(hazard_y))
        )

    # -- команды оператора --------------------------------------------------

    def _poll_mission_cmd(self, now: float) -> None:
        """Раз в секунду читает reports/mission.cmd и удаляет его после чтения."""
        if now - self.last_cmd_check_time < CMD_CHECK_PERIOD_S:
            return
        self.last_cmd_check_time = now
        path = self.mission.mission_cmd_file
        if not path or not os.path.isfile(path):
            return
        try:
            with open(path, "r", encoding="utf-8") as handle:
                text = handle.read(256).strip().lower()
        except OSError as exc:
            self._warn(f"Не удалось прочитать файл-команду {path}: {exc}")
            return
        if not text:
            # возможно, АРМ ещё дописывает файл — попробуем на следующем тике
            return
        try:
            os.remove(path)
        except OSError:
            pass
        command = text.split()[0]
        if command == "rtl":
            if self.mode != "RTL":
                self.mode = "RTL"
                self.rtl_arrived = False
                self.maneuver.stop()
                self.inspecting = False
                self._log("[КОМАНДА] Оператор: возврат на базу (RTL), сканирование приостановлено")
            return
        if command == "scan":
            was_rtl = self.mode == "RTL"
            self.rtl_arrived = False
            self.mode = "SCAN" if self.wp_index > 0 else "OUTBOUND"
            self._log(
                "[КОМАНДА] Оператор: продолжить сканирование всей арены"
                + (" (возврат с базы)" if was_rtl else "")
            )
            return
        self._warn(f"[КОМАНДА] Неизвестная команда в {path}: '{command}' (ожидалось rtl/scan)")

    # -- управление ---------------------------------------------------------

    def _phase(self) -> str:
        if not self.have_odom:
            return PHASE_WAIT_ODOM
        if self.inspecting:
            return PHASE_INSPECT
        if self.mode == "RTL":
            return PHASE_RTL if not self.rtl_arrived else PHASE_WAIT
        if self.mode == "OUTBOUND":
            return PHASE_OUTBOUND
        return PHASE_SCAN

    def _current_row(self) -> int:
        if not self.waypoints:
            return 0
        return int(self.waypoints[self.wp_index % len(self.waypoints)]["row"])

    def _progress(self) -> float:
        if not self.waypoints:
            return 0.0
        return min(1.0, self.wp_index / float(len(self.waypoints)))

    def _world_to_body(self, vx_world: float, vy_world: float) -> tuple[float, float]:
        """Мировая скорость → система корпуса.

        Плагин ``libgazebo_ros_planar_move`` трактует ``cmd_vel`` в системе
        корпуса, поэтому после доворота камеры на очаг мировые компоненты
        скорости нужно повернуть на текущий курс.
        """
        cos_yaw = math.cos(self.odom_yaw)
        sin_yaw = math.sin(self.odom_yaw)
        self._world_vx, self._world_vy = float(vx_world), float(vy_world)
        return (cos_yaw * vx_world + sin_yaw * vy_world,
                -sin_yaw * vx_world + cos_yaw * vy_world)

    def _steer(self, cmd: Twist, target_x: float, target_y: float, dist: float, pos_x: float, pos_y: float) -> None:
        """Скорость к точке с торможением у цели (в мировых осях)."""
        if dist <= 1e-6:
            return
        target_yaw = math.atan2(target_y - pos_y, target_x - pos_x)
        yaw_error = math.atan2(math.sin(target_yaw - self.odom_yaw),
                               math.cos(target_yaw - self.odom_yaw))
        # Camera is fixed to the body: face the next leg before translating.
        cmd.angular.z = max(-1.0, min(1.0, 2.0 * yaw_error))
        speed = min(self.mission.cruise_speed, max(1.0, dist * 2.0))
        if abs(yaw_error) > math.radians(15.0):
            speed = 0.0
        vx_world = speed * (target_x - pos_x) / dist
        vy_world = speed * (target_y - pos_y) / dist
        cmd.linear.x, cmd.linear.y = self._world_to_body(vx_world, vy_world)

    def _advance_waypoint(self, pos_x: float, pos_y: float) -> tuple[float, float, float]:
        """Переходит к следующей точке, если текущая достигнута. Возвращает (tx, ty, dist)."""
        total = len(self.waypoints)
        for _ in range(total):
            point = self.waypoints[self.wp_index % total]
            dist = math.hypot(point["x"] - pos_x, point["y"] - pos_y)
            if dist > self.mission.arrive_radius:
                return point["x"], point["y"], dist
            self.wp_index = (self.wp_index + 1) % total
            if self.mode == "OUTBOUND":
                self.mode = "SCAN"
                self._log(">>> Выход в зону завершён, начало сканирования змейкой")
        point = self.waypoints[self.wp_index % total]
        return point["x"], point["y"], math.hypot(point["x"] - pos_x, point["y"] - pos_y)

    def _update_hazard_observations(self, pos_x: float, pos_y: float, row_index: int) -> None:
        """Логирует очаги в радиусе 35 м не чаще одного раза на очаг за проход (ряд)."""
        if row_index != self.last_row_seen:
            self.last_row_seen = row_index
            self.observed_hazards.clear()
        for hazard in self.hazards:
            if hazard["key"] in self.observed_hazards:
                continue
            if math.hypot(hazard["x"] - pos_x, hazard["y"] - pos_y) <= HAZARD_OBSERVE_RADIUS_M:
                self.observed_hazards.add(hazard["key"])
                self._log(f"[НАБЛЮДЕНИЕ] {hazard['title']}: очаг в зоне видимости")

    def _control_loop(self) -> None:
        """Основной цикл: команды, навигация, публикация Twist и статуса."""
        now = time.monotonic()
        dt = now - self.last_control_time
        self.last_control_time = now
        if not math.isfinite(dt) or dt <= 0.0:
            dt = CONTROL_PERIOD_S
        dt = min(dt, 0.5)

        self._poll_mission_cmd(now)

        # Потеря канала/телеметрии: автономный возврат на базу по счислению.
        if link_loss_needs_rtl(self.have_odom, now - self.last_odom_time,
                               self.mode):
            self.mode = "RTL"
            self.rtl_arrived = False
            self.inspector.cancel()
            self.maneuver.stop()
            self.inspecting = False
            self._warn(
                "[СВЯЗЬ] Телеметрия не поступает > %.0f с — автономный RTL "
                "по счислению" % LINK_LOSS_RTL_S
            )

        pos_x, pos_y = self._position(now)
        cmd = Twist()
        self._world_vx = 0.0
        self._world_vy = 0.0

        # Инспекция очага: подлёт, доворот камеры и съёмка. Маршрут в это
        # время не продвигается — цель остаётся на очаге.
        inspection = self.maneuver.command(now, pos_x, pos_y, self.odom_yaw)
        self.inspecting = bool(inspection.get("active"))
        if inspection.get("done"):
            self._log("[ИНСПЕКЦИЯ] Съёмка очага завершена, продолжаю маршрут")

        if self.inspecting:
            cmd.linear.x, cmd.linear.y = self._world_to_body(
                float(inspection.get("vx") or 0.0),
                float(inspection.get("vy") or 0.0),
            )
            cmd.angular.z = float(inspection.get("yaw_rate") or 0.0)
        elif not self.have_odom:
            # До первой одометрии не двигаемся: кинематический плагин без
            # обратной связи улетел бы вслепую.
            if not self.odom_warned and (now - self.start_time) >= ODOM_TIMEOUT_S:
                self.odom_warned = True
                self._warn(
                    "[ОЖИДАНИЕ] Нет данных %s — дрон в удержании, жду одометрию"
                    % self.mission.odom_topic
                )
            self.target_x = self.waypoints[0]["x"]
            self.target_y = self.waypoints[0]["y"]
        elif self.mode == "RTL":
            self.target_x, self.target_y = 0.0, 0.0
            dist = math.hypot(self.target_x - pos_x, self.target_y - pos_y)
            if dist <= self.mission.arrive_radius:
                if not self.rtl_arrived:
                    self.rtl_arrived = True
                    self._log("[RTL] Дрон у базы, удержание позиции; жду команду 'scan'")
            else:
                self.rtl_arrived = False
                self._steer(cmd, self.target_x, self.target_y, dist, pos_x, pos_y)
        else:
            target_x, target_y, dist = self._advance_waypoint(pos_x, pos_y)
            self.target_x, self.target_y = target_x, target_y
            self._steer(cmd, target_x, target_y, dist, pos_x, pos_y)

        self.cmd_publisher.publish(cmd)

        # Fallback-оценка позиции, если одометрия пропала дольше ODOM_TIMEOUT_S.
        # Считаем по мировой скорости: cmd_vel задаётся в системе корпуса.
        if not self._odom_fresh(now):
            self.est_x += self._world_vx * dt
            self.est_y += self._world_vy * dt

        row_index = self._current_row()
        progress = self._progress()
        self.last_progress = progress
        if self.have_odom:
            self._update_hazard_observations(pos_x, pos_y, row_index)
        phase = self._phase()

        # Статус для АРМ — раз в секунду.
        if now - self.last_status_time >= STATUS_PERIOD_S:
            self.last_status_time = now
            payload = build_scan_status(
                row_index,
                self.rows_total,
                progress,
                phase,
                pos_x,
                pos_y,
                self.target_x,
                self.target_y,
            )
            message = String()
            message.data = json.dumps(payload, ensure_ascii=False)
            self.status_publisher.publish(message)

        # Журнал — раз в 3 секунды.
        if now - self.last_log_time >= LOG_PERIOD_S:
            self.last_log_time = now
            self._log(
                "[%s] ряд %d/%d: X=%.1f Y=%.1f → цель X=%.1f Y=%.1f, прогресс %.0f%%"
                % (
                    phase,
                    row_index + 1,
                    self.rows_total,
                    pos_x,
                    pos_y,
                    self.target_x,
                    self.target_y,
                    progress * 100.0,
                )
            )

    def stop(self) -> None:
        """Публикует нулевую уставку скорости (гашение движения)."""
        try:
            self.cmd_publisher.publish(Twist())
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Точка входа
# ---------------------------------------------------------------------------


def main(args=None) -> int:
    if install_crash_handler is not None:
        try:
            install_crash_handler("PILOT")
        except Exception:
            pass
    rclpy.init(args=args)
    node = None
    try:
        node = MchsScanPilot()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
