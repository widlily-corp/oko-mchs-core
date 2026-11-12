"""Чистая логика реакции автопилота на события зрения.

Модуль намеренно не зависит от ROS 2 и Gazebo: правила «зависания» над
очагом проверяются автотестами без запуска симулятора.
"""

from __future__ import annotations

import json
import math
from typing import Optional

#: Топик событий ЧС от бортовой ноды зрения.
DEFAULT_DETECTION_TOPIC = "/mchs/detection_event"

#: Длительность зависания над очагом, с.
HOVER_WARNING_S = 5.0
HOVER_CRITICAL_S = 10.0

#: Пауза между инспекциями разных очагов, с.
INSPECTION_COOLDOWN_S = 5.0

#: Порог потери телеметрии, после которого автопилот сам уходит в RTL, с.
LINK_LOSS_RTL_S = 15.0

#: Дистанция, с которой очаг снимается «крупным планом» (камера 45°), м.
INSPECTION_STANDOFF_M = 12.0
#: Радиус прибытия в точку съёмки, м.
INSPECTION_ARRIVE_M = 2.5
#: Скорость подлёта к точке съёмки, м/с.
INSPECTION_SPEED_MPS = 6.0
#: Коэффициент и предел скорости доворота на очаг, 1/с и рад/с.
YAW_GAIN = 1.4
MAX_YAW_RATE = 0.9


def wrap_angle(rad: float) -> float:
    """Приводит угол к диапазону (-pi, pi]."""
    return (float(rad) + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, low: float, high: float) -> float:
    """Ограничивает значение отрезком [low, high]."""
    return max(low, min(high, float(value)))


class InspectionManeuver:
    """Подлёт к очагу, доворот камеры на цель и съёмка с ближней дистанции.

    Камера БПЛА закреплена под 45° вниз, поэтому «крупный план» — это не
    полёт строго над очагом (там он уходит из кадра), а точка на расстоянии
    ``standoff_m`` от цели: дрон разворачивается на очаг и висит, пока
    бортовая нода фиксирует фото-улику.

    Класс не зависит от ROS и тестируется отдельно от Gazebo.
    """

    def __init__(self, standoff_m: float = INSPECTION_STANDOFF_M,
                 arrive_m: float = INSPECTION_ARRIVE_M,
                 speed_mps: float = INSPECTION_SPEED_MPS) -> None:
        self.standoff_m = float(standoff_m)
        self.arrive_m = float(arrive_m)
        self.speed_mps = float(speed_mps)
        self.active = False
        self.phase = "IDLE"  # IDLE | APPROACH | HOLD
        self.hazard = (0.0, 0.0)
        self.target = (0.0, 0.0)
        self.hold_s = 0.0
        self.label = ""
        self._hold_started = 0.0

    def start(self, hazard_x: float, hazard_y: float,
              drone_x: float, drone_y: float, hold_s: float,
              label: str = "") -> None:
        """Начинает инспекцию очага в точке (hazard_x, hazard_y)."""
        self.hazard = (float(hazard_x), float(hazard_y))
        dx = float(drone_x) - self.hazard[0]
        dy = float(drone_y) - self.hazard[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            # Очаг ровно под дроном: отходим по направлению на север.
            ux, uy = 0.0, 1.0
        else:
            ux, uy = dx / dist, dy / dist
        self.target = (
            self.hazard[0] + ux * self.standoff_m,
            self.hazard[1] + uy * self.standoff_m,
        )
        self.hold_s = max(0.0, float(hold_s))
        self.label = str(label or "")
        self.phase = "APPROACH"
        self.active = True
        self._hold_started = 0.0

    def stop(self) -> None:
        """Прерывает инспекцию (например, при потере канала)."""
        self.active = False
        self.phase = "IDLE"

    def command(self, now: float, drone_x: float, drone_y: float,
                yaw_rad: float) -> dict:
        """Возвращает уставку движения на текущем такте.

        ``vx``/``vy`` — в мировой системе (X — восток, Y — север),
        ``yaw_rate`` — скорость доворота, рад/с.
        """
        if not self.active:
            return {"active": False, "phase": self.phase, "vx": 0.0,
                    "vy": 0.0, "yaw_rate": 0.0, "done": False}

        error = wrap_angle(
            math.atan2(self.hazard[1] - drone_y, self.hazard[0] - drone_x)
            - float(yaw_rad)
        )
        yaw_rate = clamp(YAW_GAIN * error, -MAX_YAW_RATE, MAX_YAW_RATE)

        if self.phase == "APPROACH":
            tx, ty = self.target
            dist = math.hypot(tx - drone_x, ty - drone_y)
            if dist <= self.arrive_m:
                self.phase = "HOLD"
                self._hold_started = now
                return {"active": True, "phase": "HOLD", "vx": 0.0, "vy": 0.0,
                        "yaw_rate": yaw_rate, "done": False}
            speed = min(self.speed_mps, max(1.0, dist * 1.2))
            return {"active": True, "phase": "APPROACH",
                    "vx": (tx - drone_x) / dist * speed,
                    "vy": (ty - drone_y) / dist * speed,
                    "yaw_rate": yaw_rate, "done": False}

        # HOLD: висим и держим очаг в кадре до конца съёмки.
        if now - self._hold_started >= self.hold_s:
            self.active = False
            self.phase = "IDLE"
            return {"active": False, "phase": "IDLE", "vx": 0.0, "vy": 0.0,
                    "yaw_rate": 0.0, "done": True}
        return {"active": True, "phase": "HOLD", "vx": 0.0, "vy": 0.0,
                "yaw_rate": yaw_rate, "done": False}

    def remaining_s(self, now: float) -> float:
        """Сколько секунд съёмки осталось (0 — если идёт подлёт)."""
        if not self.active or self.phase != "HOLD":
            return 0.0
        return max(0.0, self.hold_s - (now - self._hold_started))


def link_loss_needs_rtl(have_odom: bool, odom_age_s: float, mode: str,
                        threshold_s: float = LINK_LOSS_RTL_S) -> bool:
    """Пора ли уходить в RTL из-за потери канала (телеметрии).

    Работает только после первой одометрии и только в режиме сканирования:
    до взлёта и на уже начатом возврате поведение не меняется.
    """
    return (bool(have_odom) and str(mode).upper() == "SCAN"
            and float(odom_age_s) > float(threshold_s))


def parse_detection_event(payload: str) -> Optional[dict]:
    """Разбирает JSON события ``/mchs/detection_event``.

    Возвращает ``None`` для мусора и для уровней ниже WARNING
    (ADVISORY не прерывает патруль).
    """
    try:
        data = json.loads(payload)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    threat = str(data.get("threat_level") or "").upper()
    if threat not in ("WARNING", "CRITICAL"):
        return None
    geo = data.get("target_geo") or {}
    if not isinstance(geo, dict):
        geo = {}
    return {
        "incident_id": str(data.get("incident_id") or ""),
        "hazard_type": str(data.get("hazard_type") or ""),
        "hazard_label_ru": str(data.get("hazard_label_ru") or ""),
        "threat_level": threat,
        "area_sq_m": float(data.get("area_sq_m") or 0.0),
        "x": geo.get("abs_x"),
        "y": geo.get("abs_y"),
    }


class InspectionController:
    """Решает, когда автопилот зависает над очагом и сколько висит.

    Правила (совпадают с заявленными в регламенте):

    * WARNING — зависание 5 с, CRITICAL — 10 с;
    * один очаг (``incident_id``) инспектируется один раз;
    * между инспекциями пауза ``cooldown_s``;
    * повторные события во время зависания его не продлевают.
    """

    def __init__(self, warning_s: float = HOVER_WARNING_S,
                 critical_s: float = HOVER_CRITICAL_S,
                 cooldown_s: float = INSPECTION_COOLDOWN_S) -> None:
        self.warning_s = float(warning_s)
        self.critical_s = float(critical_s)
        self.cooldown_s = float(cooldown_s)
        self.inspections = 0
        self._seen = set()
        self._active = False
        self._started_at = 0.0
        self._duration = 0.0
        self._last_end = 0.0

    def duration_for(self, threat: str) -> float:
        """Длительность зависания для уровня угрозы, с."""
        return self.critical_s if str(threat).upper() == "CRITICAL" \
            else self.warning_s

    def on_event(self, event: Optional[dict], now: float) -> bool:
        """Начинает инспекцию, если событие подходит. True — зависаем."""
        if not event:
            return False
        incident_id = str(event.get("incident_id") or "")
        if incident_id and incident_id in self._seen:
            return False
        if self._active:
            return False
        if self._last_end and now - self._last_end < self.cooldown_s:
            return False
        if incident_id:
            self._seen.add(incident_id)
        self._active = True
        self._started_at = now
        self._duration = self.duration_for(event.get("threat_level"))
        self.inspections += 1
        return True

    def is_active(self, now: float) -> bool:
        """True, пока идёт зависание; по истечении срока — гасит его."""
        if not self._active:
            return False
        if now - self._started_at >= self._duration:
            self._active = False
            self._last_end = now
            return False
        return True

    def remaining(self, now: float) -> float:
        """Сколько секунд зависания осталось."""
        if not self._active:
            return 0.0
        return max(0.0, self._duration - (now - self._started_at))

    def cancel(self) -> None:
        """Прерывает зависание (например, при потере канала)."""
        self._active = False
        self._last_end = 0.0
