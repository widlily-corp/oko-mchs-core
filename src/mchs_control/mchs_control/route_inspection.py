"""Чистая логика реакции автопилота на события зрения.

Модуль намеренно не зависит от ROS 2 и Gazebo: правила «зависания» над
очагом проверяются автотестами без запуска симулятора.
"""

from __future__ import annotations

import json
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
