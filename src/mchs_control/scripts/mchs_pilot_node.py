#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Автономный автопилот БПЛА «ОКО МЧС» — конечный автомат полёта (ROS 2 Humble).

Машина состояний (FSM)
----------------------

    TAKEOFF           набор рабочей высоты Z = 8,0 м (vz = +1,0 м/с)
        |
    PATROL_GRID       «змейка» по сектору поиска со скоростью 2,0 м/с
        |                 ^
        |  тревога        |  возврат в точку прерывания
        v                 |
    HOVER_INSPECTION  зависание 5 с (WARNING) или 10 с (CRITICAL) + протокол ЦУКС
        |
    RTL               разворот и полёт в точку старта (0, 0, 8)
        |
    LAND              плавное снижение до Z <= 0,2 м и остановка моторов
        |
    FINISHED          миссия выполнена

Входы/выходы
------------

    подписки:  /mchs/detection_event (std_msgs/String, JSON) — тревога от зрения
               /mchs/mission_command (std_msgs/String)        — команды оператора RTL/LAND
               /mchs/odom            (nav_msgs/Odometry)      — высота и позиция
    публикует: /mchs/cmd_vel         (geometry_msgs/Twist)    — уставки скорости
               /mchs/mission_status  (std_msgs/String, JSON)  — статус для АРМ

Почему логика вынесена в отдельный класс
----------------------------------------
``FlightPlanner`` не импортирует ROS: он получает ``dt`` и событие тревоги,
а возвращает уставку скорости. Это позволяет прогнать всю миссию в обычном
Python (``--dry-run``) и покрыть её тестами без Gazebo.

Запуск
------

    source /opt/ros/humble/setup.bash
    python3 src/mchs_control/scripts/mchs_pilot_node.py              # ROS 2
    python3 src/mchs_control/scripts/mchs_pilot_node.py --dry-run    # без ROS
    python3 src/mchs_control/scripts/mchs_pilot_node.py --dry-run --detect-at 8 --hazard FLOOD --area 120

Ручная имитация тревоги (когда нода зрения ещё не публикует события)::

    ros2 topic pub --once /mchs/detection_event std_msgs/msg/String \
      "{data: '{\"hazard_type\": \"FIRE\", \"threat_level\": \"CRITICAL\", \"area_sq_m\": 48.5, \"target_geo\": {\"rel_x\": 12.4, \"rel_y\": -3.8}, \"confidence\": 0.94}'}"
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:  # импорт работает при запуске из любой папки
    sys.path.insert(0, SCRIPT_DIR)

from generate_report import (  # noqa: E402
    DEFAULT_OUTPUT_DIR,
    classify_threat,
    generate_incident_protocol,
)


# ---------------------------------------------------------------------------
# Фазы полёта
# ---------------------------------------------------------------------------


class Phase(Enum):
    """Состояния конечного автомата. Значение — код, как в ТЗ."""

    TAKEOFF = "TAKEOFF"
    PATROL_GRID = "PATROL_GRID"
    HOVER_INSPECTION = "HOVER_INSPECTION"
    RTL = "RTL"
    LAND = "LAND"
    FINISHED = "FINISHED"

    @property
    def label(self) -> str:
        """Русская подпись фазы для логов и протокола."""
        return _PHASE_LABELS[self]


_PHASE_LABELS = {
    Phase.TAKEOFF: "ВЗЛЁТ",
    Phase.PATROL_GRID: "ПАТРУЛЬ",
    Phase.HOVER_INSPECTION: "ЗАВИСАНИЕ",
    Phase.RTL: "ВОЗВРАТ",
    Phase.LAND: "ПОСАДКА",
    Phase.FINISHED: "ЗАВЕРШЕНО",
}


# ---------------------------------------------------------------------------
# Базовые структуры
# ---------------------------------------------------------------------------


@dataclass
class Command:
    """Уставка скорости для ``geometry_msgs/Twist`` в системе координат дрона."""

    vx: float = 0.0  # вперёд, м/с
    vy: float = 0.0  # влево, м/с
    vz: float = 0.0  # вверх, м/с
    wz: float = 0.0  # поворот, рад/с

    def __str__(self) -> str:
        return f"vx={self.vx:+.2f} vz={self.vz:+.2f} wz={self.wz:+.2f}"


@dataclass
class Segment:
    """Участок маршрута: прямолинейный полёт или разворот заданной длительности."""

    title: str
    duration: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    wz: float = 0.0

    def command(self) -> Command:
        """Уставка скорости этого участка."""
        return Command(self.vx, self.vy, self.vz, self.wz)


@dataclass
class Detection:
    """Событие ЧС, пришедшее из ``/mchs/detection_event``.

    Поля повторяют контракт модуля зрения, чтобы автопилот не зависел от
    формата строки и мог разбирать как словарь, так и JSON-текст.
    """

    hazard_type: str = "FIRE"
    threat_level: str = "CRITICAL"
    area_sq_m: float = 0.0
    target_geo: dict = field(default_factory=dict)
    confidence: float = 0.0
    timestamp: str = ""
    # Поля, которые дополнительно передаёт модуль зрения (mchs_vision).
    incident_id: str = ""
    track_id: str = ""
    label_ru: str = ""
    uav_pose: dict = field(default_factory=dict)
    detections_count: int = 0

    @property
    def is_alarm(self) -> bool:
        """Тревогой считаются только WARNING и CRITICAL (по ТЗ)."""
        return self.threat_level in ("WARNING", "CRITICAL")

    @property
    def abs_position(self) -> Optional[tuple[float, float]]:
        """Координаты очага на полигоне, если зрение их передало."""
        geo = self.target_geo or {}
        if "abs_x" in geo and "abs_y" in geo:
            return float(geo["abs_x"]), float(geo["abs_y"])
        return None

    @classmethod
    def from_payload(cls, payload) -> Optional["Detection"]:
        """Разбирает JSON-строку или словарь. Возвращает ``None``, если формат не понят."""
        if isinstance(payload, str):
            try:
                payload = json.loads(payload)
            except (ValueError, TypeError):
                return None
        if not isinstance(payload, dict):
            return None

        hazard = str(payload.get("hazard_type", "FIRE") or "FIRE").upper()
        area = _to_float(payload.get("area_sq_m", payload.get("area_m2", 0.0)))
        threat = str(payload.get("threat_level", "") or "").upper()
        if threat not in ("ADVISORY", "WARNING", "CRITICAL"):
            threat = classify_threat(area)

        geo = payload.get("target_geo")
        if not isinstance(geo, dict):
            # Допускаем плоскую форму {"x": .., "y": ..} от сторонних источников.
            geo = {}
            if "x" in payload and "y" in payload:
                geo = {"abs_x": payload["x"], "abs_y": payload["y"]}

        uav_pose = payload.get("uav_pose")
        detections = payload.get("detections")

        return cls(
            hazard_type=hazard,
            threat_level=threat,
            area_sq_m=area,
            target_geo=dict(geo),
            confidence=_to_float(payload.get("confidence", 0.0)),
            timestamp=str(payload.get("timestamp", "")),
            incident_id=str(payload.get("incident_id", "") or ""),
            track_id=str(payload.get("track_id", "") or ""),
            label_ru=str(payload.get("hazard_label_ru", "") or ""),
            uav_pose=dict(uav_pose) if isinstance(uav_pose, dict) else {},
            detections_count=len(detections) if isinstance(detections, list) else 0,
        )


def _to_float(value, default: float = 0.0) -> float:
    """Мягкое приведение к float: строки и None не должны ронять ноду."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _clamp(value: float, low: float, high: float) -> float:
    """Ограничивает значение диапазоном."""
    return max(low, min(high, value))


def _wrap_angle(angle: float) -> float:
    """Приводит угол к диапазону (-pi, pi] — иначе ошибка курса «скачет»."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Конечный автомат миссии
# ---------------------------------------------------------------------------


class FlightPlanner:
    """Логика миссии без ROS: вход — ``dt`` и тревога, выход — уставка скорости."""

    def __init__(
        self,
        takeoff_altitude: float = 8.0,
        climb_speed: float = 1.0,
        cruise_speed: float = 2.0,
        lane_time: float = 15.0,
        lane_shift_m: float = 5.0,
        shift_speed: float = 0.5,
        turn_rate: float = 0.6,
        hover_warning_s: float = 5.0,
        hover_critical_s: float = 10.0,
        landing_speed: float = 0.8,
        lanes: int = 2,
        rtl_arrival_radius_m: float = 0.8,
        rtl_timeout_s: float = 180.0,
        battery_full_pct: float = 100.0,
        battery_drain_pct_per_s: float = 100.0 / (45.0 * 60.0),
        battery_rtl_threshold_pct: float = 20.0,
        max_inspections: int = 1,
        inspection_cooldown_s: float = 5.0,
        hotspot_radius_m: float = 15.0,
    ) -> None:
        if lanes < 1:
            raise ValueError("Количество галсов (lanes) должно быть >= 1")
        if min(climb_speed, cruise_speed, shift_speed, turn_rate, landing_speed) <= 0.0:
            raise ValueError("Все скорости должны быть > 0")

        self.takeoff_altitude = takeoff_altitude
        self.climb_speed = climb_speed
        self.cruise_speed = cruise_speed
        self.lane_time = lane_time
        self.lane_shift_m = lane_shift_m
        self.shift_speed = shift_speed
        self.turn_rate = turn_rate
        self.hover_warning_s = hover_warning_s
        self.hover_critical_s = hover_critical_s
        self.landing_speed = landing_speed
        self.lanes = lanes

        self.rtl_arrival_radius_m = rtl_arrival_radius_m
        self.rtl_timeout_s = rtl_timeout_s

        self.battery_full_pct = battery_full_pct
        self.battery_drain_pct_per_s = max(0.0, battery_drain_pct_per_s)
        self.battery_rtl_threshold_pct = battery_rtl_threshold_pct

        self.max_inspections = max(0, max_inspections)
        self.inspection_cooldown_s = inspection_cooldown_s
        self.hotspot_radius_m = hotspot_radius_m

        # Разворот на 90°: t = (pi/2) / omega. Смещение в полосу: t = S / v.
        self.turn_duration = (math.pi / 2.0) / turn_rate
        self.shift_duration = lane_shift_m / shift_speed

        self.patrol_route: list[Segment] = self._build_patrol_route()
        self.reset()

    # -- маршрут -------------------------------------------------------------

    def _build_patrol_route(self) -> list[Segment]:
        """Строит «змейку»: N галсов, между ними два разворота и смещение.

        Соседние галсы проходятся в противоположных направлениях, поэтому
        развороты чередуются: на чётном галсе два поворота влево, на нечётном —
        два вправо. Иначе дрон после разворота уходил бы не в соседнюю полосу,
        а обратно в предыдущую.
        """
        route: list[Segment] = []
        for lane in range(self.lanes):
            route.append(
                Segment(
                    title=f"Галс {lane + 1}: полёт вдоль просеки",
                    duration=self.lane_time,
                    vx=self.cruise_speed,
                )
            )
            if lane == self.lanes - 1:
                continue
            direction = 1.0 if lane % 2 == 0 else -1.0
            side = "влево" if direction > 0 else "вправо"
            route.append(
                Segment(
                    title=f"Разворот на 90° {side}",
                    duration=self.turn_duration,
                    wz=direction * self.turn_rate,
                )
            )
            route.append(
                Segment(
                    title=f"Смещение на {self.lane_shift_m:.0f} м в соседнюю полосу",
                    duration=self.shift_duration,
                    vx=self.shift_speed,
                )
            )
            route.append(
                Segment(
                    title=f"Разворот на 90° {side}",
                    duration=self.turn_duration,
                    wz=direction * self.turn_rate,
                )
            )
        return route

    # -- состояние -----------------------------------------------------------

    def reset(self) -> None:
        """Сбрасывает автомат в начало миссии."""
        self.phase = Phase.TAKEOFF
        self.segment_index = 0
        self.segment_time = 0.0

        self.hover_elapsed = 0.0
        self.hover_duration = self.hover_warning_s
        self.inspections = 0
        self.last_inspection_end = -math.inf
        self.active_detection: Optional[Detection] = None
        self.hotspots: list[tuple[float, float]] = []

        self.elapsed = 0.0
        self.rtl_elapsed = 0.0
        self.distance_travelled = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.heading_rad = 0.0

        self.battery_pct = self.battery_full_pct
        self._battery_rtl_done = False
        self._saved_state: Optional[tuple[Phase, int, float]] = None
        self.phase_log: list[dict] = [{"t": 0.0, "phase": self.phase.value}]

    @property
    def is_finished(self) -> bool:
        """Миссия завершена."""
        return self.phase is Phase.FINISHED

    @property
    def current_segment(self) -> Optional[Segment]:
        """Текущий участок маршрута (None, если дрон не в патруле)."""
        if self.phase is Phase.PATROL_GRID and self.segment_index < len(self.patrol_route):
            return self.patrol_route[self.segment_index]
        return None

    # -- основной такт -------------------------------------------------------

    def tick(self, dt: float, detection: Optional[Detection] = None) -> tuple[Command, list[str]]:
        """Продвигает миссию на ``dt`` секунд. Возвращает (команду, события)."""
        events: list[str] = []
        dt = max(0.0, dt)
        self.elapsed += dt
        self._update_battery(dt, events)

        if self.phase is Phase.FINISHED:
            return Command(), events

        if detection is not None and detection.is_alarm and self._can_inspect(detection):
            self._begin_inspection(detection)
            events.append("inspection_started")
        elif detection is not None and not detection.is_alarm and self.phase is Phase.PATROL_GRID:
            events.append("detection_ignored")

        handler = {
            Phase.TAKEOFF: self._tick_takeoff,
            Phase.PATROL_GRID: self._tick_patrol,
            Phase.HOVER_INSPECTION: self._tick_hover,
            Phase.RTL: self._tick_rtl,
            Phase.LAND: self._tick_land,
        }[self.phase]
        return handler(dt, events)

    # -- фазы ----------------------------------------------------------------

    def _tick_takeoff(self, dt: float, events: list[str]) -> tuple[Command, list[str]]:
        """Плавный набор высоты до рабочего эшелона."""
        command = Command(vz=self.climb_speed)
        self._integrate(command, dt)
        if self.z >= self.takeoff_altitude - 1e-9:
            self.z = self.takeoff_altitude
            self._start_phase(Phase.PATROL_GRID)
            events.append("takeoff_complete")
        return command, events

    def _tick_patrol(self, dt: float, events: list[str]) -> tuple[Command, list[str]]:
        """Полёт по галсам «змейкой»."""
        segment = self.current_segment
        if segment is None:
            self._start_phase(Phase.RTL)
            events.append("patrol_complete")
            return Command(), events

        step = min(dt, max(0.0, segment.duration - self.segment_time))
        command = segment.command()
        self._integrate(command, step)
        self.segment_time += step

        if self.segment_time >= segment.duration - 1e-9:
            self.segment_index += 1
            self.segment_time = 0.0
            if self.segment_index >= len(self.patrol_route):
                self._start_phase(Phase.RTL)
                events.append("patrol_complete")
            else:
                events.append("segment_complete")
        return command, events

    def _tick_hover(self, dt: float, events: list[str]) -> tuple[Command, list[str]]:
        """Зависание над очагом для стабильной съёмки."""
        self.hover_elapsed += dt
        if self.hover_elapsed + 1e-9 >= self.hover_duration:
            self._end_inspection()
            events.append("inspection_finished")
        return Command(), events

    def _tick_rtl(self, dt: float, events: list[str]) -> tuple[Command, list[str]]:
        """Возврат в точку старта (0, 0, 8) по вектору на базу."""
        distance = math.hypot(self.x, self.y)
        if distance <= self.rtl_arrival_radius_m:
            self._start_phase(Phase.LAND)
            events.append("rtl_complete")
            return Command(), events

        self.rtl_elapsed += dt
        if self.rtl_elapsed > self.rtl_timeout_s:  # страховка от «зацикливания»
            self._start_phase(Phase.LAND)
            events.append("rtl_timeout")
            return Command(), events

        # Курс на базу: дрон разворачивается и летит вперёд, подруливая поворотом.
        error = _wrap_angle(math.atan2(-self.y, -self.x) - self.heading_rad)
        command = Command(
            vx=self.cruise_speed * max(0.15, math.cos(error)),
            wz=_clamp(2.0 * error, -0.9, 0.9),
            vz=_clamp((self.takeoff_altitude - self.z) * 1.0, -0.5, self.climb_speed),
        )
        self._integrate(command, dt)
        return command, events

    def _tick_land(self, dt: float, events: list[str]) -> tuple[Command, list[str]]:
        """Плавное снижение до касания и остановка моторов."""
        if self.z <= 0.2:
            self._start_phase(Phase.FINISHED)
            events.append("landed")
            return Command(), events
        command = Command(vz=-self.landing_speed)
        self._integrate(command, dt)
        if self.z <= 0.2:
            self.z = 0.0
        return command, events

    # -- переходы и модель движения -----------------------------------------

    def _integrate(self, command: Command, step: float) -> None:
        """Оценка положения дрона по командам (резерв, если одометрии нет).

        Модель «рулевого колеса»: скорость направлена по курсу, курс
        интегрируется из угловой скорости. Этого достаточно, чтобы работали
        и dry-run, и RTL по вектору на базу.
        """
        heading = self.heading_rad
        self.x += (command.vx * math.cos(heading) - command.vy * math.sin(heading)) * step
        self.y += (command.vx * math.sin(heading) + command.vy * math.cos(heading)) * step
        self.z += command.vz * step
        self.heading_rad += command.wz * step
        self.distance_travelled += math.hypot(command.vx, command.vy) * step

    def _start_phase(self, phase: Phase) -> None:
        """Переключает фазу и пишет отметку в журнал миссии."""
        self.phase = phase
        self.segment_index = 0
        self.segment_time = 0.0
        self.phase_log.append({"t": round(self.elapsed, 2), "phase": phase.value})

    # -- инспекция очага -----------------------------------------------------

    def _hotspot_position(self, detection: Detection) -> tuple[float, float]:
        """Координаты очага для дедупликации: из события, иначе позиция дрона."""
        return detection.abs_position or (self.x, self.y)

    def _can_inspect(self, detection: Detection) -> bool:
        """Можно ли перейти в HOVER_INSPECTION по этому событию.

        Ограничители:
        1. зависаем только в патруле — прерывать взлёт/посадку опасно;
        2. не больше ``max_inspections`` инспекций за миссию;
        3. пауза ``inspection_cooldown_s`` между инспекциями;
        4. очаг ближе ``hotspot_radius_m`` к уже исследованному считается тем же.
        """
        if self.phase is not Phase.PATROL_GRID:
            return False
        if self.inspections >= self.max_inspections:
            return False
        if (self.elapsed - self.last_inspection_end) < self.inspection_cooldown_s:
            return False
        spot_x, spot_y = self._hotspot_position(detection)
        for hot_x, hot_y in self.hotspots:
            if math.hypot(spot_x - hot_x, spot_y - hot_y) <= self.hotspot_radius_m:
                return False
        return True

    def _begin_inspection(self, detection: Detection) -> None:
        """Замирает над очагом, запомнив точку прерывания патруля."""
        self._saved_state = (self.phase, self.segment_index, self.segment_time)
        self.phase = Phase.HOVER_INSPECTION
        self.hover_elapsed = 0.0
        # ТЗ: 5–10 секунд. Критичный очаг снимаем дольше — нужно больше кадров.
        self.hover_duration = (
            self.hover_critical_s if detection.threat_level == "CRITICAL" else self.hover_warning_s
        )
        self.inspections += 1
        self.active_detection = detection
        self.hotspots.append(self._hotspot_position(detection))
        self.phase_log.append({"t": round(self.elapsed, 2), "phase": self.phase.value})

    def _end_inspection(self) -> None:
        """Возвращает дрон в точку маршрута, где патруль был прерван."""
        if self._saved_state is not None:
            phase, index, segment_time = self._saved_state
            self.phase = phase
            self.segment_index = index
            self.segment_time = segment_time
            self._saved_state = None
        else:
            self.phase = Phase.PATROL_GRID
        self.phase_log.append(
            {"t": round(self.elapsed, 2), "phase": self.phase.value, "note": "патруль продолжен"}
        )
        self.last_inspection_end = self.elapsed
        self.active_detection = None

    # -- батарея -------------------------------------------------------------

    def _update_battery(self, dt: float, events: list[str]) -> None:
        """Разряжает АКБ и уводит дрон на базу при низком заряде.

        Модель: 100 % за 45 минут (≈ 0,037 %/с). Ниже 20 % дрон бросает
        патруль и идёт на RTL — это и есть «автономный возврат для замены АКБ».
        """
        self.battery_pct = max(0.0, self.battery_pct - self.battery_drain_pct_per_s * dt)
        if self._battery_rtl_done or self.phase in (Phase.RTL, Phase.LAND, Phase.FINISHED):
            return
        if self.battery_pct <= self.battery_rtl_threshold_pct:
            self._battery_rtl_done = True
            events.append("battery_low")
            self._saved_state = None
            self._start_phase(Phase.RTL)

    # -- команды оператора ---------------------------------------------------

    def request_rtl(self) -> bool:
        """Досрочный возврат на базу по команде оператора.

        Возвращает True, если команда принята. Из посадки и завершённой
        миссии возврат не запрашивается — дрон уже на земле.
        """
        if self.phase in (Phase.LAND, Phase.FINISHED):
            return False
        self._saved_state = None
        self._start_phase(Phase.RTL)
        return True

    def request_land(self) -> bool:
        """Команда «садиться сейчас»: миссия прерывается, дрон снижается."""
        if self.phase is Phase.FINISHED:
            return False
        self._saved_state = None
        self._start_phase(Phase.LAND)
        return True

    # -- телеметрия ----------------------------------------------------------

    def stats(self) -> dict:
        """Сводка полёта для протокола и логов."""
        return {
            "elapsed_s": round(self.elapsed, 1),
            "distance_flown_m": round(self.distance_travelled, 1),
            "altitude_m": round(self.z, 1),
            "heading_deg": round(math.degrees(self.heading_rad) % 360.0, 1),
            "position_local_m": {"x": round(self.x, 2), "y": round(self.y, 2)},
            "distance_to_home_m": round(math.hypot(self.x, self.y), 2),
            "lanes": self.lanes,
            "inspections": self.inspections,
            "battery_percent": round(self.battery_pct, 1),
            "phases": list(self.phase_log),
        }


# ---------------------------------------------------------------------------
# Режим без ROS 2
# ---------------------------------------------------------------------------

EVENT_TEXT = {
    "takeoff_complete": ">>> Высота 8,0 м набрана. Начинаю патрулирование «змейкой».",
    "segment_complete": "    участок маршрута пройден, следующий этап",
    "patrol_complete": ">>> Патруль сектора завершён. Возврат на базу (RTL).",
    "rtl_complete": ">>> Дрон в точке старта. Снижаюсь.",
    "rtl_timeout": "!!! RTL не уложился в лимит времени — сажусь на месте.",
    "landed": ">>> Посадка выполнена. Моторы остановлены. Миссия завершена.",
    "inspection_started": "!!! ТРЕВОГА: очаг ЧС. Зависание для детальной съёмки и протокола ЦУКС.",
    "inspection_finished": "    Инспекция окончена, продолжаю патруль с точки прерывания.",
    "battery_low": "!!! Заряд АКБ ниже порога: принудительный возврат на базу.",
    "detection_ignored": "    Угроза уровня ADVISORY — продолжаю патруль без зависания.",
}


def _print_route(planner: FlightPlanner) -> None:
    """Печатает таблицу маршрута патрулирования."""
    print()
    print("=== МАРШРУТ ПАТРУЛИРОВАНИЯ «ЗМЕЙКА» ===")
    print("  №   этап                                        длит., с   vx, м/с   wz, рад/с")
    for index, segment in enumerate(planner.patrol_route, start=1):
        print(
            f"  {index:<3} {segment.title:<42} {segment.duration:8.1f} "
            f"{segment.vx:9.2f} {segment.wz:10.2f}"
        )
    print(f"  Рабочая высота {planner.takeoff_altitude:.1f} м, скорость {planner.cruise_speed:.1f} м/с, "
          f"галсов {planner.lanes}")
    print()


def run_dry_run(args: argparse.Namespace) -> int:
    """Прогоняет FSM в виртуальном времени без ROS 2 и Gazebo."""
    planner = FlightPlanner(
        takeoff_altitude=args.takeoff_altitude,
        cruise_speed=args.cruise_speed,
        lanes=args.lanes,
        hover_warning_s=args.hover_warning,
        hover_critical_s=args.hover_critical,
        battery_drain_pct_per_s=args.battery_drain,
    )
    _print_route(planner)

    simulated = Detection(
        hazard_type=args.hazard,
        threat_level=(args.threat or classify_threat(args.area)).upper(),
        area_sq_m=args.area,
        target_geo={"rel_x": 12.4, "rel_y": -3.8},
        confidence=0.94,
    )

    dt = 1.0 / args.control_rate
    detection_pending = args.detect_at is not None
    last_second = -1
    protocol_saved = False

    print("=== ПРОГОН МИССИИ (виртуальное время) ===")
    while not planner.is_finished and planner.elapsed < args.duration:
        detection = None
        if detection_pending and planner.elapsed >= args.detect_at:
            detection = simulated
            detection_pending = False
            print(
                f"[{planner.elapsed:6.1f} с] ЗРЕНИЕ: событие /mchs/detection_event "
                f"{simulated.hazard_type}/{simulated.threat_level}, {simulated.area_sq_m:.1f} м²"
            )

        command, events = planner.tick(dt, detection=detection)

        for event in events:
            print(f"[{planner.elapsed:6.1f} с] {EVENT_TEXT.get(event, event)}")
            if event == "inspection_started" and not args.no_report and not protocol_saved:
                _save_protocol(planner, args)
                protocol_saved = True

        current_second = int(planner.elapsed)
        if current_second != last_second:
            last_second = current_second
            segment = planner.current_segment
            title = segment.title if segment else "—"
            print(
                f"[{planner.elapsed:6.1f} с] {planner.phase.label:<10} | {title:<42} | "
                f"{command} | Z={planner.z:4.1f} м | АКБ={planner.battery_pct:5.1f}%"
            )

    print()
    if planner.is_finished:
        print(">>> Миссия выполнена: TAKEOFF -> PATROL_GRID -> [HOVER_INSPECTION] -> RTL -> LAND.")
    else:
        print(f"[!] Прогон остановлен по лимиту {args.duration:.0f} с — миссия не завершена.")

    summary = planner.stats()
    print()
    print("=== СВОДКА ПОЛЁТА ===")
    print(f"  Время в воздухе   : {summary['elapsed_s']:.1f} с")
    print(f"  Пройдено          : {summary['distance_flown_m']:.1f} м")
    print(f"  Удаление от базы  : {summary['distance_to_home_m']:.2f} м")
    print(f"  Инспекций очага   : {summary['inspections']}")
    print(f"  Остаток АКБ       : {summary['battery_percent']:.1f} %")
    print(f"  Протокол          : {'сформирован' if protocol_saved else 'не формировался'}")
    print()
    return 0


def _resolve_target_geo(detection: Optional[Detection], x: float, y: float) -> dict:
    """Координаты очага: абсолютные из события, иначе позиция дрона + rel-смещение."""
    geo = dict(detection.target_geo) if detection else {}
    if "abs_x" not in geo or "abs_y" not in geo:
        geo["abs_x"] = x + _to_float(geo.get("rel_x", 0.0))
        geo["abs_y"] = y + _to_float(geo.get("rel_y", 0.0))
    return geo


def _save_protocol(planner: FlightPlanner, args: argparse.Namespace) -> None:
    """Формирует протокол по результатам виртуальной инспекции."""
    detection = planner.active_detection
    summary = planner.stats()
    summary["phases"] = [entry["phase"] for entry in planner.phase_log]
    generate_incident_protocol(
        hazard_code=detection.hazard_type if detection else args.hazard,
        threat_level=detection.threat_level if detection else args.threat,
        area_sq_m=detection.area_sq_m if detection else args.area,
        target_geo=_resolve_target_geo(detection, planner.x, planner.y),
        confidence=detection.confidence if detection else 0.0,
        incident_id=detection.incident_id if detection else None,
        hazard_label_ru=detection.label_ru if detection else None,
        track_id=detection.track_id if detection else "",
        distance_m=(detection.target_geo.get("distance_m") if detection else None),
        bearing_deg=(detection.target_geo.get("bearing_deg") if detection else None),
        drone_telemetry={
            "altitude_m": planner.z,
            "battery_percent": planner.battery_pct,
            "grid_x_m": planner.x,
            "grid_y_m": planner.y,
        },
        flight_summary=summary,
        drone_id=args.drone_id,
        out_dir=args.report_dir or DEFAULT_OUTPUT_DIR,
    )


# ---------------------------------------------------------------------------
# ROS 2
# ---------------------------------------------------------------------------


def run_ros(args: argparse.Namespace) -> int:
    """Запускает ноду ROS 2 и крутит FSM в реальном времени."""
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as error:
        print(f"[ОШИБКА] ROS 2 не найден: {error}")
        print("          Выполните: source /opt/ros/humble/setup.bash")
        print("          Либо запустите без ROS: --dry-run")
        return 1

    class MchsPilotNode(Node):
        """Обёртка: превращает такты FSM в топики ROS 2."""

        def __init__(self) -> None:
            super().__init__("mchs_drone_pilot")

            self.declare_parameter("takeoff_altitude", float(args.takeoff_altitude))
            self.declare_parameter("cruise_speed", float(args.cruise_speed))
            self.declare_parameter("lanes", int(args.lanes))
            self.declare_parameter("hover_warning_s", float(args.hover_warning))
            self.declare_parameter("hover_critical_s", float(args.hover_critical))
            self.declare_parameter("control_rate_hz", float(args.control_rate))
            self.declare_parameter("report_dir", args.report_dir or "")

            self.planner = FlightPlanner(
                takeoff_altitude=self._param("takeoff_altitude"),
                cruise_speed=self._param("cruise_speed"),
                lanes=int(self._param("lanes")),
                hover_warning_s=self._param("hover_warning_s"),
                hover_critical_s=self._param("hover_critical_s"),
            )
            self.report_dir = self._param("report_dir") or DEFAULT_OUTPUT_DIR
            self.drone_id = args.drone_id

            self.publisher = self.create_publisher(Twist, args.cmd_vel_topic, 10)
            self.status_publisher = self.create_publisher(String, args.status_topic, 10)
            self.create_subscription(String, args.detection_topic, self.on_detection, 10)
            self.create_subscription(String, args.command_topic, self.on_command, 10)
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)

            self.last_tick_time: Optional[float] = None
            self.last_second_logged = -1
            self.odom_altitude: Optional[float] = None
            self.odom_position: Optional[tuple[float, float]] = None
            self.pending_detection: Optional[Detection] = None
            self.protocol: Optional[dict] = None

            rate = max(1.0, float(self._param("control_rate_hz")))
            self.create_timer(1.0 / rate, self.on_tick)
            self.create_timer(1.0, self.publish_status)

            self.get_logger().info(">>> Автопилот МЧС запущен: TAKEOFF -> PATROL_GRID -> HOVER_INSPECTION -> RTL -> LAND")
            self.get_logger().info(
                f"    высота {self.planner.takeoff_altitude:.1f} м, скорость {self.planner.cruise_speed:.1f} м/с, "
                f"топик тревог {args.detection_topic}"
            )

        def _param(self, name: str):
            """Значение параметра ROS по имени."""
            return self.get_parameter(name).value

        def _now(self) -> float:
            """Время в секундах (учитывает use_sim_time)."""
            return self.get_clock().now().nanoseconds / 1e9

        # -- подписки ------------------------------------------------------

        def on_detection(self, msg: String) -> None:
            """Принимает событие ЧС от модуля зрения и ставит его в очередь на такт."""
            detection = Detection.from_payload(msg.data)
            if detection is None:
                self.get_logger().warn(f"Непонятное событие ЧС: {msg.data!r}")
                return
            self.pending_detection = detection
            incident = f", инцидент {detection.incident_id}" if detection.incident_id else ""
            self.get_logger().warn(
                f"!!! {detection.hazard_type}/{detection.threat_level} "
                f"({detection.area_sq_m:.1f} м², уверенность {detection.confidence:.2f}{incident})"
            )

        def on_odom(self, msg: Odometry) -> None:
            """Запоминает фактическую высоту и позицию из Gazebo."""
            position = msg.pose.pose.position
            self.odom_altitude = float(position.z)
            self.odom_position = (float(position.x), float(position.y))

        def on_command(self, msg: String) -> None:
            """Команды оператора: RTL (возврат) и LAND (посадка на месте)."""
            command = msg.data.strip().upper()
            if command in ("RTL", "RETURN", "ABORT"):
                if self.planner.request_rtl():
                    self.get_logger().warn("!!! Команда оператора: возврат на базу (RTL).")
                else:
                    self.get_logger().info("Команда RTL не требуется: дрон уже садится/сел.")
            elif command in ("LAND", "STOP"):
                if self.planner.request_land():
                    self.get_logger().warn("!!! Команда оператора: посадка на месте.")
                else:
                    self.get_logger().info("Команда LAND не требуется: миссия уже завершена.")
            else:
                self.get_logger().info(f"Команда {command!r} не поддерживается (есть RTL и LAND).")

        # -- такт управления -----------------------------------------------

        def on_tick(self) -> None:
            """Продвигает FSM и публикует уставку скорости."""
            now = self._now()
            if self.last_tick_time is None:
                self.last_tick_time = now
                return
            dt = min(max(now - self.last_tick_time, 0.0), 1.0)
            self.last_tick_time = now

            detection, self.pending_detection = self.pending_detection, None
            command, events = self.planner.tick(dt, detection=detection)
            self._publish_command(command)

            for event in events:
                self._handle_event(event)

            second = int(self.planner.elapsed)
            if second != self.last_second_logged:
                self.last_second_logged = second
                self._log_telemetry()

        def _publish_command(self, command: Command) -> None:
            """Публикует уставку в /mchs/cmd_vel."""
            message = Twist()
            message.linear.x = float(command.vx)
            message.linear.y = float(command.vy)
            message.linear.z = float(command.vz)
            message.angular.z = float(command.wz)
            self.publisher.publish(message)

        def _handle_event(self, event: str) -> None:
            """Логирует события FSM и формирует протокол при инспекции."""
            text = EVENT_TEXT.get(event)
            if text:
                self.get_logger().info(text)
            if event == "inspection_started":
                self._build_protocol()

        # -- протокол ------------------------------------------------------

        def _build_protocol(self) -> None:
            """Формирует протокол ЦУКС в момент зависания над очагом."""
            detection = self.planner.active_detection
            summary = self.planner.stats()
            summary["phases"] = [entry["phase"] for entry in self.planner.phase_log]

            self.protocol = generate_incident_protocol(
                hazard_code=detection.hazard_type if detection else "FIRE",
                threat_level=detection.threat_level if detection else classify_threat(0.0),
                area_sq_m=detection.area_sq_m if detection else 0.0,
                target_geo=_resolve_target_geo(detection, self.planner.x, self.planner.y),
                confidence=detection.confidence if detection else 0.0,
                incident_id=detection.incident_id if detection else None,
                hazard_label_ru=detection.label_ru if detection else None,
                track_id=detection.track_id if detection else "",
                distance_m=(detection.target_geo.get("distance_m") if detection else None),
                bearing_deg=(detection.target_geo.get("bearing_deg") if detection else None),
                drone_telemetry={
                    "altitude_m": self._altitude(),
                    "battery_percent": self.planner.battery_pct,
                    "grid_x_m": self.planner.x,
                    "grid_y_m": self.planner.y,
                },
                flight_summary=summary,
                drone_id=self.drone_id,
                out_dir=self.report_dir,
            )
            archive = self.protocol.get("archive") or {}
            self.get_logger().info(f">>> Протокол ЦУКС (JSON): {archive.get('json')}")
            self.get_logger().info(f">>> Бланк дежурного (TXT): {archive.get('txt')}")

        # -- телеметрия ----------------------------------------------------

        def _altitude(self) -> float:
            """Высота: из одометрии, иначе из модели."""
            return self.odom_altitude if self.odom_altitude is not None else self.planner.z

        def _log_telemetry(self) -> None:
            """Пишет строку телеметрии раз в секунду."""
            segment = self.planner.current_segment
            title = segment.title if segment else "—"
            self.get_logger().info(
                f"[{self.planner.elapsed:5.1f} с] {self.planner.phase.label:<10} | {title:<40} | "
                f"Z={self._altitude():4.1f} м | АКБ={self.planner.battery_pct:5.1f}%"
            )

        def publish_status(self) -> None:
            """Публикует JSON-статус миссии для АРМ оператора."""
            segment = self.planner.current_segment
            archive = (self.protocol or {}).get("archive") or {}
            status = {
                "phase": self.planner.phase.value,
                "phase_label": self.planner.phase.label,
                "segment": segment.title if segment else None,
                "elapsed_s": round(self.planner.elapsed, 1),
                "altitude_m": round(self._altitude(), 1),
                "battery_percent": round(self.planner.battery_pct, 1),
                "inspections": self.planner.inspections,
                "distance_to_home_m": round(math.hypot(self.planner.x, self.planner.y), 2),
                "protocol_json": archive.get("json"),
            }
            message = String()
            message.data = json.dumps(status, ensure_ascii=False)
            self.status_publisher.publish(message)

    rclpy.init()
    node = MchsPilotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Остановка оператором. Гашу двигатели.")
        node._publish_command(Command())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


# ---------------------------------------------------------------------------
# Командная строка
# ---------------------------------------------------------------------------


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Разбирает аргументы командной строки."""
    defaults = FlightPlanner()
    parser = argparse.ArgumentParser(
        description="Автономный автопилот «Око МЧС»: FSM патрулирования, инспекция очага, RTL.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Примеры:\n"
            "  python3 mchs_pilot_node.py --dry-run\n"
            "  python3 mchs_pilot_node.py --dry-run --detect-at 8 --hazard FLOOD --area 120\n"
            "  source /opt/ros/humble/setup.bash && python3 mchs_pilot_node.py\n"
        ),
    )
    parser.add_argument("--dry-run", action="store_true", help="прогон без ROS 2 (виртуальное время)")
    parser.add_argument("--takeoff-altitude", type=float, default=defaults.takeoff_altitude, help="рабочая высота, м")
    parser.add_argument("--cruise-speed", type=float, default=defaults.cruise_speed, help="скорость патруля, м/с")
    parser.add_argument("--lanes", type=int, default=defaults.lanes, help="количество галсов «змейки»")
    parser.add_argument("--hover-warning", type=float, default=defaults.hover_warning_s, help="зависание при WARNING, с")
    parser.add_argument("--hover-critical", type=float, default=defaults.hover_critical_s, help="зависание при CRITICAL, с")
    parser.add_argument("--control-rate", type=float, default=10.0, help="частота управления, Гц")
    parser.add_argument("--detect-at", type=float, default=6.0, help="секунда имитации тревоги в --dry-run")
    parser.add_argument("--hazard", default="FIRE", help="код ЧС для --dry-run (FIRE/SMOKE/OIL_SPILL/FLOOD)")
    parser.add_argument("--threat", default=None, help="уровень угрозы для --dry-run (по умолчанию — по площади)")
    parser.add_argument("--area", type=float, default=64.2, help="площадь очага для --dry-run, м²")
    parser.add_argument("--duration", type=float, default=300.0, help="лимит времени прогона, с")
    parser.add_argument("--battery-drain", type=float, default=defaults.battery_drain_pct_per_s, help="расход АКБ, %% в секунду")
    parser.add_argument("--no-report", action="store_true", help="не формировать протокол в --dry-run")
    parser.add_argument("--report-dir", default=None, help="папка для протоколов (по умолчанию reports/)")
    parser.add_argument("--drone-id", default="MCHS-UAV-04", help="бортовой номер БПЛА")
    parser.add_argument("--detection-topic", default="/mchs/detection_event", help="топик событий ЧС")
    parser.add_argument("--cmd-vel-topic", default="/mchs/cmd_vel", help="топик уставок скорости")
    parser.add_argument("--odom-topic", default="/mchs/odom", help="топик одометрии")
    parser.add_argument("--status-topic", default="/mchs/mission_status", help="топик статуса миссии")
    parser.add_argument("--command-topic", default="/mchs/mission_command", help="топик команд оператора")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Точка входа: выбирает режим работы (ROS 2 или виртуальный прогон)."""
    args = parse_args(argv)
    if args.dry_run:
        return run_dry_run(args)
    return run_ros(args)


if __name__ == "__main__":
    sys.exit(main())
