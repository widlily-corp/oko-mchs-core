"""Формирование машиночитаемого события ЧС и реестр инцидентов.

Контракт совместим с ТЗ команды: в ``/mchs/detection_event`` уходит JSON
со полями ``hazard_type``, ``threat_level``, ``area_sq_m``, ``target_geo``,
``confidence``. Дополнительно передаются ``incident_id``, ``track_id``,
координаты WGS-84 и массив ``detections`` со всеми найденными очагами.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from .detector import Detection
from .geometry import Pose, RegionMetrics

THREAT_RANK = {"ADVISORY": 1, "WARNING": 2, "CRITICAL": 3}


@dataclass
class HazardRecord:
    """Обнаруженный очаг, уже привязанный к местности."""

    detection: Detection
    metrics: RegionMetrics
    threat_level: str
    abs_xy: Tuple[float, float]
    rel_xy: Tuple[float, float]
    lat: float
    lon: float
    smoothed_area_m2: float

    @property
    def hazard_type(self) -> str:
        return self.detection.hazard_type

    @property
    def track_id(self) -> str:
        return self.detection.track_id

    @property
    def confidence(self) -> float:
        return self.detection.confidence


@dataclass
class _Incident:
    incident_id: str
    hazard_type: str
    center_xy: Tuple[float, float]
    created_at: datetime
    updated_at: datetime


class IncidentRegistry:
    """Присваивает стабильные ID инцидентов и склеивает повторные детекции."""

    def __init__(self, max_age_s: float = 300.0, radius_m: float = 15.0,
                 year_prefix: Optional[int] = None):
        self.max_age_s = max_age_s
        self.radius_m = radius_m
        self.year_prefix = year_prefix
        self._incidents: Dict[str, List[_Incident]] = {}
        self._counter = 0

    def incident_for(self, hazard_type: str, xy: Tuple[float, float],
                     timestamp: datetime) -> str:
        candidates = self._incidents.setdefault(hazard_type, [])
        best: Optional[_Incident] = None
        best_distance = float("inf")

        for incident in candidates:
            age = (timestamp - incident.updated_at).total_seconds()
            if age > self.max_age_s:
                continue
            distance = math.hypot(xy[0] - incident.center_xy[0],
                                  xy[1] - incident.center_xy[1])
            if distance <= self.radius_m and distance < best_distance:
                best = incident
                best_distance = distance

        if best is not None:
            best.center_xy = xy
            best.updated_at = timestamp
            return best.incident_id

        self._counter += 1
        year = self.year_prefix or timestamp.year
        incident_id = f"INC-{year}-{self._counter:03d}"
        candidates.append(_Incident(
            incident_id=incident_id,
            hazard_type=hazard_type,
            center_xy=xy,
            created_at=timestamp,
            updated_at=timestamp,
        ))
        return incident_id

    def prune(self, timestamp: datetime) -> None:
        for hazard_type, incidents in list(self._incidents.items()):
            alive = [
                inc for inc in incidents
                if (timestamp - inc.updated_at).total_seconds() <= self.max_age_s
            ]
            if alive:
                self._incidents[hazard_type] = alive
            else:
                del self._incidents[hazard_type]


class EventThrottle:
    """Не даёт публиковать одно и то же событие на каждом кадре.

    Событие проходит, если:
      * инцидент новый;
      * сменился уровень опасности;
      * площадь изменилась больше чем на ``area_change_ratio``;
      * прошло больше ``cooldown_s`` секунд (периодический heartbeat).
    """

    def __init__(self, cooldown_s: float = 2.0, min_interval_s: float = 1.0,
                 area_change_ratio: float = 0.25):
        self.cooldown_s = cooldown_s
        self.min_interval_s = min_interval_s
        self.area_change_ratio = area_change_ratio
        self._last: Dict[str, Tuple[datetime, float, str]] = {}

    def allow(self, event: Dict, timestamp: datetime) -> bool:
        return self.should_emit(
            str(event.get("incident_id", "")),
            float(event.get("area_sq_m", 0.0)),
            str(event.get("threat_level", "")),
            timestamp,
        )

    def should_emit(self, incident_id: str, area: float, threat: str,
                    timestamp: datetime) -> bool:
        """Решить, нужно ли событие, не собирая JSON заранее.

        Благодаря этому ``EventFactory`` нумерует только реально
        опубликованные события, без «дыр» в event_id.
        """
        previous = self._last.get(incident_id)

        if previous is None:
            self._last[incident_id] = (timestamp, area, threat)
            return True

        last_time, last_area, last_threat = previous
        elapsed = (timestamp - last_time).total_seconds()

        # Жёсткий нижний предел: даже при быстром росте площади не чаще,
        # чем раз в min_interval_s (защита ROS-топика от флуда).
        if elapsed < self.min_interval_s:
            return False

        area_changed = abs(area - last_area) / max(last_area, 1.0) \
            > self.area_change_ratio

        if threat != last_threat or area_changed or elapsed >= self.cooldown_s:
            self._last[incident_id] = (timestamp, area, threat)
            return True
        return False


class EventFactory:
    """Собирает JSON-словарь события из записей пайплайна."""

    def __init__(self):
        self._sequence = 0

    def build(self, primary: HazardRecord, records: List[HazardRecord],
              pose: Pose, incident_id: str, timestamp: datetime) -> Dict:
        self._sequence += 1
        event_id = f"EVT-{timestamp:%Y%m%d}-{self._sequence:06d}"

        return {
            "schema_version": "1.0",
            "event_id": event_id,
            "incident_id": incident_id,
            "timestamp": timestamp.isoformat(timespec="seconds"),
            "hazard_type": primary.hazard_type,
            "hazard_label_ru": primary.detection.label_ru,
            "threat_level": primary.threat_level,
            "area_sq_m": round(primary.smoothed_area_m2, 2),
            "area_px": int(primary.detection.area_px),
            "confidence": round(primary.confidence, 3),
            "track_id": primary.track_id,
            "target_geo": self._target_geo(primary),
            "uav_pose": {
                "x": round(pose.x_m, 2),
                "y": round(pose.y_m, 2),
                "z": round(pose.z_m, 2),
                "yaw_deg": round(pose.yaw_deg, 1),
            },
            "detections": [self._detection_payload(record) for record in records],
        }

    @staticmethod
    def _target_geo(record: HazardRecord) -> Dict:
        return {
            "abs_x": round(record.abs_xy[0], 2),
            "abs_y": round(record.abs_xy[1], 2),
            "rel_x": round(record.rel_xy[0], 2),
            "rel_y": round(record.rel_xy[1], 2),
            "lat": round(record.lat, 7),
            "lon": round(record.lon, 7),
            "distance_m": round(record.metrics.distance_m, 2),
            "bearing_deg": round(record.metrics.bearing_deg, 1),
        }

    @classmethod
    def _detection_payload(cls, record: HazardRecord) -> Dict:
        bbox = record.detection.bbox
        return {
            "hazard_type": record.hazard_type,
            "hazard_label_ru": record.detection.label_ru,
            "threat_level": record.threat_level,
            "area_sq_m": round(record.smoothed_area_m2, 2),
            "area_px": int(record.detection.area_px),
            "confidence": round(record.confidence, 3),
            "track_id": record.track_id,
            "target_geo": cls._target_geo(record),
            "bbox_px": {
                "x": int(bbox[0]),
                "y": int(bbox[1]),
                "width": int(bbox[2]),
                "height": int(bbox[3]),
            },
            "centroid_px": [
                round(record.detection.centroid_px[0], 1),
                round(record.detection.centroid_px[1], 1),
            ],
        }


def sort_records(records: List[HazardRecord]) -> List[HazardRecord]:
    """Сначала CRITICAL, затем WARNING/ADVISORY; внутри — по площади."""
    return sorted(
        records,
        key=lambda rec: (THREAT_RANK.get(rec.threat_level, 0),
                         rec.smoothed_area_m2),
        reverse=True,
    )
