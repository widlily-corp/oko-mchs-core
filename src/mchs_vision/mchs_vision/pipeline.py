"""Сквозной пайплайн бортового зрения: кадр + поза → события и HUD."""

from __future__ import annotations

import time
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional

import numpy as np

from .config import (CameraConfig, DetectorConfig, GeoOrigin, threat_level)
from .detector import Detection, HazardDetector
from .events import (EventFactory, EventThrottle, HazardRecord, IncidentRegistry,
                     sort_records)
from .geometry import GroundProjector, Pose
from .hud import TacticalHUD


@dataclass
class PipelineResult:
    """Результат обработки одного кадра."""

    records: List[HazardRecord]
    annotated: np.ndarray
    events: List[dict]
    fps: float
    primary: Optional[HazardRecord]


@dataclass
class _SmoothState:
    x: float
    y: float
    area: float
    updated_at: float


class VisionPipeline:
    """Связывает детектор, геометрию, события и HUD в один вызов."""

    def __init__(self, camera: Optional[CameraConfig] = None,
                 detector_config: Optional[DetectorConfig] = None,
                 origin: Optional[GeoOrigin] = None,
                 title: str = "MCHS OPTICAL RECON // GAZEBO 11",
                 throttle: Optional[EventThrottle] = None):
        self.camera = camera or CameraConfig()
        self.config = detector_config or DetectorConfig()
        self.origin = origin or GeoOrigin()
        self.title = title

        self.detector = HazardDetector(self.config)
        self.projector = GroundProjector(self.camera, self.origin)
        self.hud = TacticalHUD(self.projector, self.config.classes)
        self.registry = IncidentRegistry()
        self.events = EventFactory()
        self.throttle = throttle or EventThrottle()

        self._smooth: Dict[str, _SmoothState] = {}
        self._fps = 0.0
        self._last_frame_time: Optional[float] = None
        self._frames = 0

    def process(self, frame_bgr: np.ndarray, pose: Pose,
                timestamp: Optional[datetime] = None,
                draw: bool = True) -> PipelineResult:
        """Обработать кадр и вернуть записи, HUD и готовые JSON-события."""
        now = timestamp or datetime.now().astimezone()
        detections = self.detector.detect(frame_bgr)

        records: List[HazardRecord] = []
        for detection in detections:
            record = self._make_record(detection, pose, now)
            if record is not None:
                records.append(record)
        records = sort_records(records)

        events: List[dict] = []
        for record in records:
            incident_id = self.registry.incident_for(
                record.hazard_type, record.abs_xy, now
            )
            event = self.events.build(record, records, pose, incident_id, now)
            event["frame"] = {
                "width": int(frame_bgr.shape[1]),
                "height": int(frame_bgr.shape[0]),
            }
            if self.throttle.allow(event, now):
                events.append(event)

        self.registry.prune(now)
        self._prune_smooth()

        if draw:
            annotated = self.hud.draw(
                frame_bgr, records, pose, self._fps, now.strftime("%H:%M:%S"),
                title=self.title,
            )
        else:
            annotated = frame_bgr

        self._update_fps()
        self._frames += 1

        return PipelineResult(
            records=records,
            annotated=annotated,
            events=events,
            fps=self._fps,
            primary=records[0] if records else None,
        )

    def _make_record(self, detection: Detection, pose: Pose,
                     now: datetime) -> Optional[HazardRecord]:
        metrics = self.projector.region_metrics(detection.contour, pose)
        if metrics is None:
            return None

        key = detection.track_id or f"{detection.hazard_type}-{id(detection)}"
        alpha = self.config.smoothing_alpha
        previous = self._smooth.get(key)
        if previous is None:
            smooth_x, smooth_y = metrics.center_xy
            smooth_area = metrics.area_m2
        else:
            smooth_x = alpha * metrics.center_xy[0] + (1 - alpha) * previous.x
            smooth_y = alpha * metrics.center_xy[1] + (1 - alpha) * previous.y
            smooth_area = alpha * metrics.area_m2 + (1 - alpha) * previous.area

        self._smooth[key] = _SmoothState(
            x=smooth_x, y=smooth_y, area=smooth_area, updated_at=time.monotonic()
        )

        rel_x = smooth_x - pose.x_m
        rel_y = smooth_y - pose.y_m
        lat, lon = self.projector.local_to_latlon(smooth_x, smooth_y)
        level = threat_level(
            smooth_area,
            warning_m2=self.config.threat_warning_m2,
            critical_m2=self.config.threat_critical_m2,
        )

        return HazardRecord(
            detection=detection,
            metrics=metrics,
            threat_level=level,
            abs_xy=(smooth_x, smooth_y),
            rel_xy=(rel_x, rel_y),
            lat=lat,
            lon=lon,
            smoothed_area_m2=smooth_area,
        )

    def _update_fps(self) -> None:
        current = time.monotonic()
        if self._last_frame_time is not None:
            delta = max(1e-6, current - self._last_frame_time)
            instant = 1.0 / delta
            self._fps = instant if self._fps <= 0 else 0.9 * self._fps + 0.1 * instant
        self._last_frame_time = current

    def _prune_smooth(self, max_age_s: float = 10.0) -> None:
        if len(self._smooth) < 64:
            return
        current = time.monotonic()
        self._smooth = {
            key: state for key, state in self._smooth.items()
            if current - state.updated_at <= max_age_s
        }

    def reset_tracking(self) -> None:
        """Сбросить треки и сглаживание (для независимых изображений).

        Реестр инцидентов и нумерация событий сохраняются, чтобы JSON
        оставался сквозным по всему демонстрационному прогону.
        """
        self.detector.reset()
        self._smooth.clear()
        self._fps = 0.0
        self._last_frame_time = None
