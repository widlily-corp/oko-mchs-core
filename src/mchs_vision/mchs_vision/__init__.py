"""Бортовое компьютерное зрение комплекса «Око МЧС».

Пакет не зависит от ROS и может использоваться:
  * в ROS 2 ноде ``scripts/mchs_vision_node.py``;
  * в офлайн-демонстраторе ``scripts/vision_demo.py``;
  * в юнит-тестах ``tests/``.

Основные точки входа:
    * :class:`mchs_vision.pipeline.VisionPipeline` — сквозной пайплайн;
    * :class:`mchs_vision.detector.HazardDetector` — детектор 4 классов ЧС;
    * :class:`mchs_vision.fire_area.FireAreaEstimator` — расчёт площади пожара по фото;
    * :class:`mchs_vision.geometry.GroundProjector` — пиксели → метры/WGS-84.
"""

from .config import (CameraConfig, DetectorConfig, GeoOrigin, HazardClassSpec,
                     threat_level)
from .detector import Detection, HazardDetector
from .events import (EventFactory, EventThrottle, HazardRecord,
                     IncidentRegistry)
from .fire_area import FireAreaEstimator, FireAreaResult
from .geometry import GroundProjector, Pose, RegionMetrics
from .hud import TacticalHUD
from .pipeline import PipelineResult, VisionPipeline

__all__ = [
    "CameraConfig",
    "Detection",
    "DetectorConfig",
    "EventFactory",
    "EventThrottle",
    "FireAreaEstimator",
    "FireAreaResult",
    "GeoOrigin",
    "GroundProjector",
    "HazardClassSpec",
    "HazardDetector",
    "HazardRecord",
    "IncidentRegistry",
    "PipelineResult",
    "Pose",
    "RegionMetrics",
    "TacticalHUD",
    "VisionPipeline",
    "threat_level",
]
