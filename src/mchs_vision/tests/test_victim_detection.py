"""Тесты модуля обнаружения пострадавшего (VICTIM/HUMAN), фотофиксации и триангуляции."""

from __future__ import annotations

import json
import math
import os
import tempfile
import cv2
import numpy as np
import pytest

from mchs_vision.config import CameraConfig, DetectorConfig, GeoOrigin, HazardClassSpec, threat_level
from mchs_vision.detector import Detection, HazardDetector
from mchs_vision.events import EventFactory, IncidentRegistry, save_victim_evidence_snapshot
from mchs_vision.geometry import GroundProjector, Pose
from mchs_vision.pipeline import VisionPipeline


@pytest.fixture
def camera() -> CameraConfig:
    return CameraConfig(
        width=1280,
        height=720,
        hfov_deg=80.0,
        pitch_down_deg=90.0,
    )


def create_synthetic_victim_frame(width=1280, height=720, vest_x=640, vest_y=360) -> np.ndarray:
    """Создаёт тестовый кадр: трава и фигура человека в оранжевом спасательном жилете (#FF5722) с синей светоотражающей полосой."""
    frame = np.full((height, width, 3), (35, 60, 25), dtype=np.uint8)

    # Тело человека (высота 40 px, ширина 18 px)
    # 1. Тёмные ноги/брюки
    cv2.rectangle(frame, (vest_x - 7, vest_y + 8), (vest_x + 7, vest_y + 24), (25, 25, 30), -1)
    # 2. Оранжевый сигнальный жилет (#FF5722 -> BGR: 34, 87, 255)
    cv2.rectangle(frame, (vest_x - 9, vest_y - 12), (vest_x + 9, vest_y + 8), (34, 87, 255), -1)
    # 3. Светоотражающая полоса (Cyan / High-vis BGR: 255, 220, 0)
    cv2.rectangle(frame, (vest_x - 8, vest_y - 4), (vest_x + 8, vest_y - 1), (255, 220, 0), -1)
    # 4. Голова / жёлтая каска (BGR: 40, 210, 240)
    cv2.circle(frame, (vest_x, vest_y - 16), 5, (40, 210, 240), -1)

    return frame


def test_victim_spec_in_detector_config():
    """Проверяет, что класс VICTIM зарегистрирован в конфигурации детектора."""
    cfg = DetectorConfig()
    assert "VICTIM" in cfg.specs, "Класс VICTIM не найден в DetectorConfig.specs"
    spec = cfg.specs["VICTIM"]
    assert isinstance(spec, HazardClassSpec)
    assert threat_level(spec.name) == "CRITICAL"
    assert "человек" in spec.label_ru.lower() or "пострадавший" in spec.label_ru.lower()


def test_detector_finds_victim_in_frame(camera):
    """Проверяет обнаружение фигуры человека детектором на синтетическом кадре."""
    cfg = DetectorConfig(enabled=("FIRE", "SMOKE", "OIL_SPILL", "FLOOD", "VICTIM"), track_min_hits=1)
    detector = HazardDetector(cfg)
    frame = create_synthetic_victim_frame(vest_x=500, vest_y=300)

    detections = detector.detect(frame)
    victim_dets = [d for d in detections if d.hazard_type == "VICTIM"]

    assert len(victim_dets) >= 1, "Детектор не обнаружил фигуру человека в жилете"
    det = victim_dets[0]
    assert det.confirmed is True
    assert det.confidence >= 0.60
    assert abs(det.centroid_px[0] - 500) < 15
    assert abs(det.centroid_px[1] - 300) < 15


def test_victim_triangulation_and_event_payload(camera):
    """Проверяет привязку координат человека и генерацию события ЧС."""
    projector = GroundProjector(camera=camera, origin=GeoOrigin(lat=53.4129, lon=58.9814))
    registry = IncidentRegistry()
    factory = EventFactory(projector=projector, registry=registry)

    pose = Pose(x_m=20.0, y_m=-15.0, z_m=22.0, yaw_rad=0.0)
    frame = create_synthetic_victim_frame(vest_x=640, vest_y=360)

    cfg = DetectorConfig(enabled=("VICTIM",), track_min_hits=1)
    detector = HazardDetector(cfg)
    detections = detector.detect(frame)

    events = factory.build_events(detections, pose)
    assert len(events) >= 1

    ev = events[0]
    assert ev["hazard_type"] == "VICTIM"
    assert ev["threat_level"] == "CRITICAL"
    assert "target_geo" in ev
    geo = ev["target_geo"]
    assert "lat" in geo and "lon" in geo
    assert "abs_x" in geo and "abs_y" in geo

    # В центре надирного кадра координаты должны совпасть с положением БПЛА
    assert abs(geo["abs_x"] - 20.0) < 1.0
    assert abs(geo["abs_y"] - (-15.0)) < 1.0


def test_victim_photo_snapshot_saving(camera):
    """Проверяет сохранение тактического фотоснимка с разметкой для пострадавшего."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        incidents_dir = os.path.join(tmp_dir, "incidents")
        os.makedirs(incidents_dir, exist_ok=True)

        frame = create_synthetic_victim_frame(vest_x=640, vest_y=360)
        pose = Pose(x_m=12.5, y_m=8.4, z_m=22.0)

        photo_rel_path = save_victim_evidence_snapshot(
            frame=frame,
            centroid_px=(640, 360),
            bbox=(630, 340, 20, 40),
            pose=pose,
            output_dir=incidents_dir,
            incident_id="INC-VICTIM-001",
        )

        assert photo_rel_path is not None
        full_path = os.path.join(incidents_dir, os.path.basename(photo_rel_path))
        assert os.path.isfile(full_path), f"Файл фотоснимка не найден: {full_path}"

        saved_img = cv2.imread(full_path)
        assert saved_img is not None
        assert saved_img.shape == frame.shape


def test_pipeline_victim_event_is_always_critical(camera):
    """Человек — CRITICAL независимо от видимой площади (не ADVISORY)."""
    frame = create_synthetic_victim_frame(vest_x=640, vest_y=360)
    pipeline = VisionPipeline(
        camera=camera,
        detector_config=DetectorConfig(enabled=("VICTIM",), track_min_hits=1),
    )
    result = pipeline.process(frame, Pose(x_m=20.0, y_m=-15.0, z_m=22.0),
                              draw=False)
    assert result.primary is not None
    assert result.primary.hazard_type == "VICTIM"
    assert result.primary.threat_level == "CRITICAL"
    assert result.events[0]["threat_level"] == "CRITICAL"


def test_victim_clean_gazebo_frames_are_rejected():
    """Оранжевые контейнеры/крыши не должны становиться пострадавшими."""
    data = os.path.abspath(os.path.join(os.path.dirname(__file__), "data"))
    camera = CameraConfig(
        width=1280, height=720, hfov_deg=80.0, pitch_down_deg=45.0,
        mount_forward_m=0.42, mount_up_m=0.12,
    )
    for name in (
        "gazebo_containers_1280.png",
        "gazebo_helipad_1280.png",
        "gazebo_home_1280.png",
    ):
        frame = cv2.imread(os.path.join(data, name))
        assert frame is not None, name
        pipeline = VisionPipeline(
            camera=camera,
            detector_config=DetectorConfig(enabled=("VICTIM",),
                                            track_min_hits=1),
        )
        result = pipeline.process(frame, Pose(x_m=0.0, y_m=0.0, z_m=22.0),
                                  draw=False)
        assert result.records == [], name


def test_victim_rejects_wide_orange_strip(camera):
    """Длинная оранжевая полоса (борт техники) — не человек."""
    frame = np.full((720, 1280, 3), (35, 60, 25), dtype=np.uint8)
    cv2.rectangle(frame, (500, 350), (760, 368), (34, 87, 255), -1)
    pipeline = VisionPipeline(
        camera=camera,
        detector_config=DetectorConfig(enabled=("VICTIM",), track_min_hits=1),
    )
    result = pipeline.process(frame, Pose(x_m=0.0, y_m=0.0, z_m=22.0),
                              draw=False)
    assert result.records == []
