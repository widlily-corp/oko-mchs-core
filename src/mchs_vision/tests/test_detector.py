"""Тесты детектора на синтетических сценах и трекинга."""

import numpy as np
import pytest

from mchs_vision.config import DetectorConfig
from mchs_vision.detector import HazardDetector
from mchs_vision.simulated_scene import SCENES, SyntheticArena


@pytest.fixture(scope="module")
def arena():
    scene = SyntheticArena(seed=7)
    scene.render()
    return scene


def detect_scene(arena, name: str, min_hits: int = 1):
    frame = arena.crop(SCENES[name].center_px)
    detector = HazardDetector(DetectorConfig(track_min_hits=min_hits))
    return frame, detector.detect(frame)


def test_fire_detected_with_large_area(arena):
    _, detections = detect_scene(arena, "fire_critical")
    fires = [det for det in detections if det.hazard_type == "FIRE"]
    assert len(fires) == 1
    assert fires[0].area_px > 4000
    assert fires[0].confidence > 0.6


def test_oil_spill_detected(arena):
    _, detections = detect_scene(arena, "oil_warning")
    types = {det.hazard_type for det in detections}
    assert "OIL_SPILL" in types


def test_flood_detected(arena):
    _, detections = detect_scene(arena, "flood_critical")
    floods = [det for det in detections if det.hazard_type == "FLOOD"]
    assert len(floods) == 1
    assert floods[0].area_px > 10000


def test_smoke_detected(arena):
    _, detections = detect_scene(arena, "smoke_only")
    types = {det.hazard_type for det in detections}
    assert "SMOKE" in types


def test_multi_hazard_scene(arena):
    _, detections = detect_scene(arena, "multi")
    types = {det.hazard_type for det in detections}
    assert "FIRE" in types
    assert "OIL_SPILL" in types


def test_clean_forest_has_no_false_positives(arena):
    _, detections = detect_scene(arena, "clean")
    assert detections == []


def test_tracker_requires_consecutive_hits(arena):
    frame = arena.crop(SCENES["fire_critical"].center_px)
    detector = HazardDetector(DetectorConfig(track_min_hits=3))

    assert detector.detect(frame) == []
    assert detector.detect(frame) == []
    confirmed = detector.detect(frame)
    assert len(confirmed) == 1
    assert confirmed[0].hits >= 3
    assert confirmed[0].track_id.startswith("TRK-FIRE-")


def test_tracker_forgets_stale_tracks(arena):
    frame = arena.crop(SCENES["fire_critical"].center_px)
    detector = HazardDetector(DetectorConfig(track_min_hits=1,
                                             track_max_missed=1))
    assert len(detector.detect(frame)) == 1
    blank = np.full_like(frame, (40, 90, 45))

    detector.detect(blank)
    detector.detect(blank)
    detector.detect(blank)
    assert detector.active_tracks == {}
