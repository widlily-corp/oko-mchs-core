"""Сквозные тесты пайплайна: запись, событие, HUD, сброс трекинга."""

import json
from datetime import datetime, timedelta, timezone

import numpy as np
import pytest

from mchs_vision.config import CameraConfig, DetectorConfig
from mchs_vision.events import EventThrottle
from mchs_vision.pipeline import VisionPipeline
from mchs_vision.simulated_scene import SCENES, SyntheticArena


@pytest.fixture(scope="module")
def arena():
    scene = SyntheticArena(seed=7)
    scene.render()
    return scene


def make_pipeline(min_hits=1, throttle=None):
    camera = CameraConfig(width=640, height=480, hfov_deg=80.0,
                          pitch_down_deg=90.0)
    return VisionPipeline(
        camera=camera,
        detector_config=DetectorConfig(track_min_hits=min_hits),
        throttle=throttle,
    )


def test_pipeline_builds_record_and_event(arena):
    pipeline = make_pipeline()
    center = SCENES["fire_critical"].center_px
    frame = arena.crop(center)
    pose = arena.pose_for(center)
    result = pipeline.process(frame, pose)

    assert len(result.records) == 1
    record = result.records[0]
    ground_truth = arena.ground_truth(center)[0]
    assert record.hazard_type == "FIRE"
    assert record.smoothed_area_m2 == pytest.approx(
        ground_truth["area_m2"], rel=0.15
    )
    assert record.threat_level == "CRITICAL"
    assert record.abs_xy[0] == pytest.approx(42.0, abs=2.0)
    assert record.abs_xy[1] == pytest.approx(98.0, abs=2.0)

    assert len(result.events) == 1
    event = result.events[0]
    assert event["hazard_type"] == "FIRE"
    assert event["incident_id"].startswith("INC-")
    assert event["target_geo"]["lat"] != 0.0
    json.dumps(event, ensure_ascii=False)  # событие сериализуемо


def test_pipeline_hud_shape(arena):
    pipeline = make_pipeline()
    center = SCENES["flood_critical"].center_px
    frame = arena.crop(center)
    result = pipeline.process(frame, arena.pose_for(center), draw=True)
    assert result.annotated.shape == frame.shape
    assert result.annotated.dtype == np.uint8
    assert not np.array_equal(result.annotated, frame)


def test_pipeline_reset_avoids_cross_scene_tracking(arena):
    pipeline = make_pipeline()
    first_center = SCENES["fire_critical"].center_px
    second_center = SCENES["fire_advisory"].center_px

    pipeline.process(arena.crop(first_center), arena.pose_for(first_center))
    pipeline.reset_tracking()
    result = pipeline.process(arena.crop(second_center),
                              arena.pose_for(second_center))

    assert len(result.records) == 1
    assert result.records[0].smoothed_area_m2 < 15.0
    assert result.records[0].threat_level == "ADVISORY"


def test_pipeline_event_throttle(arena):
    throttle = EventThrottle(cooldown_s=3.0, min_interval_s=1.0)
    pipeline = make_pipeline(throttle=throttle)
    center = SCENES["fire_critical"].center_px
    frame = arena.crop(center)
    pose = arena.pose_for(center)
    start = datetime.now(timezone.utc)

    first = pipeline.process(frame, pose, start)
    second = pipeline.process(frame, pose, start + timedelta(seconds=0.2))
    third = pipeline.process(frame, pose, start + timedelta(seconds=3.5))

    assert len(first.events) == 1
    assert len(second.events) == 0
    assert len(third.events) == 1


def test_event_ids_have_no_gaps_when_throttled(arena):
    """EventFactory нумерует только реально опубликованные события."""
    throttle = EventThrottle(cooldown_s=3.0, min_interval_s=1.0)
    pipeline = make_pipeline(throttle=throttle)
    center = SCENES["fire_critical"].center_px
    frame = arena.crop(center)
    pose = arena.pose_for(center)
    start = datetime.now(timezone.utc)

    first = pipeline.process(frame, pose, start)
    pipeline.process(frame, pose, start + timedelta(seconds=0.2))
    third = pipeline.process(frame, pose, start + timedelta(seconds=3.5))

    assert first.events[0]["event_id"].endswith("000001")
    assert third.events[0]["event_id"].endswith("000002")


def test_pipeline_returns_empty_on_clean_scene(arena):
    pipeline = make_pipeline()
    center = SCENES["clean"].center_px
    result = pipeline.process(arena.crop(center), arena.pose_for(center))
    assert result.records == []
    assert result.events == []
    assert result.primary is None
