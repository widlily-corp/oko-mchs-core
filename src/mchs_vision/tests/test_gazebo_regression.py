"""Регрессия на реальных кадрах Gazebo 11 (1280×720, камера largedrone).

Кадры сняты в арене `mchs_emergency_arena.world` с высоты 22 м, камера
наклонена на 45°. Позы записаны в момент съёмки. Тест защищает от возврата
ложных срабатываний на оранжевой разметке вертодрома, белых резервуарах,
серых крышах и реке, а также от потери реального пожара/нефти/затопления.
"""

from pathlib import Path

import cv2
import numpy as np
import pytest

from mchs_vision.config import CameraConfig, DetectorConfig
from mchs_vision.geometry import Pose
from mchs_vision.pipeline import VisionPipeline

DATA = Path(__file__).with_name("data")
CAMERA = CameraConfig(
    width=1280,
    height=720,
    hfov_deg=80.0,
    pitch_down_deg=45.0,
    mount_forward_m=0.42,
    mount_up_m=0.12,
)

# Поза дрона в момент сохранения кадра (odom, метры/градусы).
POSES = {
    "fire": (41.762, 27.841, 22.0),
    "oil": (-75.870, 29.922, 22.0),
    "flood": (27.174, -56.118, 22.0),
    "waste": (49.533, -71.675, 22.0),
    "helipad": (-25.443, -0.185, 22.0),
    "home": (0.253, -0.367, 22.0),
    "containers": (-13.741, -21.630, 22.0),
}


def run_frame(name: str):
    frame = cv2.imread(str(DATA / f"gazebo_{name}_1280.png"),
                       cv2.IMREAD_COLOR)
    assert frame is not None, f"нет тестового кадра {name}"
    x, y, z = POSES[name]
    pipeline = VisionPipeline(
        camera=CAMERA,
        detector_config=DetectorConfig(track_min_hits=1),
        title="GAZEBO REGRESSION",
    )
    return pipeline.process(frame, Pose.from_deg(x, y, z, yaw_deg=0.0),
                            draw=False)


def test_real_gazebo_fire_is_detected():
    result = run_frame("fire")
    types = {record.hazard_type for record in result.records}
    assert types == {"FIRE"}
    fire = result.primary
    assert fire is not None
    assert fire.smoothed_area_m2 >= 10.0
    assert fire.threat_level in {"WARNING", "CRITICAL"}
    assert fire.abs_xy[0] == pytest.approx(60.0, abs=10.0)
    assert fire.abs_xy[1] == pytest.approx(28.0, abs=10.0)


def test_real_gazebo_oil_spill_is_detected_without_smoke_tanks():
    result = run_frame("oil")
    types = {record.hazard_type for record in result.records}
    assert types == {"OIL_SPILL"}
    oil = result.primary
    assert oil is not None
    assert 20.0 <= oil.smoothed_area_m2 <= 70.0
    assert oil.threat_level == "WARNING"
    assert oil.abs_xy[0] == pytest.approx(-58.0, abs=12.0)


def test_real_gazebo_flood_is_detected_without_river_noise():
    result = run_frame("flood")
    types = {record.hazard_type for record in result.records}
    assert types == {"FLOOD"}
    flood = result.primary
    assert flood is not None
    assert flood.smoothed_area_m2 > 100.0
    assert flood.threat_level == "CRITICAL"
    assert flood.abs_xy[0] == pytest.approx(45.0, abs=5.0)
    assert flood.abs_xy[1] == pytest.approx(-56.0, abs=5.0)


def test_real_gazebo_containers_are_not_smoke_or_flood():
    """Синий/белый контейнеры не должны давать дым/затопление.

    В кадре при этом видна настоящая зона затопления вдалеке — она должна
    остаться единственным событием.
    """
    result = run_frame("containers")
    types = [record.hazard_type for record in result.records]
    assert types == ["FLOOD"]
    flood = result.primary
    assert flood is not None
    assert flood.smoothed_area_m2 > 100.0
    assert flood.abs_xy[0] == pytest.approx(45.0, abs=8.0)
    assert flood.abs_xy[1] == pytest.approx(-56.0, abs=8.0)


@pytest.mark.parametrize("name", ["waste", "helipad", "home"])
def test_real_gazebo_clean_frames_have_no_hazards(name):
    """Мусорный полигон, вертодром и база не должны давать событий.

    На вертодроме проверяется оранжевое кольцо и серая плита, на базе —
    светлые крыши, синие окна и белая техника, на полигоне — река и грунт.
    """
    result = run_frame(name)
    assert result.records == []
    assert result.events == []


def test_full_frame_blue_background_is_not_flood():
    """Кадр, полностью залитый синим (небо/фон), — не затопление."""
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[..., 0] = 255   # B
    frame[..., 1] = 80    # G
    frame[..., 2] = 20    # R
    pipeline = VisionPipeline(
        camera=CAMERA,
        detector_config=DetectorConfig(track_min_hits=1),
        title="GAZEBO REGRESSION",
    )
    result = pipeline.process(frame, Pose.from_deg(94.0, -105.0, 22.0, 0.0),
                              draw=False)
    assert result.records == []
    assert result.events == []
