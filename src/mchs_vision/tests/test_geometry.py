"""Тесты геометрии: пиксель → земля, площадь, WGS-84, уровни угрозы."""

import math

import numpy as np
import pytest

from mchs_vision.config import CameraConfig, GeoOrigin, threat_level
from mchs_vision.geometry import GroundProjector, Pose


@pytest.fixture()
def nadir_projector():
    camera = CameraConfig(
        width=640,
        height=480,
        hfov_deg=80.0,
        pitch_down_deg=90.0,
        mount_forward_m=0.0,
        mount_right_m=0.0,
        mount_up_m=0.0,
    )
    return GroundProjector(camera, GeoOrigin(lat=0.0, lon=0.0))


def test_center_pixel_maps_to_drone_position(nadir_projector):
    pose = Pose(x_m=10.0, y_m=-5.0, z_m=38.14, yaw_rad=math.radians(90))
    camera = nadir_projector.camera
    x, y = nadir_projector.pixel_to_ground(camera.cx, camera.cy, pose)
    assert x == pytest.approx(10.0, abs=1e-6)
    assert y == pytest.approx(-5.0, abs=1e-6)


def test_nadir_gsd_matches_altitude(nadir_projector):
    pose = Pose(x_m=0.0, y_m=0.0, z_m=38.14, yaw_rad=math.radians(90))
    gsd_x, gsd_y = nadir_projector.gsd(pose)
    expected = 2.0 * 38.14 * math.tan(math.radians(80.0) / 2.0) / 640.0
    assert gsd_x == pytest.approx(expected, rel=1e-3)
    assert gsd_y == pytest.approx(expected, rel=1e-3)


def test_right_and_top_pixels_have_correct_signs(nadir_projector):
    pose = Pose(x_m=0.0, y_m=0.0, z_m=38.14, yaw_rad=math.radians(90))
    center = nadir_projector.pixel_to_ground(320, 240, pose)
    right = nadir_projector.pixel_to_ground(639, 240, pose)
    top = nadir_projector.pixel_to_ground(320, 0, pose)

    assert right[0] > center[0]  # вправо по кадру = восток
    assert top[1] > center[1]    # вверх по кадру = север


def test_area_of_known_rectangle(nadir_projector):
    # 100×100 px при GSD≈0.1 м/px — это примерно 100 м².
    contour = np.array([[[270, 190]], [[370, 190]], [[370, 290]], [[270, 290]]],
                       dtype=np.int32)
    pose = Pose(x_m=0.0, y_m=0.0, z_m=38.14, yaw_rad=math.radians(90))
    metrics = nadir_projector.region_metrics(contour, pose)
    assert metrics is not None
    assert metrics.area_m2 == pytest.approx(100.0, rel=0.02)


def test_tilted_camera_center_ray_distance():
    camera = CameraConfig(
        width=640,
        height=480,
        hfov_deg=80.0,
        pitch_down_deg=34.4,
        mount_forward_m=0.0,
        mount_right_m=0.0,
        mount_up_m=0.0,
    )
    projector = GroundProjector(camera, GeoOrigin())
    pose = Pose(x_m=0.0, y_m=0.0, z_m=38.14, yaw_rad=math.radians(90))
    x, y = projector.pixel_to_ground(camera.cx, camera.cy, pose)
    expected = 38.14 / math.tan(math.radians(34.4))
    assert y == pytest.approx(expected, rel=1e-3)
    assert x == pytest.approx(0.0, abs=1e-6)


def test_latlon_conversion():
    projector = GroundProjector(CameraConfig(), GeoOrigin(lat=0.0, lon=0.0))
    lat, lon = projector.local_to_latlon(0.0, 111_320.0)
    assert lat == pytest.approx(1.0, rel=1e-6)
    assert lon == pytest.approx(0.0, abs=1e-9)


def test_threat_level_boundaries():
    assert threat_level(0.0) == "ADVISORY"
    assert threat_level(14.99) == "ADVISORY"
    assert threat_level(15.0) == "WARNING"
    assert threat_level(50.0) == "WARNING"
    assert threat_level(50.01) == "CRITICAL"
