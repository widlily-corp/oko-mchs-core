"""Тесты для модуля калиброванного расчёта площади возгорания по фото."""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import tempfile
import cv2
import numpy as np
import pytest

from mchs_vision.config import CameraConfig, GeoOrigin
from mchs_vision.geometry import Pose
from mchs_vision.fire_area import FireAreaEstimator, FireAreaResult


@pytest.fixture
def nadir_camera() -> CameraConfig:
    return CameraConfig(
        width=1280,
        height=720,
        hfov_deg=80.0,
        pitch_down_deg=90.0,
        mount_forward_m=0.0,
        mount_up_m=0.0,
    )


@pytest.fixture
def tilted_camera() -> CameraConfig:
    return CameraConfig(
        width=1280,
        height=720,
        hfov_deg=80.0,
        pitch_down_deg=45.0,
        mount_forward_m=0.42,
        mount_up_m=0.12,
    )


def create_synthetic_fire_image(width=1280, height=720, flame_radius=50, smoke_radius=90) -> np.ndarray:
    """Создаёт тестовый кадр с тёмным фоном (трава), огнём в центре и дымом вокруг."""
    # Тёмно-зелёный фон (BGR: 25, 45, 20)
    img = np.full((height, width, 3), (25, 45, 20), dtype=np.uint8)
    cx, cy = width // 2, height // 2

    # Дым (серовато-белый эллипс, BGR: 180, 180, 190)
    cv2.circle(img, (cx, cy), smoke_radius, (180, 180, 190), -1)

    # Огонь (ярко-оранжевый круг, BGR: 15, 120, 245 -> HSV: H~12, S~240, V~245)
    cv2.circle(img, (cx, cy), flame_radius, (15, 120, 245), -1)

    # Внутреннее белое ядро пламени (BGR: 200, 240, 255)
    cv2.circle(img, (cx, cy), flame_radius // 2, (200, 240, 255), -1)
    return img


def test_fire_area_nadir_synthetic_geometry(nadir_camera):
    """Проверяет точность вычисления площади круга на высоте 22 м."""
    estimator = FireAreaEstimator(camera=nadir_camera)
    radius_px = 60
    img = create_synthetic_fire_image(flame_radius=radius_px, smoke_radius=0)
    pose = Pose(x_m=0.0, y_m=0.0, z_m=22.0, yaw_rad=0.0)

    result = estimator.estimate(img, pose=pose)

    assert isinstance(result, FireAreaResult)
    assert result.has_fire is True
    assert result.flame_area_px > 0
    assert result.flame_area_m2 > 0.0
    assert result.flame_area_ha > 0.0

    # Аналитический расчёт GSD для HFOV=80°, W=1280, Z=22 м
    gsd_x = (2.0 * 22.0 * math.tan(math.radians(40.0))) / 1280.0
    expected_m2 = math.pi * (radius_px * gsd_x) ** 2

    # Погрешность растрирования круга в пикселях не должна превышать 8%
    rel_error = abs(result.flame_area_m2 - expected_m2) / expected_m2
    assert rel_error < 0.08, f"Calculated {result.flame_area_m2:.2f} m2 vs expected {expected_m2:.2f} m2 (err={rel_error:.1%})"


def test_fire_and_smoke_separation(nadir_camera):
    """Проверяет корректное разделение ядра пламени и шлейфа дыма."""
    estimator = FireAreaEstimator(camera=nadir_camera)
    img = create_synthetic_fire_image(flame_radius=40, smoke_radius=100)
    pose = Pose(x_m=10.0, y_m=20.0, z_m=25.0)

    result = estimator.estimate(img, pose=pose)

    assert result.has_fire is True
    assert result.has_smoke is True
    assert result.flame_area_m2 > 0.0
    assert result.smoke_area_m2 > result.flame_area_m2
    assert result.total_area_m2 >= result.smoke_area_m2


def test_tactical_overlay_rendering(nadir_camera):
    """Проверяет наложение тактической разметки: шкала масштаба, контуры и плашка."""
    estimator = FireAreaEstimator(camera=nadir_camera)
    img = create_synthetic_fire_image(flame_radius=50, smoke_radius=80)
    pose = Pose(x_m=0.0, y_m=0.0, z_m=22.0)

    result = estimator.estimate(img, pose=pose)
    annotated = result.annotated_image

    assert annotated is not None
    assert annotated.shape == img.shape
    # Кадр разметки не должен быть идентичен исходному
    assert not np.array_equal(annotated, img)
    assert result.scale_bar_m in (5, 10, 20, 50, 100)
    assert result.scale_bar_px > 0


def test_tilted_perspective_area(tilted_camera):
    """Проверяет корректность расчёта при наклонной камере 45°."""
    estimator = FireAreaEstimator(camera=tilted_camera)
    img = create_synthetic_fire_image(flame_radius=45, smoke_radius=70)
    pose = Pose(x_m=5.0, y_m=-12.0, z_m=22.0, yaw_rad=math.radians(30.0))

    result = estimator.estimate(img, pose=pose)
    assert result.has_fire is True
    assert result.flame_area_m2 > 0.0
    assert math.isfinite(result.flame_area_m2)


def test_cli_estimate_fire_area():
    """Проверяет запуск CLI утилиты scripts/estimate_fire_area.py."""
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    script_path = os.path.join(repo_root, "scripts", "estimate_fire_area.py")

    with tempfile.TemporaryDirectory() as tmp_dir:
        input_img_path = os.path.join(tmp_dir, "test_fire.jpg")
        output_img_path = os.path.join(tmp_dir, "annotated_fire.jpg")
        json_path = os.path.join(tmp_dir, "metrics.json")

        img = create_synthetic_fire_image(flame_radius=50, smoke_radius=90)
        cv2.imwrite(input_img_path, img)

        env = os.environ.copy()
        pythonpath = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = f"{os.path.join(repo_root, 'src', 'mchs_vision')}:{os.path.join(repo_root, 'src')}:{pythonpath}"

        cmd = [
            sys.executable,
            script_path,
            "--input", input_img_path,
            "--altitude", "22.0",
            "--pitch", "90.0",
            "--output", output_img_path,
            "--json", json_path,
        ]

        proc = subprocess.run(cmd, env=env, capture_output=True, text=True)
        assert proc.returncode == 0, f"CLI failed: {proc.stderr}"
        assert os.path.exists(output_img_path)
        assert os.path.exists(json_path)

        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        assert data["has_fire"] is True
        assert data["flame_area_m2"] > 0.0
        assert "flame_area_ha" in data
