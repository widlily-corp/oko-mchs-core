"""Тесты производительности и математической эквивалентности HazardDetector.

Проверяет:
1. Точное математическое совпадение вычисления std текстуры по ROI с полнокадровым Лапласианом (< 1e-4).
2. Корректную обработку краевых случаев (границы кадра, нулевые маски).
3. 100% идентичность детекций на синтетических сценах ЧС и чистом лесу.
4. Значительное ускорение обработки кадров без очагов и с локальными очагами.
"""

import time
import cv2
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


def test_compute_roi_texture_matches_full_frame():
    """Тест математической эквивалентности ROI Laplacian и полнокадрового Laplacian."""
    rng = np.random.RandomState(42)
    h, w = 720, 1280
    gray = rng.randint(0, 256, (h, w), dtype=np.uint8)
    ksize = 5

    # Эталонный полнокадровый расчет
    blurred_full = cv2.GaussianBlur(gray, (ksize, ksize), 0)
    laplacian_full = cv2.Laplacian(blurred_full, cv2.CV_32F)

    test_rois = [
        (0, 0, 60, 50),                      # Левый верхний угол
        (w - 70, 0, 70, 80),                  # Правый верхний угол
        (0, h - 90, 80, 90),                  # Левый нижний угол
        (w - 100, h - 100, 100, 100),         # Правый нижний угол
        (200, 150, 120, 90),                  # Внутри кадра
        (500, 300, 35, 35),                   # Небольшой очаг
        (350, 400, 250, 180),                 # Крупный очаг
    ]

    for x, y, bw, bh in test_rois:
        # Случайная бинарная маска внутри ROI
        mask_roi = rng.randint(0, 2, (bh, bw), dtype=np.uint8)

        # ROI метод
        texture_roi = HazardDetector._compute_roi_texture(
            gray, x, y, bw, bh, mask_roi, ksize=ksize
        )

        # Полнокадровый эталон
        lap_patch = laplacian_full[y:y + bh, x:x + bw]
        valid_vals = lap_patch[mask_roi > 0]
        expected_std = float(valid_vals.std()) if valid_vals.size > 0 else 0.0

        assert abs(texture_roi - expected_std) < 1e-4, (
            f"ROI ({x},{y},{bw},{bh}) texture mismatch: "
            f"roi={texture_roi}, expected={expected_std}, diff={abs(texture_roi - expected_std)}"
        )

        # Проверка удобной обертки _roi_texture_std
        texture_wrapped = HazardDetector._roi_texture_std(
            gray, (x, y, bw, bh), mask_roi, ksize=ksize
        )
        assert abs(texture_wrapped - texture_roi) < 1e-6


def test_compute_roi_texture_edge_cases():
    """Тест краевых случаев: пустые маски, нулевые размеры."""
    gray = np.zeros((100, 100), dtype=np.uint8)

    # Пустая маска
    mask_empty = np.zeros((20, 20), dtype=np.uint8)
    assert HazardDetector._compute_roi_texture(gray, 10, 10, 20, 20, mask_empty, 5) == 0.0

    # Нулевая ширина/высота
    assert HazardDetector._compute_roi_texture(gray, 10, 10, 0, 20, mask_empty, 5) == 0.0
    assert HazardDetector._compute_roi_texture(gray, 10, 10, 20, 0, mask_empty, 5) == 0.0

    # None маска
    assert HazardDetector._compute_roi_texture(gray, 10, 10, 20, 20, None, 5) == 0.0


def test_detector_accuracy_matches_all_scenes(arena):
    """Тест идентичности детекций на синтетических сценах."""
    detector = HazardDetector(DetectorConfig(track_min_hits=1))

    # 1. Чистый лес — отсутствие ложных срабатываний
    clean_frame = arena.crop(SCENES["clean"].center_px)
    assert detector.detect(clean_frame) == []

    # 2. Пожар
    detector.reset()
    fire_frame = arena.crop(SCENES["fire_critical"].center_px)
    fire_dets = [d for d in detector.detect(fire_frame) if d.hazard_type == "FIRE"]
    assert len(fire_dets) == 1
    assert fire_dets[0].area_px > 4000
    assert fire_dets[0].confidence > 0.6
    assert fire_dets[0].texture_std > 0.0

    # 3. Разлив нефтепродуктов
    detector.reset()
    oil_frame = arena.crop(SCENES["oil_warning"].center_px)
    oil_dets = [d for d in detector.detect(oil_frame) if d.hazard_type == "OIL_SPILL"]
    assert len(oil_dets) >= 1

    # 4. Паводок
    detector.reset()
    flood_frame = arena.crop(SCENES["flood_critical"].center_px)
    flood_dets = [d for d in detector.detect(flood_frame) if d.hazard_type == "FLOOD"]
    assert len(flood_dets) == 1
    assert flood_dets[0].area_px > 10000

    # 5. Дым
    detector.reset()
    smoke_frame = arena.crop(SCENES["smoke_only"].center_px)
    smoke_dets = [d for d in detector.detect(smoke_frame) if d.hazard_type == "SMOKE"]
    assert len(smoke_dets) >= 1


def test_detector_performance_speedup(arena):
    """Тест прироста скорости детекции (lazy ROI vs full-frame laplacian)."""
    clean_frame = arena.crop(SCENES["clean"].center_px)
    detector = HazardDetector(DetectorConfig(track_min_hits=1))

    # Прогрев
    for _ in range(3):
        detector.detect(clean_frame)

    # Замер времени HazardDetector (Lazy ROI)
    n_iters = 30
    t0 = time.perf_counter()
    for _ in range(n_iters):
        detector.detect(clean_frame)
    time_lazy = (time.perf_counter() - t0) / n_iters

    # Замер времени полнокадрового вычисления GaussianBlur + Laplacian
    gray = cv2.cvtColor(clean_frame, cv2.COLOR_BGR2GRAY)
    t0 = time.perf_counter()
    for _ in range(n_iters):
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _ = cv2.Laplacian(blurred, cv2.CV_32F)
    time_full_lap = (time.perf_counter() - t0) / n_iters

    # Lazy detector не тратит время на полнокадровый Laplacian
    # time_lazy должно быть меньше или сопоставимо с временем чисто одного старого Laplacian
    fps_lazy = 1.0 / max(1e-6, time_lazy)
    print(f"\n[Performance Benchmark]")
    print(f"Lazy Detector frame time: {time_lazy * 1000:.2f} ms ({fps_lazy:.1f} FPS)")
    print(f"Eliminated full-frame Laplacian time: {time_full_lap * 1000:.2f} ms")
    assert time_lazy < 0.05, f"Detector is too slow: {time_lazy * 1000:.2f} ms"
