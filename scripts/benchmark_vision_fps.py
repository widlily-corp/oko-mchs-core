#!/usr/bin/env python3
"""Бенчмарк производительности бортового зрения «Око МЧС».

Измеряет FPS и время кадра для:
1. Изолированного HazardDetector (Lazy Laplacian).
2. Полного VisionPipeline (детекция + геопривязка + HUD).
3. LatestFrameStreamer (неблокирующий update + кэшированный snapshot).
"""

from __future__ import annotations

import os
import sys
import time
import cv2
import numpy as np

# Добавляем пути импорта
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(REPO_ROOT, "src")
VISION_DIR = os.path.join(SRC_DIR, "mchs_vision")
for path in (SRC_DIR, VISION_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

from mchs_vision.detector import HazardDetector
from mchs_vision.geometry import Pose
from mchs_vision.pipeline import VisionPipeline
from mchs_vision.simulated_scene import SCENES, SyntheticArena
from mchs_vision.web_stream import LatestFrameStreamer


def run_benchmark(num_frames: int = 100) -> None:
    print("=" * 65)
    print("  БОРТОВОЕ ЗРЕНИЕ «ОКО МЧС» — БЕНЧМАРК ПРОИЗВОДИТЕЛЬНОСТИ")
    print("=" * 65)

    # 1. Генерация тестовых сцен через SyntheticArena
    arena = SyntheticArena(seed=7)
    arena.render()
    frame_fire = arena.crop(SCENES["fire_critical"].center_px)
    frame_forest = arena.crop((200, 200))  # Чистый фон без очагов

    # Масштабируем до рабочего разрешения 1280x720
    frame_fire = cv2.resize(frame_fire, (1280, 720), interpolation=cv2.INTER_LINEAR)
    frame_forest = cv2.resize(frame_forest, (1280, 720), interpolation=cv2.INTER_LINEAR)

    # 2. Бенчмарк HazardDetector на чистом лесу (90% типового полета)
    detector = HazardDetector()
    # Прогрев
    detector.detect(frame_forest)

    start = time.perf_counter()
    for _ in range(num_frames):
        detector.detect(frame_forest)
    elapsed_forest = time.perf_counter() - start
    fps_forest = num_frames / elapsed_forest
    ms_forest = (elapsed_forest / num_frames) * 1000

    print(f"[Detector] Чистый фон/лес (1280x720, {num_frames} кадров):")
    print(f"  -> Время кадра: {ms_forest:.2f} мс | Частота: {fps_forest:.1f} FPS")

    # 3. Бенчмарк HazardDetector на сцене с очагом (пожар)
    detector.reset()
    detector.detect(frame_fire)

    start = time.perf_counter()
    for _ in range(num_frames):
        detector.detect(frame_fire)
    elapsed_fire = time.perf_counter() - start
    fps_fire = num_frames / elapsed_fire
    ms_fire = (elapsed_fire / num_frames) * 1000

    print(f"[Detector] Очаг ЧС пожар (1280x720, {num_frames} кадров):")
    print(f"  -> Время кадра: {ms_fire:.2f} мс | Частота: {fps_fire:.1f} FPS")

    # 4. Бенчмарк сквозного VisionPipeline (детекция + проекция + тактический HUD)
    pipeline = VisionPipeline()
    pose = Pose(x_m=0.0, y_m=0.0, z_m=22.0, yaw_rad=0.0)
    # Прогрев
    pipeline.process(frame_fire, pose, draw=True)

    start = time.perf_counter()
    for _ in range(num_frames):
        pipeline.process(frame_fire, pose, draw=True)
    elapsed_pipe = time.perf_counter() - start
    fps_pipe = num_frames / elapsed_pipe
    ms_pipe = (elapsed_pipe / num_frames) * 1000

    print(f"[Pipeline] Сквозной пайплайн + HUD (1280x720, {num_frames} кадров):")
    print(f"  -> Время кадра: {ms_pipe:.2f} мс | Частота: {fps_pipe:.1f} FPS")

    # 5. Бенчмарк LatestFrameStreamer
    streamer = LatestFrameStreamer(start_server=False)
    start = time.perf_counter()
    for _ in range(num_frames * 10):
        streamer.update(frame_fire)
    elapsed_update = time.perf_counter() - start
    us_update = (elapsed_update / (num_frames * 10)) * 1_000_000

    start = time.perf_counter()
    for _ in range(num_frames * 10):
        streamer.snapshot()
    elapsed_snap = time.perf_counter() - start
    us_snap = (elapsed_snap / (num_frames * 10)) * 1_000_000

    print(f"[Streamer] Неблокирующее обновление и кэшированный snapshot:")
    print(f"  -> update(): {us_update:.2f} мкс/вызов (практически 0 оверхеда)")
    print(f"  -> snapshot() (с кэшированием): {us_snap:.2f} мкс/вызов")
    print("=" * 65)
    print(f"  ИТОГ: Сквозной пайплайн выдаёт {fps_pipe:.1f} FPS на CPU 1280x720!")
    print("=" * 65)


if __name__ == "__main__":
    run_benchmark()
