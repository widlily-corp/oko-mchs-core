"""Синтетический полигон для проверки и демонстрации алгоритма.

Зачем это нужно:
  * в репозитории нет лицензированных аэрофотоснимков ЧС, а жюри нужно
    показать работу алгоритма на тестовых изображениях;
  * для каждого очага известна наземная истина (координаты и площадь),
    поэтому можно измерить ошибку геопривязки, а не только «на глаз»;
  * сцена воспроизводима (фиксированный seed) и работает без Gazebo/ROS.

Мир — ортофото 1800×1300 px с масштабом 0.1 м/px. Кадр 640×480 вырезается
вокруг заданной точки; высота подобрана так, чтобы GSD кадра тоже был
0.1 м/px (надирная камера, 80°). Сцена север-вверх: рыскание БПЛА = 90°.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from .config import threat_level
from .geometry import Pose

SCALE_M_PER_PX = 0.1
WORLD_WIDTH = 1800
WORLD_HEIGHT = 1300
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Высота, при которой GSD надирной камеры 80° равен 0.1 м/px.
NADIR_ALTITUDE_M = round(
    SCALE_M_PER_PX * FRAME_WIDTH
    / (2.0 * math.tan(math.radians(80.0) / 2.0)),
    2,
)
SCENE_YAW_DEG = 90.0  # север-вверх


@dataclass
class SyntheticHazard:
    hazard_type: str
    center_px: Tuple[int, int]
    polygon_px: np.ndarray
    area_m2: float
    threat_level: str


@dataclass
class SceneSpec:
    name: str
    center_px: Tuple[int, int]
    title_ru: str


SCENES: Dict[str, SceneSpec] = {
    "fire_critical": SceneSpec("fire_critical", (420, 320),
                               "Крупный пожар (CRITICAL)"),
    "fire_advisory": SceneSpec("fire_advisory", (900, 260),
                               "Малый очаг (ADVISORY)"),
    "oil_warning": SceneSpec("oil_warning", (1350, 380),
                             "Разлив нефти (WARNING)"),
    "flood_critical": SceneSpec("flood_critical", (480, 1030),
                                "Затопление (CRITICAL)"),
    "smoke_only": SceneSpec("smoke_only", (1300, 980),
                            "Задымление (WARNING)"),
    "multi": SceneSpec("multi", (780, 540),
                       "Два очага в одном кадре"),
    "clean": SceneSpec("clean", (330, 650),
                       "Чистый лес (контроль ложных срабатываний)"),
}


class SyntheticArena:
    """Процедурный ортофотополигон с четырьмя типами ЧС."""

    def __init__(self, seed: int = 7, scale_m_per_px: float = SCALE_M_PER_PX):
        self.seed = seed
        self.scale = scale_m_per_px
        self.rng = np.random.default_rng(seed)
        self.world: Optional[np.ndarray] = None
        self.hazards: List[SyntheticHazard] = []

    # ------------------------------------------------------------------
    # Построение мира
    # ------------------------------------------------------------------
    def render(self) -> np.ndarray:
        self.rng = np.random.default_rng(self.seed)
        self.hazards = []
        world = self._forest_background()
        self._add_paths(world)

        self._add_fire(world, (420, 320), radius=48)
        self._add_fire(world, (900, 260), radius=16)
        self._add_fire(world, (790, 620), radius=14)
        self._add_oil(world, (1350, 380), rx=45, ry=30)
        self._add_oil(world, (860, 560), rx=24, ry=16)
        self._add_flood(world, (480, 1030), rx=95, ry=70)
        self._add_smoke(world, (1300, 980), rx=32, ry=22)

        add_noise = self.rng.normal(0.0, 3.0, world.shape)
        world = np.clip(world.astype(np.float32) + add_noise, 0, 255).astype(np.uint8)
        world = cv2.GaussianBlur(world, (3, 3), 0)
        self.world = world
        return world

    def _forest_background(self) -> np.ndarray:
        world = np.zeros((WORLD_HEIGHT, WORLD_WIDTH, 3), dtype=np.float32)
        world[:] = (34.0, 82.0, 46.0)

        palette = [
            (24.0, 68.0, 36.0),
            (38.0, 95.0, 50.0),
            (48.0, 110.0, 58.0),
            (30.0, 78.0, 42.0),
            (44.0, 102.0, 54.0),
        ]
        for _ in range(1600):
            x = int(self.rng.integers(0, WORLD_WIDTH))
            y = int(self.rng.integers(0, WORLD_HEIGHT))
            radius = int(self.rng.integers(7, 26))
            color = palette[int(self.rng.integers(0, len(palette)))]
            cv2.circle(world, (x, y), radius, color, -1, cv2.LINE_AA)

        noise = self.rng.normal(0.0, 4.0, world.shape)
        return np.clip(world + noise, 0, 255)

    def _add_paths(self, world: np.ndarray) -> None:
        road = (46.0, 80.0, 112.0)
        for offset in (-330, -310):
            points = []
            for x in range(0, WORLD_WIDTH, 60):
                y = int(WORLD_HEIGHT // 2 + offset
                        + 40 * math.sin(x / 240.0) + 15 * math.sin(x / 61.0))
                points.append((x, y))
            cv2.polylines(world, [np.asarray(points, np.int32)], False, road, 7,
                          cv2.LINE_AA)

        trail = [(120, 1100), (380, 980), (620, 900), (900, 940), (1180, 1080),
                 (1500, 1150)]
        cv2.polylines(world, [np.asarray(trail, np.int32)], False,
                      (52.0, 88.0, 120.0), 5, cv2.LINE_AA)

    # ------------------------------------------------------------------
    # Отдельные типы ЧС
    # ------------------------------------------------------------------
    def _add_fire(self, world: np.ndarray, center: Tuple[int, int],
                  radius: int) -> None:
        cx, cy = center
        points = []
        count = 28
        for index in range(count):
            angle = 2 * math.pi * index / count
            r = radius * (0.72 + 0.35 * self.rng.random())
            points.append((int(cx + r * math.cos(angle)),
                           int(cy + r * math.sin(angle))))
        polygon = np.asarray(points, np.int32)

        cv2.fillPoly(world, [polygon], (0.0, 42.0, 225.0), cv2.LINE_AA)
        inner = self._scaled_polygon(polygon, cx, cy, 0.72)
        cv2.fillPoly(world, [inner], (0.0, 120.0, 255.0), cv2.LINE_AA)
        core = self._scaled_polygon(polygon, cx, cy, 0.38)
        cv2.fillPoly(world, [core], (0.0, 215.0, 255.0), cv2.LINE_AA)
        for _ in range(radius // 3):
            angle = 2 * math.pi * self.rng.random()
            distance = radius * (1.0 + 0.5 * self.rng.random())
            x = int(cx + distance * math.cos(angle))
            y = int(cy + distance * math.sin(angle))
            cv2.circle(world, (x, y), int(self.rng.integers(1, 4)),
                       (0.0, 90.0, 245.0), -1, cv2.LINE_AA)

        self._register("FIRE", center, polygon)

    def _add_oil(self, world: np.ndarray, center: Tuple[int, int],
                 rx: int, ry: int) -> None:
        cx, cy = center
        points = []
        count = 30
        for index in range(count):
            angle = 2 * math.pi * index / count
            wobble = 0.82 + 0.30 * self.rng.random()
            points.append((int(cx + rx * wobble * math.cos(angle)),
                           int(cy + ry * wobble * math.sin(angle))))
        polygon = np.asarray(points, np.int32)

        cv2.fillPoly(world, [polygon], (36.0, 40.0, 44.0), cv2.LINE_8)
        inner = self._scaled_polygon(polygon, cx, cy, 0.62)
        cv2.fillPoly(world, [inner], (26.0, 30.0, 34.0), cv2.LINE_8)
        for _ in range(8):
            angle = 2 * math.pi * self.rng.random()
            x1 = int(cx + rx * 0.2 * math.cos(angle))
            y1 = int(cy + ry * 0.2 * math.sin(angle))
            x2 = int(cx + rx * 0.8 * math.cos(angle))
            y2 = int(cy + ry * 0.8 * math.sin(angle))
            cv2.line(world, (x1, y1), (x2, y2), (55.0, 58.0, 62.0),
                     int(self.rng.integers(1, 3)), cv2.LINE_AA)

        self._register("OIL_SPILL", center, polygon)

    def _add_flood(self, world: np.ndarray, center: Tuple[int, int],
                   rx: int, ry: int) -> None:
        cx, cy = center
        points = []
        count = 34
        for index in range(count):
            angle = 2 * math.pi * index / count
            wobble = 0.78 + 0.32 * self.rng.random()
            points.append((int(cx + rx * wobble * math.cos(angle)),
                           int(cy + ry * wobble * math.sin(angle))))
        polygon = np.asarray(points, np.int32)

        cv2.fillPoly(world, [polygon], (195.0, 110.0, 45.0), cv2.LINE_AA)
        for _ in range(14):
            angle = 2 * math.pi * self.rng.random()
            x = int(cx + rx * 0.55 * math.cos(angle))
            y = int(cy + ry * 0.55 * math.sin(angle))
            cv2.ellipse(world, (x, y), (int(rx * 0.18), int(ry * 0.10)),
                        self.rng.uniform(0, 180), 0, 360, (188.0, 128.0, 62.0),
                        1, cv2.LINE_AA)

        self._register("FLOOD", center, polygon)

    def _add_smoke(self, world: np.ndarray, center: Tuple[int, int],
                   rx: int, ry: int) -> None:
        cx, cy = center
        mask = np.zeros(world.shape[:2], dtype=np.uint8)
        for index in range(9):
            x = int(cx + index * rx * 0.16)
            y = int(cy - index * ry * 0.18)
            radius = int(rx * (0.45 + 0.08 * index))
            cv2.circle(mask, (x, y), radius, 255, -1, cv2.LINE_AA)

        alpha = cv2.GaussianBlur(mask.astype(np.float32) / 255.0, (41, 41), 0)
        alpha = np.clip((alpha - 0.18) / 0.55, 0.0, 1.0)[..., None] * 0.94
        smoke_color = np.array([180.0, 182.0, 184.0], dtype=np.float32)
        world[:] = world * (1.0 - alpha) + smoke_color * alpha

        core_mask = (alpha[..., 0] > 0.55).astype(np.uint8)
        contours, _ = cv2.findContours(core_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            polygon = max(contours, key=cv2.contourArea).reshape(-1, 2)
            self._register("SMOKE", center, polygon)
        else:
            polygon = np.asarray([[cx - rx, cy], [cx + rx, cy],
                                  [cx + rx, cy + ry], [cx - rx, cy + ry]], np.int32)
            self._register("SMOKE", center, polygon)

    # ------------------------------------------------------------------
    # Утилиты
    # ------------------------------------------------------------------
    @staticmethod
    def _scaled_polygon(polygon: np.ndarray, cx: int, cy: int,
                        factor: float) -> np.ndarray:
        points = polygon.reshape(-1, 2).astype(np.float32)
        points[:, 0] = cx + (points[:, 0] - cx) * factor
        points[:, 1] = cy + (points[:, 1] - cy) * factor
        return points.astype(np.int32)

    def _register(self, hazard_type: str, center_px: Tuple[int, int],
                  polygon_px: np.ndarray) -> None:
        polygon = np.asarray(polygon_px).reshape(-1, 2)
        moments = cv2.moments(polygon.astype(np.float32))
        if abs(moments["m00"]) > 1e-9:
            center = (
                int(round(moments["m10"] / moments["m00"])),
                int(round(moments["m01"] / moments["m00"])),
            )
        else:
            center = (int(center_px[0]), int(center_px[1]))
        area_m2 = abs(cv2.contourArea(polygon.astype(np.float32))) * (self.scale ** 2)
        level = threat_level(area_m2)
        self.hazards.append(SyntheticHazard(
            hazard_type=hazard_type,
            center_px=center,
            polygon_px=polygon,
            area_m2=float(area_m2),
            threat_level=level,
        ))

    # ------------------------------------------------------------------
    # Кадры и эталонные данные
    # ------------------------------------------------------------------
    def crop(self, center_px: Tuple[int, int],
             add_noise: bool = True) -> np.ndarray:
        if self.world is None:
            self.render()
        cx, cy = self._clamp_center(center_px)
        x1, y1 = cx - FRAME_WIDTH // 2, cy - FRAME_HEIGHT // 2
        frame = self.world[y1:y1 + FRAME_HEIGHT, x1:x1 + FRAME_WIDTH].copy()
        if add_noise:
            noise = self.rng.normal(0.0, 2.0, frame.shape)
            frame = np.clip(frame.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        return frame

    def pose_for(self, center_px: Tuple[int, int]) -> Pose:
        cx, cy = self._clamp_center(center_px)
        return Pose(
            x_m=cx * self.scale,
            y_m=(WORLD_HEIGHT - cy) * self.scale,
            z_m=NADIR_ALTITUDE_M,
            yaw_rad=math.radians(SCENE_YAW_DEG),
        )

    def ground_truth(self, center_px: Tuple[int, int]) -> List[dict]:
        cx, cy = self._clamp_center(center_px)
        x1, y1 = cx - FRAME_WIDTH // 2, cy - FRAME_HEIGHT // 2
        result = []
        for hazard in self.hazards:
            hx, hy = hazard.center_px
            if not (x1 <= hx < x1 + FRAME_WIDTH and y1 <= hy < y1 + FRAME_HEIGHT):
                continue
            world_x = hx * self.scale
            world_y = (WORLD_HEIGHT - hy) * self.scale
            polygon_world = np.column_stack((
                hazard.polygon_px[:, 0] * self.scale,
                (WORLD_HEIGHT - hazard.polygon_px[:, 1]) * self.scale,
            ))
            result.append({
                "hazard_type": hazard.hazard_type,
                "center_xy_m": [round(world_x, 2), round(world_y, 2)],
                "area_m2": round(hazard.area_m2, 2),
                "threat_level": hazard.threat_level,
                "polygon_xy_m": [[round(float(x), 2), round(float(y), 2)]
                                 for x, y in polygon_world],
            })
        return result

    @staticmethod
    def _clamp_center(center_px: Tuple[int, int]) -> Tuple[int, int]:
        cx = int(min(max(center_px[0], FRAME_WIDTH // 2),
                     WORLD_WIDTH - FRAME_WIDTH // 2))
        cy = int(min(max(center_px[1], FRAME_HEIGHT // 2),
                     WORLD_HEIGHT - FRAME_HEIGHT // 2))
        return cx, cy

    def camera_overrides(self) -> dict:
        """Параметры камеры для синтетических кадров (надир)."""
        return {
            "width": FRAME_WIDTH,
            "height": FRAME_HEIGHT,
            "hfov_deg": 80.0,
            "pitch_down_deg": 90.0,
            "mount_forward_m": 0.0,
            "mount_up_m": 0.0,
        }
