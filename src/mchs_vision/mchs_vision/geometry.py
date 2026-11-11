"""Геометрия «пиксель → точка на земле» для бортовой камеры БПЛА.

Модель:
    * локальная система координат — ENU-подобная: X — восток, Y — север,
      Z — вверх; рыскание (yaw) отсчитывается от +X против часовой стрелки
      (стандарт ROS REP-103);
    * камера закреплена на дроне и наклонена вниз на угол pitch_down_deg
      (0° — горизонт, 90° — строго вниз);
    * земля — плоскость Z = 0;
    * пиксели квадратные, главная точка — центр кадра.

Для надирной камеры (pitch_down_deg = 90°) формулы сводятся к виду из ТЗ:

    ΔX = (u - W/2) / W * Z * 2 * tan(FOV_x / 2)
    ΔY = (v - H/2) / H * Z * 2 * tan(FOV_y / 2)

Но в общем случае используется честное пересечение луча с плоскостью земли,
поэтому модуль корректно работает и с наклонной камерой 45° из Gazebo,
и с перспективными искажениями: площадь считается по спроецированному
на землю полигону, а не по числу пикселей.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from .config import CameraConfig, GeoOrigin

EARTH_METERS_PER_DEG_LAT = 111_320.0


@dataclass(frozen=True)
class Pose:
    """Положение и ориентация БПЛА в локальной системе координат (метры)."""

    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 8.0
    yaw_rad: float = 0.0

    @classmethod
    def from_deg(cls, x_m: float, y_m: float, z_m: float,
                 yaw_deg: float = 0.0) -> "Pose":
        return cls(x_m=x_m, y_m=y_m, z_m=z_m, yaw_rad=math.radians(yaw_deg))

    @property
    def yaw_deg(self) -> float:
        return math.degrees(self.yaw_rad)


@dataclass
class RegionMetrics:
    """Результат проецирования контура ЧС на землю."""

    area_m2: float
    center_xy: Tuple[float, float]
    rel_xy: Tuple[float, float]
    distance_m: float
    bearing_deg: float
    gsd_x_m: float
    gsd_y_m: float
    polygon_xy: np.ndarray

    @property
    def area_ha(self) -> float:
        return self.area_m2 / 10_000.0


class GroundProjector:
    """Проектор пикселей кадра в локальные метры на плоскости земли."""

    def __init__(self, camera: CameraConfig, origin: GeoOrigin = GeoOrigin()):
        self.camera = camera
        self.origin = origin

    # ------------------------------------------------------------------
    # Базовая геометрия
    # ------------------------------------------------------------------
    def _camera_frame(self, pose: Pose
                      ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Вернуть (origin, forward, right, down) камеры в мировой СК.

        Векторы forward/right/down — единичные орты оптической системы:
        forward — куда смотрит камера, right — вправо по кадру,
        down — вниз по кадру.
        """
        cy, sy = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)

        # Орты корпуса дрона в мировой СК (FLU: X вперёд, Y влево, Z вверх).
        body_forward = np.array([cy, sy, 0.0], dtype=np.float64)
        body_left = np.array([-sy, cy, 0.0], dtype=np.float64)
        body_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        pitch = math.radians(self.camera.pitch_down_deg)
        cam_forward = math.cos(pitch) * body_forward - math.sin(pitch) * body_up
        cam_right = -body_left
        cam_down = -math.sin(pitch) * body_forward - math.cos(pitch) * body_up

        origin = np.array([pose.x_m, pose.y_m, pose.z_m], dtype=np.float64)
        origin = (
            origin
            + body_forward * self.camera.mount_forward_m
            + (-body_left) * self.camera.mount_right_m
            + body_up * self.camera.mount_up_m
        )
        return origin, cam_forward, cam_right, cam_down

    def pixel_ray(self, u: float, v: float, pose: Pose
                  ) -> Tuple[np.ndarray, np.ndarray]:
        """Луч из оптического центра через пиксель (u, v) в мировой СК."""
        origin, forward, right, down = self._camera_frame(pose)
        x_n = (float(u) - self.camera.cx) / self.camera.fx
        y_n = (float(v) - self.camera.cy) / self.camera.fy
        direction = forward + x_n * right + y_n * down
        return origin, direction

    def pixel_to_ground(self, u: float, v: float, pose: Pose,
                        max_range_m: Optional[float] = None
                        ) -> Optional[Tuple[float, float]]:
        """Пересечение луча пикселя с плоскостью Z = 0.

        Возвращает (x, y) в метрах или None, если луч уходит выше горизонта,
        пересечение за спиной/слишком далеко.
        """
        origin, direction = self.pixel_ray(u, v, pose)
        if direction[2] >= -1e-4:
            return None  # луч параллелен земле или смотрит в небо

        t = (0.0 - origin[2]) / direction[2]
        if t <= 0.0:
            return None

        range_m = max_range_m if max_range_m is not None else self.camera.max_range_m
        distance = t * float(np.linalg.norm(direction))
        if distance < self.camera.min_range_m or distance > range_m:
            return None

        point = origin + t * direction
        return float(point[0]), float(point[1])

    # ------------------------------------------------------------------
    # Контуры и площади
    # ------------------------------------------------------------------
    def contour_to_ground(self, contour: np.ndarray, pose: Pose,
                          max_points: int = 180) -> Optional[np.ndarray]:
        """Спроецировать контур OpenCV на землю и вернуть выпуклый полигон.

        Прореживание точек нужно только для скорости: контур пожара может
        содержать тысячи пикселей, а для площади достаточно ~180.
        """
        if contour is None or len(contour) < 3:
            return None

        points = contour.reshape(-1, 2)
        step = max(1, len(points) // max_points)
        projected = []
        for u, v in points[::step]:
            ground = self.pixel_to_ground(float(u), float(v), pose)
            if ground is not None:
                projected.append(ground)

        if len(projected) < 3:
            return None

        polygon = np.asarray(projected, dtype=np.float32)

        # Проекция монотонна, поэтому простой контур остаётся простым.
        # Выпуклую оболочку НЕ берём: у пламени бывает вогнутая форма,
        # и hull завышает площадь на 10–15%.
        if cv2.contourArea(polygon) <= 0.0:
            hull = cv2.convexHull(polygon)
            if hull is None or len(hull) < 3:
                return None
            polygon = hull.reshape(-1, 2).astype(np.float32)

        if cv2.contourArea(polygon) <= 0.0:
            return None
        return polygon.reshape(-1, 2)

    def region_metrics(self, contour: np.ndarray, pose: Pose
                       ) -> Optional[RegionMetrics]:
        """Площадь, центр, дальность и пеленг очага ЧС в локальных метрах."""
        polygon = self.contour_to_ground(contour, pose)
        if polygon is None:
            return None

        area = float(cv2.contourArea(polygon))
        if area <= 0.0:
            return None

        moments = cv2.moments(polygon)
        if abs(moments["m00"]) > 1e-9:
            cx = float(moments["m10"] / moments["m00"])
            cy = float(moments["m01"] / moments["m00"])
        else:
            cx, cy = float(polygon[:, 0].mean()), float(polygon[:, 1].mean())

        rel_x = cx - pose.x_m
        rel_y = cy - pose.y_m
        distance = math.hypot(rel_x, rel_y)
        bearing = math.degrees(math.atan2(rel_x, rel_y)) % 360.0
        gsd_x, gsd_y = self.gsd(pose)

        return RegionMetrics(
            area_m2=area,
            center_xy=(cx, cy),
            rel_xy=(rel_x, rel_y),
            distance_m=distance,
            bearing_deg=bearing,
            gsd_x_m=gsd_x,
            gsd_y_m=gsd_y,
            polygon_xy=polygon,
        )

    def gsd(self, pose: Pose) -> Tuple[float, float]:
        """Размер пикселя на земле (м/пиксель) в центре кадра."""
        center = self.pixel_to_ground(self.camera.cx, self.camera.cy, pose)
        right = self.pixel_to_ground(self.camera.cx + 1.0, self.camera.cy, pose)
        down = self.pixel_to_ground(self.camera.cx, self.camera.cy + 1.0, pose)

        if center is None or right is None or down is None:
            # Страховка для случая, когда центр кадра не пересекает землю
            # (например, камера смотрит в горизонт).
            gsd = self.camera.gsd_at_nadir_m * max(pose.z_m, 0.1)
            return gsd, gsd

        gsd_x = math.hypot(right[0] - center[0], right[1] - center[1])
        gsd_y = math.hypot(down[0] - center[0], down[1] - center[1])
        return gsd_x, gsd_y

    def footprint_size_m(self, pose: Pose) -> Tuple[float, float]:
        """Приблизительный размер кадра на земле в метрах."""
        gsd_x, gsd_y = self.gsd(pose)
        return gsd_x * self.camera.width, gsd_y * self.camera.height

    def north_screen_direction(self, pose: Pose) -> Tuple[float, float]:
        """Куда указывает север на кадре (dx, dy) — для компаса в HUD."""
        _, _, cam_right, cam_down = self._camera_frame(pose)
        north = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        return float(np.dot(north, cam_right)), float(np.dot(north, cam_down))

    # ------------------------------------------------------------------
    # Географическая привязка
    # ------------------------------------------------------------------
    def local_to_latlon(self, x_m: float, y_m: float) -> Tuple[float, float]:
        """Пересчитать локальные метры (X — восток, Y — север) в WGS-84."""
        lat = self.origin.lat + y_m / EARTH_METERS_PER_DEG_LAT
        cos_lat = math.cos(math.radians(self.origin.lat))
        lon = self.origin.lon + x_m / (EARTH_METERS_PER_DEG_LAT * max(cos_lat, 1e-6))
        return lat, lon
