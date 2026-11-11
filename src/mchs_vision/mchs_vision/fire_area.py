"""Калиброванный расчёт площади возгорания и шлейфа задымления по аэрофотосъёмке.

Модуль выполняет:
1. Цветовую и текстурную сегментацию пламени (FLAME) и дыма (SMOKE) в пространстве HSV.
2. Проекцию контуров пикселей на плоскость земли (Z = 0) с учётом угла наклона камеры,
   высоты полёта Z и калибровки сенсора (HFOV, фокусное расстояние).
3. Вычисление физической площади в квадратных метрах (м²) и гектарах (га).
4. Отрисовку тактического оверлея: полупрозрачная заливка, контуры, шкала масштаба (scale bar).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .config import CameraConfig
from .geometry import GroundProjector, Pose


@dataclass
class FireAreaResult:
    """Результаты калиброванного расчёта площади пожара."""

    has_fire: bool = False
    has_smoke: bool = False
    flame_area_px: int = 0
    smoke_area_px: int = 0
    flame_area_m2: float = 0.0
    smoke_area_m2: float = 0.0
    total_area_m2: float = 0.0
    flame_area_ha: float = 0.0
    smoke_area_ha: float = 0.0
    total_area_ha: float = 0.0
    perimeter_m: float = 0.0
    gsd_m_per_px: float = 0.0
    scale_bar_m: int = 10
    scale_bar_px: int = 0
    centroid_px: Tuple[int, int] = (0, 0)
    ground_centroid_xy: Tuple[float, float] = (0.0, 0.0)
    flame_contours: List[np.ndarray] = field(default_factory=list)
    smoke_contours: List[np.ndarray] = field(default_factory=list)
    annotated_image: Optional[np.ndarray] = None


class FireAreaEstimator:
    """Оценщик площади возгорания и задымления по фотоснимкам БПЛА."""

    # Калиброванные диапазоны HSV для открытого огня (пламя в Gazebo и реале)
    # Диапазон 1: ярко-оранжевый / жёлтый
    FLAME_HSV_LOW_1 = np.array([5, 120, 160], dtype=np.uint8)
    FLAME_HSV_HIGH_1 = np.array([28, 255, 255], dtype=np.uint8)
    # Диапазон 2: интенсивный красный
    FLAME_HSV_LOW_2 = np.array([0, 140, 160], dtype=np.uint8)
    FLAME_HSV_HIGH_2 = np.array([6, 255, 255], dtype=np.uint8)
    # Белое горячее ядро пламени (высокая яркость, умеренное насыщение)
    CORE_HSV_LOW = np.array([0, 0, 240], dtype=np.uint8)
    CORE_HSV_HIGH = np.array([180, 70, 255], dtype=np.uint8)

    # Дым (серовато-белые полупрозрачные области)
    SMOKE_HSV_LOW = np.array([0, 0, 140], dtype=np.uint8)
    SMOKE_HSV_HIGH = np.array([180, 65, 230], dtype=np.uint8)

    def __init__(self, camera: Optional[CameraConfig] = None):
        self.camera = camera or CameraConfig(
            width=1280,
            height=720,
            hfov_deg=80.0,
            pitch_down_deg=90.0,
        )
        self.projector = GroundProjector(self.camera)

    def compute_gsd(self, altitude_m: float) -> Tuple[float, float]:
        """Расчёт Ground Sampling Distance (GSD) в метрах на пиксель в надире."""
        hfov_rad = math.radians(self.camera.hfov_deg)
        vfov_rad = math.radians(self.camera.vfov_deg)
        ground_w = 2.0 * altitude_m * math.tan(hfov_rad / 2.0)
        ground_h = 2.0 * altitude_m * math.tan(vfov_rad / 2.0)
        gsd_x = ground_w / float(self.camera.width)
        gsd_y = ground_h / float(self.camera.height)
        return gsd_x, gsd_y

    def segment_flame_and_smoke(self, image_bgr: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Сегментация масок огня и дыма."""
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        # Маска пламени
        mask1 = cv2.inRange(hsv, self.FLAME_HSV_LOW_1, self.FLAME_HSV_HIGH_1)
        mask2 = cv2.inRange(hsv, self.FLAME_HSV_LOW_2, self.FLAME_HSV_HIGH_2)
        mask_core = cv2.inRange(hsv, self.CORE_HSV_LOW, self.CORE_HSV_HIGH)
        flame_mask = cv2.bitwise_or(mask1, mask2)
        flame_mask = cv2.bitwise_or(flame_mask, mask_core)

        # Морфологическая очистка пламени
        kernel_flame = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        flame_mask = cv2.morphologyEx(flame_mask, cv2.MORPH_OPEN, kernel_flame)
        flame_mask = cv2.morphologyEx(flame_mask, cv2.MORPH_CLOSE, kernel_flame)

        # Маска дыма (исключая прямое пламя)
        smoke_raw = cv2.inRange(hsv, self.SMOKE_HSV_LOW, self.SMOKE_HSV_HIGH)
        smoke_mask = cv2.bitwise_and(smoke_raw, cv2.bitwise_not(flame_mask))
        kernel_smoke = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        smoke_mask = cv2.morphologyEx(smoke_mask, cv2.MORPH_OPEN, kernel_smoke)
        smoke_mask = cv2.morphologyEx(smoke_mask, cv2.MORPH_CLOSE, kernel_smoke)

        return flame_mask, smoke_mask

    def estimate(
        self,
        image_bgr: np.ndarray,
        pose: Optional[Pose] = None,
        altitude_m: Optional[float] = None,
        annotate: bool = True,
    ) -> FireAreaResult:
        """Полный расчёт площади по фотоснимку с геометрической привязкой."""
        if pose is None:
            alt = altitude_m if altitude_m is not None else 22.0
            pose = Pose(x_m=0.0, y_m=0.0, z_m=max(1.0, alt), yaw_rad=0.0)

        # Обновляем разрешение камеры, если изображение отличается от дефолтного
        h, w = image_bgr.shape[:2]
        if w != self.camera.width or h != self.camera.height:
            self.camera = CameraConfig(
                width=w,
                height=h,
                hfov_deg=self.camera.hfov_deg,
                pitch_down_deg=self.camera.pitch_down_deg,
                mount_forward_m=self.camera.mount_forward_m,
                mount_up_m=self.camera.mount_up_m,
                max_range_m=self.camera.max_range_m,
            )
            self.projector = GroundProjector(self.camera)

        flame_mask, smoke_mask = self.segment_flame_and_smoke(image_bgr)

        # Поиск контуров
        flame_contours, _ = cv2.findContours(
            flame_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        smoke_contours, _ = cv2.findContours(
            smoke_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # Фильтр контуров по минимальной площади (отсечение одиночных шумов)
        flame_contours = [c for c in flame_contours if cv2.contourArea(c) >= 20.0]
        smoke_contours = [c for c in smoke_contours if cv2.contourArea(c) >= 50.0]

        flame_area_px = sum(int(cv2.contourArea(c)) for c in flame_contours)
        smoke_area_px = sum(int(cv2.contourArea(c)) for c in smoke_contours)

        flame_area_m2 = 0.0
        smoke_area_m2 = 0.0
        perimeter_m = 0.0
        centroids = []

        # Расчёт физической площади пламени через GroundProjector
        for contour in flame_contours:
            metrics = self.projector.region_metrics(contour, pose)
            if metrics is not None:
                flame_area_m2 += metrics.area_m2
                centroids.append(metrics.center_xy)
            else:
                # Аналитический fallback по GSD в надире
                gsd_x, gsd_y = self.compute_gsd(pose.z_m)
                flame_area_m2 += float(cv2.contourArea(contour)) * gsd_x * gsd_y

            # Периметр
            gsd_x, _ = self.compute_gsd(pose.z_m)
            perimeter_m += float(cv2.arcLength(contour, True)) * gsd_x

        # Расчёт физической площади дыма
        for contour in smoke_contours:
            metrics = self.projector.region_metrics(contour, pose)
            if metrics is not None:
                smoke_area_m2 += metrics.area_m2
            else:
                gsd_x, gsd_y = self.compute_gsd(pose.z_m)
                smoke_area_m2 += float(cv2.contourArea(contour)) * gsd_x * gsd_y

        total_area_m2 = flame_area_m2 + smoke_area_m2
        has_fire = flame_area_px > 0 or flame_area_m2 > 0.0
        has_smoke = smoke_area_px > 0 or smoke_area_m2 > 0.0

        # Средний центроид
        if centroids:
            ground_cx = float(np.mean([pt[0] for pt in centroids]))
            ground_cy = float(np.mean([pt[1] for pt in centroids]))
        else:
            ground_cx, ground_cy = pose.x_m, pose.y_m

        # Центроид в пикселях
        if flame_contours:
            all_pts = np.vstack(flame_contours)
            M = cv2.moments(all_pts)
            if abs(M["m00"]) > 1e-4:
                cx_px = int(M["m10"] / M["m00"])
                cy_px = int(M["m01"] / M["m00"])
            else:
                cx_px, cy_px = w // 2, h // 2
        else:
            cx_px, cy_px = w // 2, h // 2

        gsd_x, _ = self.compute_gsd(pose.z_m)

        # Подбор шкалы масштаба (10, 20, 50, 100 метров)
        desired_px = 140
        available_scales = [5, 10, 20, 50, 100, 200]
        chosen_scale_m = 10
        best_diff = float("inf")
        for s in available_scales:
            px_len = s / max(1e-5, gsd_x)
            diff = abs(px_len - desired_px)
            if diff < best_diff:
                best_diff = diff
                chosen_scale_m = s
        scale_bar_px = int(chosen_scale_m / max(1e-5, gsd_x))

        annotated = None
        if annotate:
            annotated = self._render_annotation(
                image_bgr=image_bgr,
                flame_contours=flame_contours,
                smoke_contours=smoke_contours,
                flame_area_m2=flame_area_m2,
                smoke_area_m2=smoke_area_m2,
                altitude_m=pose.z_m,
                scale_bar_m=chosen_scale_m,
                scale_bar_px=scale_bar_px,
                centroid_px=(cx_px, cy_px),
            )

        return FireAreaResult(
            has_fire=has_fire,
            has_smoke=has_smoke,
            flame_area_px=flame_area_px,
            smoke_area_px=smoke_area_px,
            flame_area_m2=round(flame_area_m2, 2),
            smoke_area_m2=round(smoke_area_m2, 2),
            total_area_m2=round(total_area_m2, 2),
            flame_area_ha=round(flame_area_m2 / 10_000.0, 4),
            smoke_area_ha=round(smoke_area_m2 / 10_000.0, 4),
            total_area_ha=round(total_area_m2 / 10_000.0, 4),
            perimeter_m=round(perimeter_m, 2),
            gsd_m_per_px=round(gsd_x, 4),
            scale_bar_m=chosen_scale_m,
            scale_bar_px=scale_bar_px,
            centroid_px=(cx_px, cy_px),
            ground_centroid_xy=(round(ground_cx, 2), round(ground_cy, 2)),
            flame_contours=flame_contours,
            smoke_contours=smoke_contours,
            annotated_image=annotated,
        )

    def _render_annotation(
        self,
        image_bgr: np.ndarray,
        flame_contours: List[np.ndarray],
        smoke_contours: List[np.ndarray],
        flame_area_m2: float,
        smoke_area_m2: float,
        altitude_m: float,
        scale_bar_m: int,
        scale_bar_px: int,
        centroid_px: Tuple[int, int],
    ) -> np.ndarray:
        """Отрисовка тактической инфографики поверх кадра."""
        annotated = image_bgr.copy()
        h, w = annotated.shape[:2]

        # 1. Полупрозрачные заливки очагов
        overlay = annotated.copy()
        if smoke_contours:
            cv2.drawContours(overlay, smoke_contours, -1, (160, 160, 180), -1)
        if flame_contours:
            cv2.drawContours(overlay, flame_contours, -1, (0, 90, 255), -1)

        cv2.addWeighted(overlay, 0.35, annotated, 0.65, 0, annotated)

        # 2. Чёткие границы контуров
        if smoke_contours:
            cv2.drawContours(annotated, smoke_contours, -1, (200, 200, 220), 2)
        if flame_contours:
            cv2.drawContours(annotated, flame_contours, -1, (0, 60, 255), 2)
            cv2.circle(annotated, centroid_px, 6, (0, 255, 255), -1)
            cv2.circle(annotated, centroid_px, 12, (0, 90, 255), 2)

        # 3. Тактическая плашка с метриками (левый верхний угол)
        card_w, card_h = 360, 110
        cv2.rectangle(annotated, (15, 15), (15 + card_w, 15 + card_h), (15, 20, 28), -1)
        cv2.rectangle(annotated, (15, 15), (15 + card_w, 15 + card_h), (0, 120, 255), 2)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(annotated, "MCHS FIRE RECON // AREA ESTIMATION", (25, 38), font, 0.52, (0, 210, 255), 1, cv2.LINE_AA)
        flame_ha = flame_area_m2 / 10_000.0
        cv2.putText(annotated, f"FLAME AREA: {flame_area_m2:.1f} m2 ({flame_ha:.3f} ha)", (25, 64), font, 0.60, (0, 90, 255), 2, cv2.LINE_AA)
        smoke_ha = smoke_area_m2 / 10_000.0
        cv2.putText(annotated, f"SMOKE PLUME: {smoke_area_m2:.1f} m2 ({smoke_ha:.3f} ha)", (25, 88), font, 0.50, (180, 190, 200), 1, cv2.LINE_AA)
        cv2.putText(annotated, f"ALTITUDE: {altitude_m:.1f} m | GSD: {(altitude_m*2*math.tan(math.radians(40))/w*100):.1f} cm/px", (25, 112), font, 0.44, (120, 140, 160), 1, cv2.LINE_AA)

        # 4. Шкала масштаба (Scale Bar) в правом нижнем углу
        bar_len = min(scale_bar_px, w - 80)
        p1 = (w - 30 - bar_len, h - 35)
        p2 = (w - 30, h - 35)

        # Тёмная подложка под шкалу
        cv2.rectangle(annotated, (p1[0] - 10, p1[1] - 25), (p2[0] + 10, p2[1] + 20), (15, 20, 28), -1)
        cv2.rectangle(annotated, (p1[0] - 10, p1[1] - 25), (p2[0] + 10, p2[1] + 20), (50, 70, 90), 1)

        # Линия шкалы с засечками
        cv2.line(annotated, p1, p2, (255, 255, 255), 3)
        cv2.line(annotated, (p1[0], p1[1] - 6), (p1[0], p1[1] + 6), (255, 255, 255), 2)
        cv2.line(annotated, (p2[0], p2[1] - 6), (p2[0], p2[1] + 6), (255, 255, 255), 2)

        label = f"{scale_bar_m} m"
        (tw, _), _ = cv2.getTextSize(label, font, 0.48, 1)
        tx = p1[0] + (bar_len - tw) // 2
        cv2.putText(annotated, label, (tx, p1[1] - 10), font, 0.48, (255, 255, 255), 1, cv2.LINE_AA)

        return annotated
