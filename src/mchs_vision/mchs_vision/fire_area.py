"""Калиброванный расчёт площади возгорания и шлейфа задымления по аэрофотосъёмке.

Модуль выполняет:
1. Цветовую и текстурную сегментацию пламени (FLAME) и дыма (SMOKE) в HSV.
2. Фильтрацию контуров: площадь, заполненность bbox, solidity, выпуклость,
   прямоугольность, границы кадра и СКО оттенка. Без этого оранжевое кольцо
   вертодрома, белые контейнеры и крыши превращаются в «пожар».
3. Проекцию контуров на плоскость Z = 0 с учётом высоты, рыскания и наклона
   камеры; площадь в м² и га.
4. Тактический оверлей: заливка, контуры, GSD и масштабная линейка.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from .config import CameraConfig, FIRE, SMOKE
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

    # Калиброванные диапазоны HSV для открытого огня (Gazebo и реальные кадры).
    FLAME_HSV_LOW_1 = np.array([5, 120, 160], dtype=np.uint8)
    FLAME_HSV_HIGH_1 = np.array([28, 255, 255], dtype=np.uint8)
    FLAME_HSV_LOW_2 = np.array([0, 140, 160], dtype=np.uint8)
    FLAME_HSV_HIGH_2 = np.array([6, 255, 255], dtype=np.uint8)
    CORE_HSV_LOW = np.array([0, 0, 240], dtype=np.uint8)
    CORE_HSV_HIGH = np.array([180, 70, 255], dtype=np.uint8)

    # Дым: серовато-белые полупрозрачные области.
    SMOKE_HSV_LOW = np.array([0, 0, 140], dtype=np.uint8)
    SMOKE_HSV_HIGH = np.array([180, 65, 230], dtype=np.uint8)

    # Дополнительные ограничения, которых нет в общем HazardClassSpec.
    SMOKE_MAX_RECTANGULARITY = 0.86
    SMOKE_MAX_FILL_RATIO = 0.88
    SMOKE_MIN_HUE_STD = 0.0
    SMOKE_MAX_MEAN_SATURATION = 20.0
    FIRE_MAX_MEAN_VALUE = 250.0

    def __init__(self, camera: Optional[CameraConfig] = None):
        self.camera = camera or CameraConfig(
            width=1280,
            height=720,
            hfov_deg=80.0,
            pitch_down_deg=45.0,
        )
        self.projector = GroundProjector(self.camera)

    # ------------------------------------------------------------------
    # Геометрия / сегментация
    # ------------------------------------------------------------------
    def compute_gsd(self, altitude_m: float) -> Tuple[float, float]:
        """GSD для надирной съёмки (м/пиксель)."""
        hfov_rad = math.radians(self.camera.hfov_deg)
        vfov_rad = math.radians(self.camera.vfov_deg)
        ground_w = 2.0 * altitude_m * math.tan(hfov_rad / 2.0)
        ground_h = 2.0 * altitude_m * math.tan(vfov_rad / 2.0)
        return ground_w / self.camera.width, ground_h / self.camera.height

    def segment_flame_and_smoke(
        self, image_bgr: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Сегментация масок огня и дыма."""
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self.FLAME_HSV_LOW_1, self.FLAME_HSV_HIGH_1)
        mask2 = cv2.inRange(hsv, self.FLAME_HSV_LOW_2, self.FLAME_HSV_HIGH_2)
        mask_core = cv2.inRange(hsv, self.CORE_HSV_LOW, self.CORE_HSV_HIGH)
        flame_mask = cv2.bitwise_or(mask1, mask2)
        flame_mask = cv2.bitwise_or(flame_mask, mask_core)

        kernel_flame = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (FIRE.open_kernel, FIRE.open_kernel)
        )
        flame_mask = cv2.morphologyEx(flame_mask, cv2.MORPH_OPEN, kernel_flame)
        flame_mask = cv2.morphologyEx(
            flame_mask, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                      (FIRE.close_kernel, FIRE.close_kernel)),
        )

        smoke_raw = cv2.inRange(hsv, self.SMOKE_HSV_LOW, self.SMOKE_HSV_HIGH)
        smoke_mask = cv2.bitwise_and(smoke_raw, cv2.bitwise_not(flame_mask))
        kernel_smoke = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (SMOKE.open_kernel, SMOKE.open_kernel)
        )
        smoke_mask = cv2.morphologyEx(smoke_mask, cv2.MORPH_OPEN, kernel_smoke)
        smoke_mask = cv2.morphologyEx(
            smoke_mask, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                      (SMOKE.close_kernel, SMOKE.close_kernel)),
        )
        return flame_mask, smoke_mask

    # ------------------------------------------------------------------
    # Фильтрация контуров
    # ------------------------------------------------------------------
    @staticmethod
    def _shape_features(
        contour: np.ndarray, pixel_count: int
    ) -> Dict[str, float]:
        contour_area = float(cv2.contourArea(contour))
        if contour_area <= 0.0:
            return {"solidity": 0.0, "hull_ratio": 0.0,
                    "rectangularity": 1.0, "contour_area": 0.0}
        hull = cv2.convexHull(contour)
        hull_area = float(cv2.contourArea(hull)) if hull is not None else 0.0
        rect = cv2.minAreaRect(contour)
        rect_area = float(rect[1][0] * rect[1][1])
        return {
            "solidity": min(1.0, pixel_count / contour_area),
            "hull_ratio": contour_area / hull_area if hull_area > 0 else 0.0,
            "rectangularity": contour_area / rect_area if rect_area > 0 else 1.0,
            "contour_area": contour_area,
        }

    def _filter_contours(
        self,
        mask: np.ndarray,
        hsv: np.ndarray,
        laplacian: np.ndarray,
        spec,
        frame_shape: Tuple[int, int],
        max_rectangularity: Optional[float] = None,
        max_fill_ratio: Optional[float] = None,
        min_hue_std: Optional[float] = None,
        max_mean_saturation: Optional[float] = None,
        max_mean_value: Optional[float] = None,
    ) -> List[np.ndarray]:
        """Оставить только компактные непустые очаги, похожие на класс."""
        height, width = frame_shape
        frame_area = float(width * height)
        min_area = float(spec.min_area_px) * frame_area / float(640 * 480)
        max_rect = max_rectangularity if max_rectangularity is not None \
            else spec.max_rectangularity
        max_fill = max_fill_ratio if max_fill_ratio is not None \
            else spec.max_fill_ratio
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        accepted: List[np.ndarray] = []

        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            if bw <= 0 or bh <= 0:
                continue
            if max(bw, bh) / float(min(bw, bh)) > 10.0:
                continue
            if x <= 1 or y <= 1 or x + bw >= width - 1 or y + bh >= height - 1:
                continue

            roi = mask[y:y + bh, x:x + bw]
            pixel_count = int(cv2.countNonZero(roi))
            if pixel_count < min_area:
                continue
            if pixel_count > 0.50 * frame_area:
                continue
            fill = pixel_count / float(bw * bh)
            if fill < 0.18 or fill > max_fill:
                continue

            shape = self._shape_features(contour, pixel_count)
            if shape["solidity"] < spec.min_solidity_ratio:
                continue
            if shape["hull_ratio"] < spec.min_hull_ratio:
                continue
            if shape["rectangularity"] > max_rect:
                continue

            hsv_pixels = hsv[y:y + bh, x:x + bw][roi > 0]
            hue_std = float(hsv_pixels[:, 0].std())
            min_hue = spec.min_hue_std if min_hue_std is None else min_hue_std
            if hue_std < min_hue:
                continue
            mean_s = float(hsv_pixels[:, 1].mean())
            mean_v = float(hsv_pixels[:, 2].mean())
            max_s = 255.0 if max_mean_saturation is None \
                else max_mean_saturation
            max_v = spec.max_mean_value if max_mean_value is None \
                else max_mean_value
            if mean_s > max_s or mean_v > max_v:
                continue
            if mean_s < spec.min_mean_saturation:
                continue

            texture = float(laplacian[y:y + bh, x:x + bw][roi > 0].std())
            if texture > spec.max_texture_std:
                continue
            accepted.append(contour)

        return accepted

    # ------------------------------------------------------------------
    # Расчёт
    # ------------------------------------------------------------------
    def estimate(
        self,
        image_bgr: np.ndarray,
        pose: Optional[Pose] = None,
        altitude_m: Optional[float] = None,
        annotate: bool = True,
    ) -> FireAreaResult:
        """Полный расчёт площади по фотоснимку с геометрической привязкой."""
        if image_bgr is None or image_bgr.size == 0:
            raise ValueError("Пустое изображение")
        if pose is None:
            alt = altitude_m if altitude_m is not None else 22.0
            pose = Pose(x_m=0.0, y_m=0.0, z_m=max(1.0, alt), yaw_rad=0.0)

        h, w = image_bgr.shape[:2]
        if w != self.camera.width or h != self.camera.height:
            self.camera = CameraConfig(
                width=w,
                height=h,
                hfov_deg=self.camera.hfov_deg,
                pitch_down_deg=self.camera.pitch_down_deg,
                mount_forward_m=self.camera.mount_forward_m,
                mount_right_m=self.camera.mount_right_m,
                mount_up_m=self.camera.mount_up_m,
                min_range_m=self.camera.min_range_m,
                max_range_m=self.camera.max_range_m,
            )
            self.projector = GroundProjector(self.camera)

        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        laplacian = cv2.Laplacian(
            cv2.GaussianBlur(gray, (5, 5), 0), cv2.CV_32F
        )
        flame_mask, smoke_mask = self.segment_flame_and_smoke(image_bgr)

        flame_contours = self._filter_contours(
            flame_mask, hsv, laplacian, FIRE, (h, w),
            max_mean_value=self.FIRE_MAX_MEAN_VALUE,
        )
        smoke_contours = self._filter_contours(
            smoke_mask, hsv, laplacian, SMOKE, (h, w),
            max_rectangularity=self.SMOKE_MAX_RECTANGULARITY,
            max_fill_ratio=self.SMOKE_MAX_FILL_RATIO,
            min_hue_std=self.SMOKE_MIN_HUE_STD,
            max_mean_saturation=self.SMOKE_MAX_MEAN_SATURATION,
        )

        flame_area_px = 0
        smoke_area_px = 0
        flame_area_m2 = 0.0
        smoke_area_m2 = 0.0
        perimeter_m = 0.0
        centroids: List[Tuple[float, float]] = []
        valid_flame: List[np.ndarray] = []
        valid_smoke: List[np.ndarray] = []

        for contour in flame_contours:
            metrics = self.projector.region_metrics(contour, pose)
            if metrics is not None and metrics.area_m2 < FIRE.min_area_m2:
                continue
            if metrics is None:
                gsd_x, gsd_y = self.compute_gsd(pose.z_m)
                area = float(cv2.contourArea(contour)) * gsd_x * gsd_y
                if area < FIRE.min_area_m2:
                    continue
                flame_area_m2 += area
                perimeter_m += float(cv2.arcLength(contour, True)) * gsd_x
            else:
                flame_area_m2 += metrics.area_m2
                polygon = self.projector.contour_to_ground(contour, pose)
                if polygon is not None:
                    perimeter_m += float(cv2.arcLength(
                        polygon.reshape(-1, 1, 2), True
                    ))
                centroids.append(metrics.center_xy)
            valid_flame.append(contour)
            roi = flame_mask[
                cv2.boundingRect(contour)[1]:cv2.boundingRect(contour)[1] + cv2.boundingRect(contour)[3],
                cv2.boundingRect(contour)[0]:cv2.boundingRect(contour)[0] + cv2.boundingRect(contour)[2],
            ]
            flame_area_px += int(cv2.countNonZero(roi))

        for contour in smoke_contours:
            metrics = self.projector.region_metrics(contour, pose)
            if metrics is not None and metrics.area_m2 < SMOKE.min_area_m2:
                continue
            if metrics is None:
                gsd_x, gsd_y = self.compute_gsd(pose.z_m)
                area = float(cv2.contourArea(contour)) * gsd_x * gsd_y
                if area < SMOKE.min_area_m2:
                    continue
                smoke_area_m2 += area
            else:
                smoke_area_m2 += metrics.area_m2
            valid_smoke.append(contour)
            x, y, bw, bh = cv2.boundingRect(contour)
            smoke_area_px += int(cv2.countNonZero(
                smoke_mask[y:y + bh, x:x + bw]
            ))

        flame_contours = valid_flame
        smoke_contours = valid_smoke
        total_area_m2 = flame_area_m2 + smoke_area_m2
        has_fire = bool(flame_contours)
        has_smoke = bool(smoke_contours)

        if centroids:
            ground_cx = float(np.mean([pt[0] for pt in centroids]))
            ground_cy = float(np.mean([pt[1] for pt in centroids]))
        else:
            ground_cx, ground_cy = pose.x_m, pose.y_m

        if flame_contours:
            all_pts = np.vstack(flame_contours)
            moments = cv2.moments(all_pts)
            if abs(moments["m00"]) > 1e-4:
                cx_px = int(moments["m10"] / moments["m00"])
                cy_px = int(moments["m01"] / moments["m00"])
            else:
                cx_px, cy_px = w // 2, h // 2
        else:
            cx_px, cy_px = w // 2, h // 2

        # GSD в центре кадра с учётом наклона, а не только надира.
        gsd_x, _ = self.projector.gsd(pose)
        if not math.isfinite(gsd_x) or gsd_x <= 0:
            gsd_x, _ = self.compute_gsd(pose.z_m)

        desired_px = 140
        available_scales = [5, 10, 20, 50, 100, 200]
        chosen_scale_m = 10
        best_diff = float("inf")
        for scale in available_scales:
            px_len = scale / max(1e-5, gsd_x)
            diff = abs(px_len - desired_px)
            if diff < best_diff:
                best_diff = diff
                chosen_scale_m = scale
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
                gsd_m_per_px=gsd_x,
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

    # ------------------------------------------------------------------
    # Оверлей
    # ------------------------------------------------------------------
    def _render_annotation(
        self,
        image_bgr: np.ndarray,
        flame_contours: List[np.ndarray],
        smoke_contours: List[np.ndarray],
        flame_area_m2: float,
        smoke_area_m2: float,
        altitude_m: float,
        gsd_m_per_px: float,
        scale_bar_m: int,
        scale_bar_px: int,
        centroid_px: Tuple[int, int],
    ) -> np.ndarray:
        annotated = image_bgr.copy()
        h, w = annotated.shape[:2]

        overlay = annotated.copy()
        if smoke_contours:
            cv2.drawContours(overlay, smoke_contours, -1, (160, 160, 180), -1)
        if flame_contours:
            cv2.drawContours(overlay, flame_contours, -1, (0, 90, 255), -1)
        cv2.addWeighted(overlay, 0.35, annotated, 0.65, 0, annotated)

        if smoke_contours:
            cv2.drawContours(annotated, smoke_contours, -1, (200, 200, 220), 2)
        if flame_contours:
            cv2.drawContours(annotated, flame_contours, -1, (0, 60, 255), 2)
            cv2.circle(annotated, centroid_px, 6, (0, 255, 255), -1)
            cv2.circle(annotated, centroid_px, 12, (0, 90, 255), 2)

        card_w, card_h = 360, 110
        cv2.rectangle(annotated, (15, 15), (15 + card_w, 15 + card_h),
                      (15, 20, 28), -1)
        cv2.rectangle(annotated, (15, 15), (15 + card_w, 15 + card_h),
                      (0, 120, 255), 2)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(annotated, "MCHS FIRE RECON // AREA ESTIMATION",
                    (25, 38), font, 0.52, (0, 210, 255), 1, cv2.LINE_AA)
        flame_ha = flame_area_m2 / 10_000.0
        cv2.putText(annotated,
                    f"FLAME AREA: {flame_area_m2:.1f} m2 ({flame_ha:.3f} ha)",
                    (25, 64), font, 0.60, (0, 90, 255), 2, cv2.LINE_AA)
        smoke_ha = smoke_area_m2 / 10_000.0
        cv2.putText(annotated,
                    f"SMOKE PLUME: {smoke_area_m2:.1f} m2 ({smoke_ha:.3f} ha)",
                    (25, 88), font, 0.50, (180, 190, 200), 1, cv2.LINE_AA)
        cv2.putText(annotated,
                    f"ALTITUDE: {altitude_m:.1f} m | GSD: {gsd_m_per_px*100:.1f} cm/px",
                    (25, 112), font, 0.44, (120, 140, 160), 1, cv2.LINE_AA)

        bar_len = min(scale_bar_px, w - 80)
        p1 = (w - 30 - bar_len, h - 35)
        p2 = (w - 30, h - 35)
        cv2.rectangle(annotated, (p1[0] - 10, p1[1] - 25),
                      (p2[0] + 10, p2[1] + 20), (15, 20, 28), -1)
        cv2.rectangle(annotated, (p1[0] - 10, p1[1] - 25),
                      (p2[0] + 10, p2[1] + 20), (50, 70, 90), 1)
        cv2.line(annotated, p1, p2, (255, 255, 255), 3)
        cv2.line(annotated, (p1[0], p1[1] - 6), (p1[0], p1[1] + 6),
                 (255, 255, 255), 2)
        cv2.line(annotated, (p2[0], p2[1] - 6), (p2[0], p2[1] + 6),
                 (255, 255, 255), 2)

        label = f"{scale_bar_m} m"
        (tw, _), _ = cv2.getTextSize(label, font, 0.48, 1)
        tx = p1[0] + (bar_len - tw) // 2
        cv2.putText(annotated, label, (tx, p1[1] - 10), font, 0.48,
                    (255, 255, 255), 1, cv2.LINE_AA)
        return annotated
