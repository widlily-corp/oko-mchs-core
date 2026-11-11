"""Мультиклассовый детектор ЧС на классическом компьютерном зрении.

Пайплайн одного кадра:

    BGR → HSV → маски по 4 классам → морфология → контуры →
    геометрические/текстурные фильтры → подавление перекрытий →
    склейка близких пятен → трекинг (N кадров подряд) → Detection

Классы и пороги заданы в :mod:`mchs_vision.config` и повторяют ТЗ хакатона:
FIRE, SMOKE, OIL_SPILL, FLOOD.

Детектор намеренно не зависит от ROS и от глубинных нейросетей: на борту
Jetson Orin Nano такой пайплайн даёт десятки FPS на CPU и остаётся
объяснимым для жюри. Место для замены на YOLO предусмотрено: наружу
отдаётся один и тот же список :class:`Detection`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .config import DetectorConfig, HazardClassSpec

BBox = Tuple[int, int, int, int]


@dataclass
class Detection:
    """Одно обнаруженное пятно ЧС на кадре."""

    hazard_type: str
    label_ru: str
    contour: np.ndarray
    bbox: BBox
    area_px: int
    centroid_px: Tuple[float, float]
    confidence: float
    fill_ratio: float
    mean_hsv: Tuple[float, float, float]
    texture_std: float
    track_id: str = ""
    hits: int = 0
    confirmed: bool = False
    stability: float = 0.0

    @property
    def width_px(self) -> int:
        return self.bbox[2]

    @property
    def height_px(self) -> int:
        return self.bbox[3]


@dataclass
class _Track:
    """Внутреннее состояние трека (не отдаётся наружу)."""

    track_id: str
    hazard_type: str
    bbox: BBox
    centroid_px: Tuple[float, float]
    area_px: int
    hits: int = 1
    missed: int = 0


def _kernel(size: int) -> np.ndarray:
    size = max(1, int(size))
    if size % 2 == 0:
        size += 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))


def _bbox_iou(a: BBox, b: BBox) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1 = max(ax, bx)
    y1 = max(ay, by)
    x2 = min(ax + aw, bx + bw)
    y2 = min(ay + ah, by + bh)
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    if inter <= 0:
        return 0.0
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def _bbox_gap(a: BBox, b: BBox) -> float:
    """Кратчайшее расстояние между прямоугольниками (0, если пересекаются)."""
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    dx = max(0, max(bx - (ax + aw), ax - (bx + bw)))
    dy = max(0, max(by - (ay + ah), ay - (by + bh)))
    return math.hypot(dx, dy)


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


class HazardDetector:
    """Детектор четырёх классов ЧС с трекингом и подтверждением."""

    def __init__(self, config: Optional[DetectorConfig] = None):
        self.config = config or DetectorConfig()
        enabled = set(self.config.enabled)
        self.specs: List[HazardClassSpec] = [
            spec for spec in self.config.classes if spec.name in enabled
        ]
        if not self.specs:
            raise ValueError("Не включено ни одного класса ЧС")

        self._tracks: Dict[str, _Track] = {}
        self._next_track = {spec.name: 1 for spec in self.specs}
        self._frame_index = 0

    # ------------------------------------------------------------------
    # Публичный API
    # ------------------------------------------------------------------
    def detect(self, frame_bgr: np.ndarray, update_tracks: bool = True
               ) -> List[Detection]:
        """Найти все подтверждённые ЧС на кадре BGR."""
        if frame_bgr is None or frame_bgr.size == 0:
            return []

        height, width = frame_bgr.shape[:2]
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.config.texture_blur_ksize,
                                         self.config.texture_blur_ksize), 0)
        laplacian = cv2.Laplacian(blurred, cv2.CV_32F)

        claimed = np.zeros((height, width), dtype=np.uint8)
        detections: List[Detection] = []

        for spec in self.specs:
            mask = self._class_mask(hsv, spec)
            if np.any(claimed):
                mask[claimed > 0] = 0
            detections.extend(
                self._detect_class(frame_bgr, hsv, laplacian, mask, claimed,
                                   spec, width, height)
            )

        if update_tracks:
            detections = self._update_tracks(detections)
        self._frame_index += 1
        return detections

    def class_masks(self, frame_bgr: np.ndarray) -> Dict[str, np.ndarray]:
        """Отладочные маски классов после морфологии (без подавления)."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        return {spec.name: self._class_mask(hsv, spec) for spec in self.specs}

    def reset(self) -> None:
        self._tracks.clear()
        self._next_track = {spec.name: 1 for spec in self.specs}
        self._frame_index = 0

    @property
    def active_tracks(self) -> Dict[str, _Track]:
        """Копия активных треков (для тестов и диагностики)."""
        return dict(self._tracks)

    # ------------------------------------------------------------------
    # Сегментация и фильтрация
    # ------------------------------------------------------------------
    def _class_mask(self, hsv: np.ndarray, spec: HazardClassSpec) -> np.ndarray:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in spec.hsv_ranges:
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(hsv, np.asarray(lower, np.uint8),
                            np.asarray(upper, np.uint8)),
            )

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _kernel(spec.open_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _kernel(spec.close_kernel))
        return mask

    def _detect_class(self, frame_bgr: np.ndarray, hsv: np.ndarray,
                      laplacian: np.ndarray, mask: np.ndarray,
                      claimed: np.ndarray, spec: HazardClassSpec,
                      width: int, height: int) -> List[Detection]:
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        frame_area = float(width * height)
        min_area = float(spec.min_area_px)
        if self.config.scale_min_area_with_resolution:
            min_area *= frame_area / float(self.config.reference_frame_pixels)
        candidates: List[Dict] = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            if area > self.config.max_area_ratio * frame_area:
                continue

            x, y, bw, bh = cv2.boundingRect(contour)
            if bw <= 0 or bh <= 0:
                continue
            if max(bw, bh) / float(min(bw, bh)) > self.config.max_aspect_ratio:
                continue

            fill = area / float(bw * bh)
            if fill < self.config.min_fill_ratio:
                continue
            if self._border_touches((x, y, bw, bh), width, height) \
                    > self.config.border_touch_limit:
                continue

            roi_mask = mask[y:y + bh, x:x + bw]
            pixel_count = int(cv2.countNonZero(roi_mask))
            if pixel_count < min_area:
                continue

            mean_hsv = cv2.mean(hsv[y:y + bh, x:x + bw], mask=roi_mask)[:3]
            lap_roi = laplacian[y:y + bh, x:x + bw]
            texture_std = float(lap_roi[roi_mask > 0].std())

            if texture_std > spec.max_texture_std:
                continue

            purity = self._color_purity(spec.name, mean_hsv)
            if purity < 0.15:
                continue

            moments = cv2.moments(contour)
            if abs(moments["m00"]) > 1e-9:
                cx = moments["m10"] / moments["m00"]
                cy = moments["m01"] / moments["m00"]
            else:
                cx, cy = x + bw / 2.0, y + bh / 2.0

            candidates.append({
                "contour": contour,
                "bbox": (x, y, bw, bh),
                "area": area,
                "pixel_count": pixel_count,
                "centroid": (float(cx), float(cy)),
                "fill": fill,
                "mean_hsv": (float(mean_hsv[0]), float(mean_hsv[1]),
                             float(mean_hsv[2])),
                "texture": texture_std,
                "purity": purity,
            })

        candidates = self._merge_candidates(candidates, hsv, laplacian, mask)

        # Сортируем по «весу» и оставляем не больше N пятен на класс.
        candidates.sort(key=lambda c: c["area"] * c["purity"], reverse=True)
        candidates = candidates[:self.config.max_detections_per_class]

        results: List[Detection] = []
        for cand in candidates:
            area = float(cand["area"])
            fill = float(cand["fill"])
            purity = float(cand["purity"])
            confidence = self._confidence(spec, area, fill, purity, stability=0.2)
            if confidence < self.config.min_confidence:
                continue

            detection = Detection(
                hazard_type=spec.name,
                label_ru=spec.label_ru,
                contour=cand["contour"],
                bbox=cand["bbox"],
                area_px=int(cand["pixel_count"]),
                centroid_px=cand["centroid"],
                confidence=confidence,
                fill_ratio=fill,
                mean_hsv=cand["mean_hsv"],
                texture_std=float(cand["texture"]),
            )
            results.append(detection)

            # Помечаем пиксели как «занятые», чтобы дым не перекрывал огонь.
            cv2.drawContours(claimed, [cand["contour"]], -1, 255, -1)
            cv2.dilate(claimed, _kernel(5), dst=claimed)

        return results

    def _merge_candidates(self, candidates: List[Dict], hsv: np.ndarray,
                          laplacian: np.ndarray, mask: np.ndarray) -> List[Dict]:
        """Склеить близкие пятна одного класса в один очаг."""
        if len(candidates) <= 1:
            return candidates

        groups: List[List[Dict]] = []
        for cand in candidates:
            placed = False
            for group in groups:
                if any(_bbox_gap(cand["bbox"], other["bbox"])
                       <= self.config.merge_gap_px for other in group):
                    group.append(cand)
                    placed = True
                    break
            if not placed:
                groups.append([cand])

        merged: List[Dict] = []
        for group in groups:
            if len(group) == 1:
                merged.append(group[0])
                continue

            union = np.zeros(mask.shape, dtype=np.uint8)
            for cand in group:
                cv2.drawContours(union, [cand["contour"]], -1, 255, -1)

            union = cv2.morphologyEx(union, cv2.MORPH_CLOSE, _kernel(5))
            contours, _ = cv2.findContours(
                union, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                merged.extend(group)
                continue
            contour = max(contours, key=cv2.contourArea)
            x, y, bw, bh = cv2.boundingRect(contour)
            if bw <= 0 or bh <= 0:
                merged.extend(group)
                continue

            roi = union[y:y + bh, x:x + bw]
            pixel_count = int(cv2.countNonZero(roi))
            mean_hsv = cv2.mean(hsv[y:y + bh, x:x + bw], mask=roi)[:3]
            texture = float(laplacian[y:y + bh, x:x + bw][roi > 0].std())
            moments = cv2.moments(contour)
            if abs(moments["m00"]) > 1e-9:
                cx = moments["m10"] / moments["m00"]
                cy = moments["m01"] / moments["m00"]
            else:
                cx, cy = x + bw / 2.0, y + bh / 2.0

            # purity пересчитывается вызывающей стороной по имени класса;
            # здесь сохраняем средневзвешенную чистоту группы.
            weights = [max(c["area"], 1.0) for c in group]
            purity = sum(c["purity"] * w for c, w in zip(group, weights)) / sum(weights)
            merged.append({
                "contour": contour,
                "bbox": (x, y, bw, bh),
                "area": float(cv2.contourArea(contour)),
                "pixel_count": pixel_count,
                "centroid": (float(cx), float(cy)),
                "fill": float(cv2.contourArea(contour)) / float(bw * bh),
                "mean_hsv": (float(mean_hsv[0]), float(mean_hsv[1]),
                             float(mean_hsv[2])),
                "texture": texture,
                "purity": purity,
            })
        return merged

    @staticmethod
    def _border_touches(bbox: BBox, width: int, height: int) -> int:
        x, y, bw, bh = bbox
        touches = 0
        if x <= 1:
            touches += 1
        if y <= 1:
            touches += 1
        if x + bw >= width - 1:
            touches += 1
        if y + bh >= height - 1:
            touches += 1
        return touches

    @staticmethod
    def _color_purity(name: str, mean_hsv: Sequence[float]) -> float:
        h, s, v = float(mean_hsv[0]), float(mean_hsv[1]), float(mean_hsv[2])
        if name == "FIRE":
            return _clamp01(s / 200.0) * _clamp01(v / 220.0)
        if name == "SMOKE":
            return _clamp01(1.0 - s / 60.0) * _clamp01((v - 140.0) / 80.0)
        if name == "OIL_SPILL":
            return _clamp01(1.0 - abs(v - 40.0) / 35.0) * _clamp01(1.0 - s / 90.0)
        if name == "FLOOD":
            hue_score = math.exp(-((h - 110.0) / 18.0) ** 2)
            return _clamp01(s / 180.0) * hue_score
        return 0.5

    def _confidence(self, spec: HazardClassSpec, area_px: float, fill: float,
                    purity: float, stability: float) -> float:
        size_score = _clamp01(area_px / (3.0 * spec.min_area_px))
        fill_score = _clamp01(fill / 0.75)
        value = (
            0.30 * size_score
            + 0.25 * fill_score
            + 0.30 * _clamp01(purity)
            + 0.15 * _clamp01(stability)
        )
        return round(_clamp01(value), 3)

    # ------------------------------------------------------------------
    # Трекинг
    # ------------------------------------------------------------------
    def _update_tracks(self, detections: List[Detection]) -> List[Detection]:
        for track in self._tracks.values():
            track.missed += 1

        used_tracks = set()
        for detection in detections:
            best: Optional[_Track] = None
            best_score = -1.0

            for track in self._tracks.values():
                if track.track_id in used_tracks:
                    continue
                if track.hazard_type != detection.hazard_type:
                    continue

                iou = _bbox_iou(track.bbox, detection.bbox)
                distance = math.hypot(
                    track.centroid_px[0] - detection.centroid_px[0],
                    track.centroid_px[1] - detection.centroid_px[1],
                )
                if iou < self.config.track_match_iou \
                        and distance > self.config.track_match_distance_px:
                    continue

                score = iou + max(
                    0.0, 1.0 - distance / self.config.track_match_distance_px
                )
                if score > best_score:
                    best_score = score
                    best = track

            if best is None:
                track_id = f"TRK-{detection.hazard_type}-" \
                           f"{self._next_track[detection.hazard_type]:04d}"
                self._next_track[detection.hazard_type] += 1
                track = _Track(
                    track_id=track_id,
                    hazard_type=detection.hazard_type,
                    bbox=detection.bbox,
                    centroid_px=detection.centroid_px,
                    area_px=detection.area_px,
                )
                self._tracks[track_id] = track
            else:
                track = best
                track.hits += 1
                track.missed = 0
                alpha = self.config.smoothing_alpha
                track.centroid_px = (
                    alpha * detection.centroid_px[0] + (1 - alpha) * track.centroid_px[0],
                    alpha * detection.centroid_px[1] + (1 - alpha) * track.centroid_px[1],
                )
                track.area_px = int(alpha * detection.area_px
                                    + (1 - alpha) * track.area_px)
                track.bbox = detection.bbox

            used_tracks.add(track.track_id)
            detection.track_id = track.track_id
            detection.hits = track.hits
            detection.stability = _clamp01(track.hits / 6.0)
            detection.confidence = round(
                min(0.99, detection.confidence + 0.10 * detection.stability), 3
            )
            detection.confirmed = (
                track.hits >= self.config.track_min_hits
                and detection.confidence >= self.config.min_confidence
            )

        # Удаляем треки, которые давно не подтверждались.
        self._tracks = {
            track_id: track
            for track_id, track in self._tracks.items()
            if track.missed <= self.config.track_max_missed
        }

        return [det for det in detections if det.confirmed]
