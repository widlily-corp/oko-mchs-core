"""Тактический HUD аннотированного видеопотока для АРМ оператора.

HUD рисуется на кадре 640×480 и масштабируется вместе с изображением.
Все надписи — латиницей: стандартные шрифты OpenCV не поддерживают
кириллицу, а подключать FreeType ради демо-оверлея избыточно.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .config import HazardClassSpec
from .events import HazardRecord, THREAT_RANK
from .geometry import GroundProjector, Pose

THREAT_COLORS = {
    "ADVISORY": (0, 220, 255),   # жёлтый
    "WARNING": (0, 140, 255),    # оранжевый
    "CRITICAL": (0, 0, 255),     # красный
}
PANEL_COLOR = (18, 20, 24)
TEXT_COLOR = (225, 232, 240)
ACCENT_COLOR = (0, 229, 255)


class TacticalHUD:
    def __init__(self, projector: GroundProjector,
                 classes: Optional[Tuple[HazardClassSpec, ...]] = None):
        self.projector = projector
        self._class_colors = {
            spec.name: spec.color_bgr for spec in (classes or ())
        }

    def draw(self, frame: np.ndarray, records: List[HazardRecord], pose: Pose,
             fps: float, timestamp: str = "",
             title: str = "MCHS OPTICAL RECON // GAZEBO 11"
             ) -> np.ndarray:
        out = frame.copy()
        height, width = out.shape[:2]

        self._draw_top_bar(out, width, pose, fps, timestamp, title)
        self._draw_bottom_bar(out, height, width, records)
        self._draw_legend(out, height, records)
        self._draw_reticle(out, width, height)
        self._draw_scale_bar(out, width, height, pose)
        self._draw_compass(out, width, height, pose)

        ordered = sorted(
            records,
            key=lambda rec: (THREAT_RANK.get(rec.threat_level, 0),
                             rec.smoothed_area_m2),
            reverse=True,
        )
        for index, record in enumerate(ordered):
            self._draw_detection(out, record, pose, primary=(index == 0))

        return out

    # ------------------------------------------------------------------
    # Панели
    # ------------------------------------------------------------------
    @staticmethod
    def _panel(image: np.ndarray, rect: Tuple[int, int, int, int],
               alpha: float = 0.62) -> None:
        x1, y1, x2, y2 = rect
        x1, y1 = max(0, x1), max(0, y1)
        x2 = min(image.shape[1], x2)
        y2 = min(image.shape[0], y2)
        if x2 <= x1 or y2 <= y1:
            return
        overlay = image.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), PANEL_COLOR, -1)
        cv2.addWeighted(overlay, alpha, image, 1.0 - alpha, 0.0, image)

    @staticmethod
    def _text(image: np.ndarray, text: str, org: Tuple[int, int],
              scale: float = 0.45, color: Tuple[int, int, int] = TEXT_COLOR,
              thickness: int = 1) -> None:
        x, y = org
        cv2.putText(image, text, (x + 1, y + 1), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, (0, 0, 0), thickness + 1, cv2.LINE_AA)
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, thickness, cv2.LINE_AA)

    def _draw_top_bar(self, image: np.ndarray, width: int, pose: Pose,
                      fps: float, timestamp: str, title: str) -> None:
        self._panel(image, (0, 0, width, 52), alpha=0.66)
        cv2.line(image, (0, 52), (width, 52), ACCENT_COLOR, 1)

        self._text(image, f"{title}", (10, 18),
                   scale=0.48, color=ACCENT_COLOR, thickness=1)
        fps_text = f"{fps:5.1f}" if fps > 0 else "  -- "
        self._text(
            image,
            f"ALT {pose.z_m:5.1f} M   YAW {pose.yaw_deg:05.1f}   FPS {fps_text}",
            (10, 40), scale=0.42,
        )

        blink = int(time.time() * 2) % 2 == 0
        dot_color = (0, 0, 255) if blink else (0, 0, 110)
        cv2.circle(image, (width - 128, 16), 5, dot_color, -1)
        self._text(image, "REC", (width - 118, 21), scale=0.42,
                   color=(240, 240, 240))

        if timestamp:
            self._text(image, timestamp, (width - 118, 42), scale=0.38,
                       color=(180, 190, 200))

    def _draw_bottom_bar(self, image: np.ndarray, height: int, width: int,
                         records: List[HazardRecord]) -> None:
        self._panel(image, (0, height - 34, width, height), alpha=0.66)
        cv2.line(image, (0, height - 34), (width, height - 34),
                 ACCENT_COLOR, 1)

        if records:
            primary = max(records,
                          key=lambda r: (THREAT_RANK.get(r.threat_level, 0),
                                         r.smoothed_area_m2))
            color = THREAT_COLORS.get(primary.threat_level, (0, 0, 255))
            text = (f"HAZARD DETECTED: {primary.hazard_type} "
                    f"[{primary.threat_level}]  {primary.smoothed_area_m2:.1f} M2")
            cv2.circle(image, (14, height - 17), 5, color, -1)
            self._text(image, text, (26, height - 11), scale=0.46,
                       color=color, thickness=1)
        else:
            cv2.circle(image, (14, height - 17), 5, (0, 220, 120), -1)
            self._text(image, "STATUS: PATROL / SCANNING", (26, height - 11),
                       scale=0.46, color=(0, 220, 120))

        self._text(image, f"TRACKS {len(records)}", (width - 92, height - 11),
                   scale=0.40, color=(190, 200, 210))

    def _draw_legend(self, image: np.ndarray, height: int, records: List[HazardRecord]
                     ) -> None:
        counts = {}
        for record in records:
            counts[record.hazard_type] = counts.get(record.hazard_type, 0) + 1

        names = [("FIRE", "FIRE"), ("SMOKE", "SMOKE"),
                 ("OIL", "OIL_SPILL"), ("FLOOD", "FLOOD")]
        base_y = height - 44
        x = 10
        for label, key in names:
            color = self._class_colors.get(key, (200, 200, 200))
            cv2.rectangle(image, (x, base_y - 8), (x + 8, base_y), color, -1)
            text = f"{label} {counts.get(key, 0)}"
            self._text(image, text, (x + 12, base_y), scale=0.34,
                       color=(205, 212, 220))
            x += 12 + int(6.2 * len(text)) + 10

    def _draw_reticle(self, image: np.ndarray, width: int, height: int) -> None:
        cx, cy = width // 2, height // 2
        cv2.line(image, (cx - 22, cy), (cx - 8, cy), ACCENT_COLOR, 1)
        cv2.line(image, (cx + 8, cy), (cx + 22, cy), ACCENT_COLOR, 1)
        cv2.line(image, (cx, cy - 22), (cx, cy - 8), ACCENT_COLOR, 1)
        cv2.line(image, (cx, cy + 8), (cx, cy + 22), ACCENT_COLOR, 1)
        cv2.circle(image, (cx, cy), 2, ACCENT_COLOR, -1)

    def _draw_scale_bar(self, image: np.ndarray, width: int, height: int,
                        pose: Pose) -> None:
        try:
            gsd_x, _ = self.projector.gsd(pose)
        except Exception:
            return
        if gsd_x <= 0:
            return

        target_px = 90
        candidates = [2, 5, 10, 20, 50, 100, 200]
        length_m = min(candidates, key=lambda value: abs(value / gsd_x - target_px))
        bar_px = int(round(length_m / gsd_x))
        if bar_px < 12 or bar_px > width * 0.5:
            return

        x2 = width - 14
        x1 = x2 - bar_px
        y = height - 58
        cv2.line(image, (x1, y), (x2, y), TEXT_COLOR, 3)
        cv2.line(image, (x1, y - 5), (x1, y + 5), TEXT_COLOR, 2)
        cv2.line(image, (x2, y - 5), (x2, y + 5), TEXT_COLOR, 2)
        self._text(image, f"{length_m} M", (x1, y - 9), scale=0.36,
                   color=TEXT_COLOR)

    def _draw_compass(self, image: np.ndarray, width: int, height: int,
                      pose: Pose) -> None:
        cx, cy, radius = width - 34, 88, 18
        cv2.circle(image, (cx, cy), radius, (210, 220, 230), 1, cv2.LINE_AA)
        dx, dy = self.projector.north_screen_direction(pose)
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            dx, dy, norm = 0.0, -1.0, 1.0
        dx, dy = dx / norm, dy / norm
        tip = (int(cx + dx * (radius - 3)), int(cy + dy * (radius - 3)))
        tail = (int(cx - dx * (radius - 7)), int(cy - dy * (radius - 7)))
        cv2.arrowedLine(image, tail, tip, (0, 80, 255), 2, tipLength=0.35)
        self._text(image, "N", (cx + int(dx * (radius + 7)) - 3,
                                cy + int(dy * (radius + 7)) + 4), scale=0.38,
                   color=(0, 120, 255))

    # ------------------------------------------------------------------
    # Рамки детекций
    # ------------------------------------------------------------------
    def _draw_detection(self, image: np.ndarray, record: HazardRecord,
                        pose: Pose, primary: bool) -> None:
        x, y, bw, bh = record.detection.bbox
        color = self._class_colors.get(record.hazard_type, (0, 255, 0))
        thickness = 2 if primary else 1
        corner = max(7, min(18, min(bw, bh) // 4))

        cv2.rectangle(image, (x, y), (x + bw, y + bh), color, thickness)
        for px, py, sx, sy in (
            (x, y, 1, 1),
            (x + bw, y, -1, 1),
            (x, y + bh, 1, -1),
            (x + bw, y + bh, -1, -1),
        ):
            cv2.line(image, (px, py), (px + sx * corner, py), color, 3)
            cv2.line(image, (px, py), (px, py + sy * corner), color, 3)

        label = (f"{record.hazard_type} {record.smoothed_area_m2:5.1f} M2 "
                 f"{record.confidence * 100:3.0f}%")
        scale = 0.42
        (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                              scale, 1)
        label_y = max(0, y - text_h - 7)
        label_x = min(max(2, x), image.shape[1] - text_w - 10)
        cv2.rectangle(image, (label_x, label_y),
                      (label_x + text_w + 8, label_y + text_h + 6),
                      (10, 12, 16), -1)
        cv2.rectangle(image, (label_x, label_y),
                      (label_x + text_w + 8, label_y + text_h + 6),
                      color, 1)
        self._text(image, label, (label_x + 4, label_y + text_h + 2),
                   scale=scale, color=color)

        threat_color = THREAT_COLORS.get(record.threat_level, (255, 255, 255))
        line2 = (f"[{record.threat_level}]  X {record.abs_xy[0]:+.1f}  "
                 f"Y {record.abs_xy[1]:+.1f}  {record.metrics.distance_m:.1f} M")
        line2_y = min(image.shape[0] - 40, y + bh + 14)
        (w2, h2), _ = cv2.getTextSize(line2, cv2.FONT_HERSHEY_SIMPLEX, 0.36, 1)
        line2_x = min(max(2, x), image.shape[1] - w2 - 10)
        cv2.rectangle(image, (line2_x, line2_y - h2 - 3),
                      (line2_x + w2 + 8, line2_y + 3), (10, 12, 16), -1)
        self._text(image, line2, (line2_x + 4, line2_y), scale=0.36,
                   color=threat_color)

        cx, cy = record.detection.centroid_px
        cv2.drawMarker(image, (int(cx), int(cy)), color,
                       markerType=cv2.MARKER_CROSS, markerSize=10,
                       thickness=1)
