"""Тактический HUD аннотированного видеопотока для АРМ оператора.

HUD масштабируется под разрешение кадра: на 640×480 рисуется базовыми
размерами, на 1280×720 — удвоенными, чтобы текст оставался читаемым.
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
        self._ui = 1.0

    # ------------------------------------------------------------------
    # Масштабирование интерфейса
    # ------------------------------------------------------------------
    def _px(self, value: float) -> int:
        """Пиксели с учётом разрешения (база — 640×480)."""
        return max(1, int(round(value * self._ui)))

    def _fs(self, value: float) -> float:
        """Масштаб шрифта с учётом разрешения."""
        return value * self._ui

    def _th(self, value: int) -> int:
        """Толщина линии с учётом разрешения."""
        return max(1, int(round(value * self._ui)))

    def draw(self, frame: np.ndarray, records: List[HazardRecord], pose: Pose,
             fps: float, timestamp: str = "",
             title: str = "MCHS OPTICAL RECON // GAZEBO 11"
             ) -> np.ndarray:
        out = frame.copy()
        height, width = out.shape[:2]
        self._ui = max(0.75, min(3.0, width / 640.0))

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

    def _text(self, image: np.ndarray, text: str, org: Tuple[int, int],
              scale: float = 0.45, color: Tuple[int, int, int] = TEXT_COLOR,
              thickness: int = 1) -> None:
        x, y = org
        cv2.putText(image, text, (x + self._px(1), y + self._px(1)),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0),
                    thickness + self._th(1), cv2.LINE_AA)
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, thickness, cv2.LINE_AA)

    def _draw_top_bar(self, image: np.ndarray, width: int, pose: Pose,
                      fps: float, timestamp: str, title: str) -> None:
        bar_h = self._px(52)
        self._panel(image, (0, 0, width, bar_h), alpha=0.66)
        cv2.line(image, (0, bar_h), (width, bar_h), ACCENT_COLOR, self._th(1))

        self._text(image, title, (self._px(10), self._px(18)),
                   scale=self._fs(0.48), color=ACCENT_COLOR,
                   thickness=self._th(1))
        fps_text = f"{fps:5.1f}" if fps > 0 else "  -- "
        self._text(
            image,
            f"ALT {pose.z_m:5.1f} M   YAW {pose.yaw_deg:05.1f}   FPS {fps_text}",
            (self._px(10), self._px(40)), scale=self._fs(0.42),
            thickness=self._th(1),
        )

        blink = int(time.time() * 2) % 2 == 0
        dot_color = (0, 0, 255) if blink else (0, 0, 110)
        cv2.circle(image, (width - self._px(128), self._px(16)),
                   self._px(5), dot_color, -1)
        self._text(image, "REC", (width - self._px(118), self._px(21)),
                   scale=self._fs(0.42), color=(240, 240, 240),
                   thickness=self._th(1))

        if timestamp:
            self._text(image, timestamp,
                       (width - self._px(118), self._px(42)),
                       scale=self._fs(0.38), color=(180, 190, 200),
                       thickness=self._th(1))

    def _draw_bottom_bar(self, image: np.ndarray, height: int, width: int,
                         records: List[HazardRecord]) -> None:
        bar_h = self._px(34)
        self._panel(image, (0, height - bar_h, width, height), alpha=0.66)
        cv2.line(image, (0, height - bar_h), (width, height - bar_h),
                 ACCENT_COLOR, self._th(1))

        if records:
            primary = max(records,
                          key=lambda r: (THREAT_RANK.get(r.threat_level, 0),
                                         r.smoothed_area_m2))
            color = THREAT_COLORS.get(primary.threat_level, (0, 0, 255))
            text = (f"HAZARD DETECTED: {primary.hazard_type} "
                    f"[{primary.threat_level}]  {primary.smoothed_area_m2:.1f} M2")
            cv2.circle(image, (self._px(14), height - self._px(17)),
                       self._px(5), color, -1)
            self._text(image, text, (self._px(26), height - self._px(11)),
                       scale=self._fs(0.46), color=color,
                       thickness=self._th(1))
        else:
            cv2.circle(image, (self._px(14), height - self._px(17)),
                       self._px(5), (0, 220, 120), -1)
            self._text(image, "STATUS: PATROL / SCANNING",
                       (self._px(26), height - self._px(11)),
                       scale=self._fs(0.46), color=(0, 220, 120),
                       thickness=self._th(1))

        self._text(image, f"TRACKS {len(records)}",
                   (width - self._px(92), height - self._px(11)),
                   scale=self._fs(0.40), color=(190, 200, 210),
                   thickness=self._th(1))

    def _draw_legend(self, image: np.ndarray, height: int,
                     records: List[HazardRecord]) -> None:
        counts = {}
        for record in records:
            counts[record.hazard_type] = counts.get(record.hazard_type, 0) + 1

        names = [("FIRE", "FIRE"), ("SMOKE", "SMOKE"),
                 ("OIL", "OIL_SPILL"), ("FLOOD", "FLOOD")]
        base_y = height - self._px(44)
        x = self._px(10)
        for label, key in names:
            color = self._class_colors.get(key, (200, 200, 200))
            cv2.rectangle(image, (x, base_y - self._px(8)),
                          (x + self._px(8), base_y), color, -1)
            text = f"{label} {counts.get(key, 0)}"
            self._text(image, text, (x + self._px(12), base_y),
                       scale=self._fs(0.34), color=(205, 212, 220),
                       thickness=self._th(1))
            x += self._px(12) + int(6.2 * len(text) * self._ui) + self._px(10)

    def _draw_reticle(self, image: np.ndarray, width: int, height: int) -> None:
        cx, cy = width // 2, height // 2
        near, far, dot = self._px(8), self._px(22), self._px(2)
        cv2.line(image, (cx - far, cy), (cx - near, cy), ACCENT_COLOR,
                 self._th(1))
        cv2.line(image, (cx + near, cy), (cx + far, cy), ACCENT_COLOR,
                 self._th(1))
        cv2.line(image, (cx, cy - far), (cx, cy - near), ACCENT_COLOR,
                 self._th(1))
        cv2.line(image, (cx, cy + near), (cx, cy + far), ACCENT_COLOR,
                 self._th(1))
        cv2.circle(image, (cx, cy), dot, ACCENT_COLOR, -1)

    def _draw_scale_bar(self, image: np.ndarray, width: int, height: int,
                        pose: Pose) -> None:
        try:
            gsd_x, _ = self.projector.gsd(pose)
        except Exception:
            return
        if gsd_x <= 0:
            return

        target_px = 90 * self._ui
        candidates = [2, 5, 10, 20, 50, 100, 200]
        length_m = min(candidates, key=lambda value: abs(value / gsd_x - target_px))
        bar_px = int(round(length_m / gsd_x))
        if bar_px < self._px(12) or bar_px > width * 0.5:
            return

        x2 = width - self._px(14)
        x1 = x2 - bar_px
        y = height - self._px(58)
        cv2.line(image, (x1, y), (x2, y), TEXT_COLOR, self._th(3))
        cv2.line(image, (x1, y - self._px(5)), (x1, y + self._px(5)),
                 TEXT_COLOR, self._th(2))
        cv2.line(image, (x2, y - self._px(5)), (x2, y + self._px(5)),
                 TEXT_COLOR, self._th(2))
        self._text(image, f"{length_m} M", (x1, y - self._px(9)),
                   scale=self._fs(0.36), color=TEXT_COLOR,
                   thickness=self._th(1))

    def _draw_compass(self, image: np.ndarray, width: int, height: int,
                      pose: Pose) -> None:
        cx, cy, radius = width - self._px(34), self._px(88), self._px(18)
        cv2.circle(image, (cx, cy), radius, (210, 220, 230), self._th(1),
                   cv2.LINE_AA)
        dx, dy = self.projector.north_screen_direction(pose)
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            dx, dy, norm = 0.0, -1.0, 1.0
        dx, dy = dx / norm, dy / norm
        tip = (int(cx + dx * (radius - self._px(3))),
               int(cy + dy * (radius - self._px(3))))
        tail = (int(cx - dx * (radius - self._px(7))),
                int(cy - dy * (radius - self._px(7))))
        cv2.arrowedLine(image, tail, tip, (0, 80, 255), self._th(2),
                        tipLength=0.35)
        self._text(image, "N",
                   (cx + int(dx * (radius + self._px(7))) - self._px(3),
                    cy + int(dy * (radius + self._px(7))) + self._px(4)),
                   scale=self._fs(0.38), color=(0, 120, 255),
                   thickness=self._th(1))

    # ------------------------------------------------------------------
    # Рамки детекций
    # ------------------------------------------------------------------
    def _draw_detection(self, image: np.ndarray, record: HazardRecord,
                        pose: Pose, primary: bool) -> None:
        x, y, bw, bh = record.detection.bbox
        color = self._class_colors.get(record.hazard_type, (0, 255, 0))
        thickness = self._th(2 if primary else 1)
        corner = max(self._px(7), min(self._px(18), min(bw, bh) // 4))

        cv2.rectangle(image, (x, y), (x + bw, y + bh), color, thickness)
        for px, py, sx, sy in (
            (x, y, 1, 1),
            (x + bw, y, -1, 1),
            (x, y + bh, 1, -1),
            (x + bw, y + bh, -1, -1),
        ):
            cv2.line(image, (px, py), (px + sx * corner, py), color,
                     self._th(3))
            cv2.line(image, (px, py), (px, py + sy * corner), color,
                     self._th(3))

        label = (f"{record.hazard_type} {record.smoothed_area_m2:5.1f} M2 "
                 f"{record.confidence * 100:3.0f}%")
        scale = self._fs(0.42)
        (text_w, text_h), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, scale, self._th(1)
        )
        label_y = max(0, y - text_h - self._px(7))
        label_x = min(max(self._px(2), x),
                      image.shape[1] - text_w - self._px(10))
        cv2.rectangle(image, (label_x, label_y),
                      (label_x + text_w + self._px(8),
                       label_y + text_h + self._px(6)),
                      (10, 12, 16), -1)
        cv2.rectangle(image, (label_x, label_y),
                      (label_x + text_w + self._px(8),
                       label_y + text_h + self._px(6)),
                      color, self._th(1))
        self._text(image, label,
                   (label_x + self._px(4), label_y + text_h + self._px(2)),
                   scale=scale, color=color, thickness=self._th(1))

        threat_color = THREAT_COLORS.get(record.threat_level, (255, 255, 255))
        line2 = (f"[{record.threat_level}]  X {record.abs_xy[0]:+.1f}  "
                 f"Y {record.abs_xy[1]:+.1f}  {record.metrics.distance_m:.1f} M")
        line2_y = min(image.shape[0] - self._px(40), y + bh + self._px(14))
        (w2, h2), _ = cv2.getTextSize(line2, cv2.FONT_HERSHEY_SIMPLEX,
                                      self._fs(0.36), self._th(1))
        line2_x = min(max(self._px(2), x),
                      image.shape[1] - w2 - self._px(10))
        cv2.rectangle(image, (line2_x, line2_y - h2 - self._px(3)),
                      (line2_x + w2 + self._px(8), line2_y + self._px(3)),
                      (10, 12, 16), -1)
        self._text(image, line2, (line2_x + self._px(4), line2_y),
                   scale=self._fs(0.36), color=threat_color,
                   thickness=self._th(1))

        cx, cy = record.detection.centroid_px
        cv2.drawMarker(image, (int(cx), int(cy)), color,
                       markerType=cv2.MARKER_CROSS,
                       markerSize=self._px(10),
                       thickness=self._th(1))
