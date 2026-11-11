"""Компилятор тактических видеорапортов и видеоотчётов миссий «Око МЧС».

Модуль создаёт готовый видеоролик MP4/AVI с каскадной защитой от сбоев кодеков:
1. Вступительная тактическая заставка (Title Card) с логотипом МЧС, номером борта и временем.
2. Последовательность ключевых кадров миссии и тактических съёмок с бортовой камеры.
3. Карточки обнаруженных инцидентов (очаг пожара, пострадавший, разлив нефти).
4. Итоговая сводка миссии (длительность, площадь, найденные угрозы).
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


@dataclass
class VideoReportConfig:
    width: int = 1280
    height: int = 720
    fps: float = 12.0
    mission_title: str = "ОКО МЧС // ОПЕРАТИВНЫЙ РАПОРТ"
    uav_callsign: str = "МЧС-БАС-04"
    organization: str = "МЧС РОССИИ — ЦУКС"


class MissionVideoReporter:
    """Генератор тактических видеоотчётов патрулирования."""

    # Каскад поддерживаемых видеокодеков (FourCC) для надёжности на Windows и Linux
    CODEC_CASCADE = [
        ("mp4v", ".mp4"),
        ("avc1", ".mp4"),
        ("XVID", ".avi"),
        ("MJPG", ".avi"),
    ]

    # TrueType-шрифты с кириллицей. OpenCV Hershey не умеет русский текст.
    FONT_CANDIDATES = (
        r"C:\Windows\Fonts\arial.ttf",
        r"C:\Windows\Fonts\segoeui.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
    )
    FONT_BOLD_CANDIDATES = (
        r"C:\Windows\Fonts\arialbd.ttf",
        r"C:\Windows\Fonts\segoeuib.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
    )

    def __init__(self, config: Optional[VideoReportConfig] = None):
        self.config = config or VideoReportConfig()
        self._regular_font_path = self._resolve_font(self.FONT_CANDIDATES)
        self._bold_font_path = self._resolve_font(self.FONT_BOLD_CANDIDATES) \
            or self._regular_font_path

    @staticmethod
    def _resolve_font(candidates) -> Optional[str]:
        for path in candidates:
            if os.path.isfile(path):
                return path
        return None

    def _font(self, size: int, bold: bool = False):
        path = self._bold_font_path if bold else self._regular_font_path
        if path:
            try:
                return ImageFont.truetype(path, max(8, int(size)))
            except OSError:
                pass
        return ImageFont.load_default()

    def _draw_texts(self, frame: np.ndarray, items) -> np.ndarray:
        """Нарисовать список (text, x, y, size, bgr, bold, max_width) через PIL."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        draw = ImageDraw.Draw(image)
        for item in items:
            text, x, y, size, color_bgr = item[:5]
            bold = bool(item[5]) if len(item) > 5 else False
            max_width = item[6] if len(item) > 6 else None
            font = self._font(size, bold)
            if max_width:
                while size > 10:
                    box = draw.textbbox((0, 0), text, font=font)
                    if box[2] - box[0] <= max_width:
                        break
                    size -= 1
                    font = self._font(size, bold)
                box = draw.textbbox((0, 0), text, font=font)
                if box[2] - box[0] > max_width:
                    # Обрезаем длинную строку, чтобы она не уезжала за кадр.
                    while text and draw.textbbox(
                            (0, 0), text + "…", font=font)[2] > max_width:
                        text = text[:-1]
                    text += "…"
            color_rgb = (int(color_bgr[2]), int(color_bgr[1]),
                         int(color_bgr[0]))
            draw.text((int(x), int(y)), text, font=font, fill=color_rgb)
        return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)

    def _create_writer(self, target_path: str) -> Tuple[cv2.VideoWriter, str]:
        """Инициализация VideoWriter с каскадом кодеков."""
        os.makedirs(os.path.dirname(os.path.abspath(target_path)), exist_ok=True)
        w, h, fps = self.config.width, self.config.height, self.config.fps

        base, ext = os.path.splitext(target_path)
        for fourcc_str, preferred_ext in self.CODEC_CASCADE:
            current_path = base + preferred_ext
            fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
            writer = cv2.VideoWriter(current_path, fourcc, fps, (w, h))
            if writer.isOpened():
                return writer, current_path

        # Аварийный fallback на встроенный MJPG
        fallback_path = base + ".avi"
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        writer = cv2.VideoWriter(fallback_path, fourcc, fps, (w, h))
        return writer, fallback_path

    def _render_title_card(self, title: str, subtitle: str, timestamp_str: str) -> np.ndarray:
        """Создаёт кадр титульной заставки."""
        w, h = self.config.width, self.config.height
        scale = min(w / 1280.0, h / 720.0)
        x0 = max(12, int(80 * scale))
        max_w = w - 2 * x0
        fs = lambda value: max(10, int(value * scale))  # noqa: E731
        ys = lambda value: int(value * scale)  # noqa: E731
        frame = np.full((h, w, 3), (15, 20, 28), dtype=np.uint8)

        # Фоновая тактическая сетка
        grid_color = (25, 34, 46)
        for x in range(0, w, 60):
            cv2.line(frame, (x, 0), (x, h), grid_color, 1)
        for y in range(0, h, 60):
            cv2.line(frame, (0, y), (w, y), grid_color, 1)

        # Декоративная акцентная рамка
        cv2.rectangle(frame, (40, 40), (w - 40, h - 40), (0, 140, 255), 2)
        cv2.rectangle(frame, (44, 44), (w - 44, h - 44), (0, 80, 180), 1)

        items = [
            (self.config.organization, x0, ys(70), fs(38),
             (0, 210, 255), True, max_w),
            (title, x0, ys(145), fs(62), (255, 255, 255), True, max_w),
            (subtitle, x0, ys(230), fs(36), (0, 160, 255), False, max_w),
        ]
        cv2.line(frame, (x0, ys(300)), (w - x0, ys(300)), (0, 120, 200), 2)
        items.extend([
            (f"БОРТОВОЙ КОМПЛЕКС: {self.config.uav_callsign}",
             x0, ys(355), fs(28), (220, 230, 240), False, max_w),
            (f"ДАТА И ВРЕМЯ ФОРМИРОВАНИЯ: {timestamp_str}",
             x0, ys(405), fs(24), (160, 180, 200), False, max_w),
            ("СТАТУС МИССИИ: АВТОНОМНЫЙ ПАТРУЛЬ // ДАННЫЕ ЗАФИКСИРОВАНЫ",
             x0, ys(460), fs(24), (0, 220, 120), True, max_w),
        ])
        return self._draw_texts(frame, items)

    def _render_summary_card(self, stats: Dict[str, any]) -> np.ndarray:
        """Создаёт финальный экран с итогами миссии."""
        w, h = self.config.width, self.config.height
        scale = min(w / 1280.0, h / 720.0)
        x0 = max(12, int(80 * scale))
        max_w = w - 2 * x0
        fs = lambda value: max(10, int(value * scale))  # noqa: E731
        ys = lambda value: int(value * scale)  # noqa: E731
        frame = np.full((h, w, 3), (12, 16, 22), dtype=np.uint8)

        # Фоновая тактическая сетка
        grid_color = (20, 28, 38)
        for x in range(0, w, 60):
            cv2.line(frame, (x, 0), (x, h), grid_color, 1)
        for y in range(0, h, 60):
            cv2.line(frame, (0, y), (w, y), grid_color, 1)

        cv2.rectangle(frame, (int(40*scale), int(40*scale)),
                      (w - int(40*scale), h - int(40*scale)), (0, 210, 120), 2)
        items = [
            ("СВОДНЫЙ ОТЧЁТ РЕЗУЛЬТАТОВ ВОЗДУШНОГО ПАТРУЛЯ",
             x0, ys(75), fs(42), (255, 255, 255), True, max_w),
        ]
        cv2.line(frame, (x0, ys(150)), (w - x0, ys(150)), (0, 210, 120), 2)

        y_offset = ys(190)
        for key, val in stats.items():
            label = str(key).upper().replace("_", " ")
            items.append((f"{label}: {val}", x0, y_offset, fs(28),
                          (200, 220, 240), False, max_w))
            y_offset += ys(55)

        items.append(("МАТЕРИАЛЫ ПЕРЕДАНЫ В ОПЕРАТИВНЫЙ ШТАБ ЦУКС МЧС РОССИИ",
                      x0, h - ys(90), fs(24), (0, 210, 255), True, max_w))
        return self._draw_texts(frame, items)

    def compile_report(
        self,
        frames: List[np.ndarray],
        incidents: Optional[List[Dict]] = None,
        stats: Optional[Dict] = None,
        output_path: str = "reports/video/mission_summary.mp4",
    ) -> str:
        """Компилирует сквозной видеорапорт."""
        writer, final_path = self._create_writer(output_path)
        w, h, fps = self.config.width, self.config.height, self.config.fps

        timestamp_str = datetime.now().strftime("%d.%m.%Y %H:%M:%S МСК")

        try:
            # 1. Титульная заставка (2.0 секунды)
            title_frame = self._render_title_card(
                title=self.config.mission_title,
                subtitle="АВТОНОМНОЕ ВЫЯВЛЕНИЕ ЧРЕЗВЫЧАЙНЫХ СИТУАЦИЙ",
                timestamp_str=timestamp_str,
            )
            title_repeats = int(fps * 2.0)
            for _ in range(title_repeats):
                writer.write(title_frame)

            # 2. Основные кадры полёта (каждый кадр удерживается по 1.0 сек)
            frame_hold = max(1, int(fps * 1.0))
            for frame in frames:
                resized = cv2.resize(frame, (w, h)) if (frame.shape[1] != w or frame.shape[0] != h) else frame
                for _ in range(frame_hold):
                    writer.write(resized)

            # 3. Карточки инцидентов (если переданы)
            if incidents:
                card_scale = min(w / 1280.0, h / 720.0)
                cx0 = max(12, int(90 * card_scale))
                cmax = w - 2 * cx0
                cfs = lambda value: max(10, int(value * card_scale))  # noqa: E731
                cys = lambda value: int(value * card_scale)  # noqa: E731
                for inc in incidents:
                    card = np.full((h, w, 3), (18, 22, 32), dtype=np.uint8)
                    cv2.rectangle(card, (int(50*card_scale), int(50*card_scale)),
                                  (w - int(50*card_scale), h - int(50*card_scale)),
                                  (0, 90, 255), 2)
                    card = self._draw_texts(card, [
                        (f"ОБНАРУЖЕН ИНЦИДЕНТ: {inc.get('type', 'ЧС')}",
                         cx0, cys(105), cfs(38), (0, 90, 255), True, cmax),
                        (f"ОПИСАНИЕ: {inc.get('label', '')}",
                         cx0, cys(190), cfs(28), (255, 255, 255), False, cmax),
                        (f"КООРДИНАТЫ: {inc.get('coords', '')}",
                         cx0, cys(255), cfs(28), (0, 220, 255), False, cmax),
                        (f"МАСШТАБ / ПЛОЩАДЬ: {inc.get('area', '')}",
                         cx0, cys(320), cfs(28), (200, 240, 200), False, cmax),
                    ])
                    card_hold = int(fps * 1.5)
                    for _ in range(card_hold):
                        writer.write(card)

            # 4. Финальная сводка (2.0 секунды)
            stats_dict = stats or {
                "статус": "ПАТРУЛИРОВАНИЕ ЗАВЕРШЕНО",
                "обнаружено_инцидентов": len(incidents) if incidents else 0,
            }
            summary_frame = self._render_summary_card(stats_dict)
            summary_repeats = int(fps * 2.0)
            for _ in range(summary_repeats):
                writer.write(summary_frame)

        finally:
            writer.release()

        return final_path
