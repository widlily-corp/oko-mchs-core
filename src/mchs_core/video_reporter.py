"""Компилятор тактических видеорапортов и видеоотчётов миссий «Око МЧС».

Модуль создаёт готовый видеоролик MP4/AVI с каскадной защитой от сбоев кодеков:
1. Вступительная тактическая заставка (Title Card) с логотипом МЧС, номером борта и временем.
2. Последовательность ключевых кадров миссии и тактических съёмок с бортовой камеры.
3. Кадры фотофиксации обнаруженных инцидентов (очаг пожара, пострадавший,
   разлив нефти) с подписью координат, площади и времени.
4. Текстовые карточки инцидентов, если готовых кадров нет.
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

    @staticmethod
    def card_pad(width: int, height: int) -> int:
        """Внутренний отступ акцентной рамки карточек (масштабируется)."""
        return max(12, int(min(width, height) * 0.055))

    def _render_title_card(self, title: str, subtitle: str, timestamp_str: str) -> np.ndarray:
        """Создаёт кадр титульной заставки."""
        w, h = self.config.width, self.config.height
        scale = min(w / 1280.0, h / 720.0)
        fs = lambda value: max(10, int(value * scale))  # noqa: E731
        pad = self.card_pad(w, h)
        frame = np.full((h, w, 3), (15, 20, 28), dtype=np.uint8)

        # Фоновая тактическая сетка
        grid_color = (25, 34, 46)
        for x in range(0, w, 60):
            cv2.line(frame, (x, 0), (x, h), grid_color, 1)
        for y in range(0, h, 60):
            cv2.line(frame, (0, y), (w, y), grid_color, 1)

        # Декоративная акцентная рамка
        cv2.rectangle(frame, (pad, pad), (w - pad, h - pad), (0, 140, 255), 2)
        cv2.rectangle(frame, (pad + 4, pad + 4), (w - pad - 4, h - pad - 4),
                      (0, 80, 180), 1)

        x0 = pad + max(10, int(28 * scale))
        max_w = w - pad - x0
        y_cursor = [pad + max(4, int(8 * scale))]

        def next_line(size: int, gap: int) -> int:
            """Возвращает верх строки и сдвигает курсор ниже неё.

            ``_draw_texts`` рисует от верхней кромки (PIL anchor по умолчанию —
            left-ascender), поэтому высоту шрифта прибавляем к курсору.
            """
            top_y = y_cursor[0]
            y_cursor[0] = top_y + size + max(2, int(gap * scale))
            return top_y

        org_y = next_line(fs(38), 12)
        title_y = next_line(fs(62), 22)
        subtitle_y = next_line(fs(36), 18)
        sep_y = y_cursor[0] + max(2, int(6 * scale))
        y_cursor[0] = sep_y + max(2, int(6 * scale))
        callsign_y = next_line(fs(28), 14)
        timestamp_y = next_line(fs(24), 10)
        status_y = next_line(fs(24), 10)

        cv2.line(frame, (x0, sep_y), (w - x0, sep_y), (0, 120, 200), 2)
        items = [
            (self.config.organization, x0, org_y, fs(38),
             (0, 210, 255), True, max_w),
            (title, x0, title_y, fs(62), (255, 255, 255), True, max_w),
            (subtitle, x0, subtitle_y, fs(36), (0, 160, 255), False, max_w),
            (f"БОРТОВОЙ КОМПЛЕКС: {self.config.uav_callsign}",
             x0, callsign_y, fs(28), (220, 230, 240), False, max_w),
            (f"ДАТА И ВРЕМЯ ФОРМИРОВАНИЯ: {timestamp_str}",
             x0, timestamp_y, fs(24), (160, 180, 200), False, max_w),
            ("СТАТУС МИССИИ: АВТОНОМНЫЙ ПАТРУЛЬ // ДАННЫЕ ЗАФИКСИРОВАНЫ",
             x0, status_y, fs(24), (0, 220, 120), True, max_w),
        ]
        return self._draw_texts(frame, items)

    def _render_summary_card(self, stats: Dict[str, any]) -> np.ndarray:
        """Создаёт финальный экран с итогами миссии."""
        w, h = self.config.width, self.config.height
        scale = min(w / 1280.0, h / 720.0)
        fs = lambda value: max(10, int(value * scale))  # noqa: E731
        pad = self.card_pad(w, h)
        frame = np.full((h, w, 3), (12, 16, 22), dtype=np.uint8)

        # Фоновая тактическая сетка
        grid_color = (20, 28, 38)
        for x in range(0, w, 60):
            cv2.line(frame, (x, 0), (x, h), grid_color, 1)
        for y in range(0, h, 60):
            cv2.line(frame, (0, y), (w, y), grid_color, 1)

        cv2.rectangle(frame, (pad, pad), (w - pad, h - pad), (0, 210, 120), 2)
        x0 = pad + max(10, int(28 * scale))
        max_w = w - pad - x0
        title_y = pad + max(4, int(8 * scale))
        sep_y = title_y + fs(42) + max(4, int(10 * scale))

        items = [
            ("СВОДНЫЙ ОТЧЁТ РЕЗУЛЬТАТОВ ВОЗДУШНОГО ПАТРУЛЯ",
             x0, title_y, fs(42), (255, 255, 255), True, max_w),
        ]
        cv2.line(frame, (x0, sep_y), (w - x0, sep_y), (0, 210, 120), 2)

        y_offset = sep_y + max(6, int(12 * scale))
        row_step = fs(28) + max(6, int(16 * scale))
        for key, val in stats.items():
            label = str(key).upper().replace("_", " ")
            items.append((f"{label}: {val}", x0, y_offset, fs(28),
                          (200, 220, 240), False, max_w))
            y_offset += row_step

        items.append(("МАТЕРИАЛЫ СФОРМИРОВАНЫ ДЛЯ ПЕРЕДАЧИ В ЦУКС",
                      x0, h - pad - fs(24) - max(4, int(8 * scale)), fs(24),
                      (0, 210, 255), True, max_w))
        return self._draw_texts(frame, items)

    def _render_incident_shot(self, shot: Dict) -> np.ndarray:
        """Кадр фотофиксации: снимок с борта + тактическая подпись снизу."""
        w, h = self.config.width, self.config.height
        scale = min(w / 1280.0, h / 720.0)
        fs = lambda value: max(10, int(value * scale))  # noqa: E731
        frame = shot.get("frame")
        canvas = np.full((h, w, 3), (10, 13, 18), dtype=np.uint8)
        if frame is not None and getattr(frame, "size", 0):
            src_h, src_w = frame.shape[:2]
            factor = min(w / float(src_w), h / float(src_h))
            new_w = max(1, int(src_w * factor))
            new_h = max(1, int(src_h * factor))
            resized = cv2.resize(frame, (new_w, new_h))
            x_off = (w - new_w) // 2
            y_off = (h - new_h) // 2
            canvas[y_off:y_off + new_h, x_off:x_off + new_w] = resized

        bar_h = max(44, int(82 * scale))
        cv2.rectangle(canvas, (0, h - bar_h), (w, h), (12, 16, 22), -1)
        cv2.line(canvas, (0, h - bar_h), (w, h - bar_h), (0, 120, 255), 2)

        threat = str(shot.get("threat_level") or "ЧС").upper()
        color = (0, 60, 255) if threat == "CRITICAL" else (0, 170, 255)
        label = shot.get("label") or shot.get("hazard_type") or "ЧС"
        labels = {"VICTIM": "ЧЕЛОВЕК — ТРЕБУЕТСЯ ПРОВЕРКА", "HUMAN": "ЧЕЛОВЕК — ТРЕБУЕТСЯ ПРОВЕРКА",
                  "FIRE": "ПОЖАР", "SMOKE": "ЗАДЫМЛЕНИЕ", "OIL": "РАЗЛИВ НЕФТИ",
                  "FLOOD": "ЗАТОПЛЕНИЕ", "WASTE": "СВАЛКА"}
        label = labels.get(str(label).upper(), label)
        threat = {"CRITICAL": "КРИТИЧЕСКАЯ УГРОЗА", "WARNING": "ВНИМАНИЕ",
                  "ADVISORY": "НАБЛЮДЕНИЕ"}.get(threat, threat)
        title = f"ФОТОФИКСАЦИЯ: {label} [{threat}]"
        parts = []
        if shot.get("area") is not None:
            try:
                area = float(shot["area"])
                if str(shot.get("hazard_type") or shot.get("label") or "").upper() in ("VICTIM", "HUMAN"):
                    parts.append("Обнаружен человек; состояние не подтверждено")
                else:
                    parts.append(f"{area:.0f} м²" if area >= 10.0
                                 else f"{area:.1f} м²")
            except (TypeError, ValueError):
                pass
        if shot.get("x") is not None and shot.get("y") is not None:
            try:
                parts.append(f"X={float(shot['x']):.1f} Y={float(shot['y']):.1f} м")
            except (TypeError, ValueError):
                pass
        if shot.get("time"):
            parts.append(str(shot["time"]))
        meta = "  •  ".join(parts)

        x0 = max(12, int(24 * scale))
        max_w = w - 2 * x0
        title_y = h - bar_h + max(3, int(8 * scale))
        items = [(title, x0, title_y, fs(26), color, True, max_w)]
        if meta:
            items.append((meta, x0, title_y + fs(26) + max(2, int(6 * scale)),
                          fs(22), (220, 230, 240), False, max_w))
        return self._draw_texts(canvas, items)

    def compile_report(
        self,
        frames: List[np.ndarray],
        incidents: Optional[List[Dict]] = None,
        stats: Optional[Dict] = None,
        output_path: str = "reports/video/mission_summary.mp4",
        shots: Optional[List[Dict]] = None,
    ) -> str:
        """Компилирует сквозной видеорапорт.

        ``shots`` — кадры фотофиксации очагов: словари с ключами ``frame``
        (кадр BGR), ``label``/``hazard_type``, ``threat_level``, ``area``,
        ``x``, ``y``, ``time``. Каждый показывается отдельно с подписью, чтобы
        зритель видел именно то, что обнаружил борт, а не только текст.
        """
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

            # 3. Кадры фотофиксации очагов (то, что реально видел борт)
            if shots:
                shot_hold = max(1, int(fps * 4.0))
                for shot in shots:
                    shot_frame = self._render_incident_shot(shot)
                    for _ in range(shot_hold):
                        writer.write(shot_frame)

            # 4. Текстовые карточки инцидентов (если переданы)
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

            # 5. Финальная сводка (2.0 секунды)
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
