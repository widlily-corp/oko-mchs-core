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

    def __init__(self, config: Optional[VideoReportConfig] = None):
        self.config = config or VideoReportConfig()

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

        font = cv2.FONT_HERSHEY_SIMPLEX
        # Заголовок
        cv2.putText(frame, self.config.organization, (80, 120), font, 0.75, (0, 210, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, title, (80, 200), font, 1.25, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, subtitle, (80, 260), font, 0.85, (0, 160, 255), 2, cv2.LINE_AA)

        # Данные БПЛА и время
        cv2.line(frame, (80, 310), (w - 80, 310), (0, 120, 200), 2)
        cv2.putText(frame, f"БОРТОВОЙ КОМПЛЕКС: {self.config.uav_callsign}", (80, 380), font, 0.70, (220, 230, 240), 2, cv2.LINE_AA)
        cv2.putText(frame, f"ДАТА И ВРЕМЯ ФОРМИРОВАНИЯ: {timestamp_str}", (80, 430), font, 0.65, (160, 180, 200), 1, cv2.LINE_AA)
        cv2.putText(frame, "СТАТУС МИССИИ: АВТОНОМНЫЙ ПАТРУЛЬ // ДАННЫЕ ЗАФИКСИРОВАНЫ", (80, 480), font, 0.60, (0, 220, 120), 2, cv2.LINE_AA)

        return frame

    def _render_summary_card(self, stats: Dict[str, any]) -> np.ndarray:
        """Создаёт финальный экран с итогами миссии."""
        w, h = self.config.width, self.config.height
        frame = np.full((h, w, 3), (12, 16, 22), dtype=np.uint8)

        # Фоновая тактическая сетка
        grid_color = (20, 28, 38)
        for x in range(0, w, 60):
            cv2.line(frame, (x, 0), (x, h), grid_color, 1)
        for y in range(0, h, 60):
            cv2.line(frame, (0, y), (w, y), grid_color, 1)

        cv2.rectangle(frame, (40, 40), (w - 40, h - 40), (0, 210, 120), 2)
        font = cv2.FONT_HERSHEY_SIMPLEX

        cv2.putText(frame, "СВОДНЫЙ ОТЧЁТ РЕЗУЛЬТАТОВ ВОЗДУШНОГО ПАТРУЛЯ", (80, 120), font, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.line(frame, (80, 150), (w - 80, 150), (0, 210, 120), 2)

        y_offset = 220
        for key, val in stats.items():
            label = str(key).upper().replace("_", " ")
            text = f"{label}: {val}"
            cv2.putText(frame, text, (80, y_offset), font, 0.72, (200, 220, 240), 2, cv2.LINE_AA)
            y_offset += 55

        cv2.putText(frame, "МАТЕРИАЛЫ ПЕРЕДАНЫ В ОПЕРАТИВНЫЙ ШТАБ ЦУКС МЧС РОССИИ", (80, h - 80), font, 0.65, (0, 210, 255), 2, cv2.LINE_AA)
        return frame

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
                for inc in incidents:
                    card = np.full((h, w, 3), (18, 22, 32), dtype=np.uint8)
                    cv2.rectangle(card, (50, 50), (w - 50, h - 50), (0, 90, 255), 2)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(card, f"ОБНАРУЖЕН ИНЦИДЕНТ: {inc.get('type', 'ЧС')}", (90, 140), font, 1.1, (0, 90, 255), 3, cv2.LINE_AA)
                    cv2.putText(card, f"ОПИСАНИЕ: {inc.get('label', '')}", (90, 220), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(card, f"КООРДИНАТЫ: {inc.get('coords', '')}", (90, 290), font, 0.8, (0, 220, 255), 2, cv2.LINE_AA)
                    cv2.putText(card, f"МАСШТАБ / ПЛОЩАДЬ: {inc.get('area', '')}", (90, 360), font, 0.8, (200, 240, 200), 2, cv2.LINE_AA)
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
