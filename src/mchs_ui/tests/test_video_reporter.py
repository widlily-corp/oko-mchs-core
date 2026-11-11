"""Тесты компилятора тактических видеоотчётов и видеорапорта миссии."""

from __future__ import annotations

import os
import tempfile
import cv2
import numpy as np
import pytest

from mchs_core.video_reporter import MissionVideoReporter, VideoReportConfig


def create_sample_frame(text: str, color=(30, 40, 20), w=640, h=360) -> np.ndarray:
    frame = np.full((h, w, 3), color, dtype=np.uint8)
    cv2.putText(frame, text, (50, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return frame


def test_video_reporter_compiles_valid_video():
    """Проверяет сборку MP4 видеорапорта с титульным экраном, кадрами и титрами."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        output_video_path = os.path.join(tmp_dir, "mission_report.mp4")

        cfg = VideoReportConfig(
            width=640,
            height=360,
            fps=10.0,
            mission_title="ТЕСТОВЫЙ ПОЛЁТ // МЧС-04",
        )
        reporter = MissionVideoReporter(cfg)

        # 3 кадра миссии
        frames = [
            create_sample_frame("ПАТРУЛИРОВАНИЕ: СЕКТОР А", color=(40, 50, 20)),
            create_sample_frame("ОБНАРУЖЕН ОЧАГ: ПОЖАР (64 м²)", color=(10, 30, 180)),
            create_sample_frame("ОБНАРУЖЕН ЧЕЛОВЕК: [15.0, -22.0]", color=(120, 20, 140)),
        ]

        incidents = [
            {
                "type": "FIRE",
                "label": "Лесной пожар",
                "coords": "X=-62.2, Y=68.5",
                "area": "64.2 м²",
            },
            {
                "type": "VICTIM",
                "label": "Пострадавший",
                "coords": "X=15.0, Y=-22.0",
                "area": "1 чел (сигнальный жилет)",
            },
        ]

        stats = {
            "flight_time": "12 мин 40 с",
            "distance_m": "1840 м",
            "hazards_found": 2,
            "status": "МИССИЯ УСПЕШНО ЗАВЕРШЕНА",
        }

        result_path = reporter.compile_report(
            frames=frames,
            incidents=incidents,
            stats=stats,
            output_path=output_video_path,
        )

        assert os.path.isfile(result_path), f"Видеофайл не создан: {result_path}"
        assert os.path.getsize(result_path) > 1000, "Видеофайл слишком мал (пустой)"

        # Проверяем читаемость сгенерированного видео через OpenCV VideoCapture
        cap = cv2.VideoCapture(result_path)
        assert cap.isOpened(), "cv2.VideoCapture не смог открыть сгенерированное видео"
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        assert frame_count >= 15, f"Недостаточно кадров в ролике: {frame_count}"
        cap.release()


def test_video_reporter_uses_cyrillic_capable_font():
    """OpenCV Hershey рисовал '?' вместо русского; должен быть TrueType-шрифт."""
    reporter = MissionVideoReporter(VideoReportConfig(width=640, height=360))
    assert reporter._regular_font_path is not None
    card = reporter._render_title_card(
        "ОКО МЧС // ОПЕРАТИВНЫЙ РАПОРТ",
        "АВТОНОМНОЕ ВЫЯВЛЕНИЕ ЧРЕЗВЫЧАЙНЫХ СИТУАЦИЙ",
        "11.11.2026 21:00:00 МСК",
    )
    assert card.shape == (360, 640, 3)
    assert card.mean() > 0
