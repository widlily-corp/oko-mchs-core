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


def test_title_card_text_does_not_touch_frame():
    """Вёрстка титула на 640x360: текст не наезжает на акцентную рамку."""
    reporter = MissionVideoReporter(VideoReportConfig(width=640, height=360,
                                                      fps=10.0))
    card = reporter._render_title_card(
        "ОКО МЧС // ОПЕРАТИВНЫЙ РАПОРТ",
        "АВТОНОМНОЕ ВЫЯВЛЕНИЕ ЧРЕЗВЫЧАЙНЫХ СИТУАЦИЙ",
        "11.11.2026 21:00:00 МСК",
    )
    pad = MissionVideoReporter.card_pad(640, 360)
    top_margin = card[: pad - 2, :, :]
    bottom_margin = card[360 - pad + 4:, :, :]
    assert int(top_margin.max()) < 60, "текст залез на верхнюю кромку рамки"
    assert int(bottom_margin.max()) < 60, "текст залез на нижнюю кромку рамки"


def test_report_includes_every_flight_frame():
    """Каждый переданный кадр полёта попадает в ролик (по 1 с на кадр)."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        reporter = MissionVideoReporter(VideoReportConfig(width=640, height=360,
                                                          fps=10.0))
        frames = [
            create_sample_frame("КАДР 1", color=(40, 50, 20)),
            create_sample_frame("КАДР 2", color=(10, 30, 180)),
            create_sample_frame("КАДР 3", color=(120, 20, 140)),
        ]
        path = reporter.compile_report(
            frames=frames,
            stats={"статус": "ТЕСТ"},
            output_path=os.path.join(tmp_dir, "frames.mp4"),
        )
        cap = cv2.VideoCapture(path)
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        # титул 2 с + 3 кадра по 1 с + сводка 2 с = 70 кадров при 10 fps
        assert count >= 65, f"кадры полёта потерялись: {count} кадров"


def test_report_includes_incident_shots():
    """Кадр фотофиксации очага попадает в ролик отдельной вставкой."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        reporter = MissionVideoReporter(VideoReportConfig(width=640, height=360,
                                                          fps=10.0))
        shot = {
            "frame": create_sample_frame("FLOOD", color=(200, 120, 20)),
            "label": "Зона затопления",
            "threat_level": "CRITICAL",
            "area": 221.0,
            "x": -26.3,
            "y": 65.1,
            "time": "08:55:08",
        }
        path = reporter.compile_report(
            frames=[create_sample_frame("ПАТРУЛЬ")],
            stats={"статус": "ТЕСТ"},
            output_path=os.path.join(tmp_dir, "shots.mp4"),
            shots=[shot],
        )
        cap = cv2.VideoCapture(path)
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        # титул 2 с + кадр 1 с + фотофиксация 2 с + сводка 2 с = 70 кадров
        assert count >= 65, f"вставка фотофиксации потерялась: {count}"


def test_incident_shot_has_caption_bar():
    """Под кадром инцидента есть тёмная плашка с координатами."""
    reporter = MissionVideoReporter(VideoReportConfig(width=640, height=360,
                                                      fps=10.0))
    frame = reporter._render_incident_shot({
        "frame": create_sample_frame("FLOOD"),
        "label": "Зона затопления",
        "threat_level": "CRITICAL",
        "area": 221.0,
        "x": -26.3,
        "y": 65.1,
        "time": "08:55:08",
    })
    assert frame.shape == (360, 640, 3)
    assert int(frame[-4:, :, :].max()) < 60, "нижняя плашка не отрисована"


def test_person_caption_is_plain_russian_and_not_area(monkeypatch):
    reporter = MissionVideoReporter(VideoReportConfig(width=640, height=360))
    texts = []
    def capture(frame, items):
        texts.extend(item[0] for item in items)
        return frame
    monkeypatch.setattr(reporter, "_draw_texts", capture)
    reporter._render_incident_shot({"label": "VICTIM", "threat_level": "CRITICAL",
                                    "area": 1.8, "frame": create_sample_frame("person")})
    caption = " ".join(texts)
    assert "ЧЕЛОВЕК" in caption and "КРИТИЧЕСКАЯ УГРОЗА" in caption
    assert "состояние не подтверждено" in caption
    assert "м²" not in caption
