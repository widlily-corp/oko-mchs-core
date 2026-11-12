"""Тесты производительности и корректности LatestFrameStreamer (on-demand JPEG encoding)."""

import threading
import time
from unittest.mock import patch

import cv2
import numpy as np
import pytest

from mchs_vision.web_stream import LatestFrameStreamer


def test_update_zero_jpeg_encoding_performance():
    """Проверка, что update() не выполняет cv2.imencode и отрабатывает < 5 мс за 100 кадров 1280x720."""
    streamer = LatestFrameStreamer(start_server=False)
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    # Прогрев
    for _ in range(5):
        streamer.update(frame)

    # 100 обновлений HD-кадров
    t0 = time.perf_counter()
    for _ in range(100):
        streamer.update(frame)
    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    print(f"\n[BENCHMARK] 100 updates of 1280x720: {elapsed_ms:.4f} ms ({elapsed_ms/100*1000:.2f} µs/update)")

    # Никакого кодирования не произошло: _encoded_frame должен оставаться None
    assert streamer._encoded_frame is None
    assert streamer._frame_id == 105
    # 100 обновлений чистого сохранения указателя должны занимать < 5 мс (обычно < 0.2 мс)
    assert elapsed_ms < 5.0, f"100 updates took {elapsed_ms:.2f} ms (expected < 5.0 ms)"


def test_snapshot_encodes_and_caches_result():
    """Проверка, что snapshot() кодирует один раз и кэширует байты для последующих запросов."""
    streamer = LatestFrameStreamer(quality=75, start_server=False)
    frame = np.full((720, 1280, 3), 120, dtype=np.uint8)

    # До update snapshot возвращает None
    assert streamer.snapshot() is None

    streamer.update(frame)
    assert streamer._encoded_frame is None
    assert streamer._cached_frame_id == -1

    # Первый вызов snapshot - кодирование и кэширование
    with patch("cv2.imencode", wraps=cv2.imencode) as mock_imencode:
        first_bytes = streamer.snapshot()
        assert first_bytes is not None
        assert isinstance(first_bytes, bytes)
        assert len(first_bytes) > 0
        assert first_bytes[:2] == b"\xff\xd8"  # JPEG SOI marker
        assert mock_imencode.call_count == 1
        assert streamer._cached_frame_id == streamer._frame_id

        # Повторные вызовы на том же кадре возвращают тот же объект из кэша без вызова imencode
        for _ in range(10):
            cached_bytes = streamer.snapshot()
            assert cached_bytes is first_bytes
        assert mock_imencode.call_count == 1


def test_snapshot_updates_on_new_frame():
    """Проверка, что при поступлении нового кадра кэш сбрасывается и генерируются новые байты."""
    streamer = LatestFrameStreamer(quality=80, start_server=False)

    frame1 = np.full((100, 100, 3), 30, dtype=np.uint8)
    frame2 = np.full((100, 100, 3), 230, dtype=np.uint8)

    streamer.update(frame1)
    bytes1 = streamer.snapshot()
    assert bytes1 is not None

    # Новый кадр
    streamer.update(frame2)
    assert streamer._frame_id != streamer._cached_frame_id

    bytes2 = streamer.snapshot()
    assert bytes2 is not None
    assert bytes2 != bytes1
    assert streamer._cached_frame_id == streamer._frame_id

    # Последующий запрос снова берет из кэша
    bytes3 = streamer.snapshot()
    assert bytes3 is bytes2


def test_invalid_and_empty_frames():
    """Проверка обработки некорректных и пустых кадров."""
    streamer = LatestFrameStreamer(start_server=False)

    # None
    streamer.update(None)
    assert streamer.snapshot() is None
    assert streamer._raw_frame is None
    assert streamer._frame_id == 0

    # Пустой массив
    streamer.update(np.empty((0, 0, 3), dtype=np.uint8))
    assert streamer.snapshot() is None
    assert streamer._raw_frame is None
    assert streamer._frame_id == 0

    # Корректный кадр после пустых
    valid_frame = np.zeros((50, 50, 3), dtype=np.uint8)
    streamer.update(valid_frame)
    data = streamer.snapshot()
    assert data is not None
    assert streamer._frame_id == 1

    # Пустой массив поверх валидного не затирает состояние
    streamer.update(np.empty((0, 0, 3), dtype=np.uint8))
    assert streamer.snapshot() == data
    assert streamer._frame_id == 1


def test_thread_safety_concurrent_readers():
    """Проверка потокобезопасности при конкурентном чтении snapshot() и параллельных update()."""
    streamer = LatestFrameStreamer(start_server=False)
    stop_event = threading.Event()
    errors = []

    def writer_loop():
        try:
            for i in range(100):
                img = np.full((120, 160, 3), i % 256, dtype=np.uint8)
                streamer.update(img)
                time.sleep(0.001)
        except Exception as e:
            errors.append(e)

    def reader_loop():
        try:
            while not stop_event.is_set():
                data = streamer.snapshot()
                if data is not None:
                    # Валидируем заголовок JPEG
                    assert data[:2] == b"\xff\xd8", "Corrupted JPEG header"
                    assert len(data) > 100, "Truncated JPEG"
                time.sleep(0.0005)
        except Exception as e:
            errors.append(e)

    # Запускаем 1 пишущий поток и 8 читающих потоков
    writer = threading.Thread(target=writer_loop)
    readers = [threading.Thread(target=reader_loop) for _ in range(8)]

    for r in readers:
        r.start()
    writer.start()

    writer.join(timeout=5.0)
    assert not writer.is_alive(), "Writer thread timed out"

    time.sleep(0.05)
    stop_event.set()

    for r in readers:
        r.join(timeout=2.0)
        assert not r.is_alive(), "Reader thread timed out"

    assert len(errors) == 0, f"Thread errors: {errors}"
    assert streamer.snapshot() is not None


def test_clean_shutdown():
    """Проверка корректного останова сервера и потоков без утечек."""
    # Без сервера
    s1 = LatestFrameStreamer(start_server=False)
    s1.stop()
    assert not s1.running

    # С сервером на свободном порту
    s2 = LatestFrameStreamer(port=0, start_server=True)
    assert s2.running
    assert s2._server is not None
    assert s2._thread is not None and s2._thread.is_alive()
    s2.stop()
    assert not s2.running
    assert s2._server is None
    assert s2._thread is None
