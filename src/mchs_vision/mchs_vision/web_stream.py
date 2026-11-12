"""HTTP-трансляция аннотированного видеопотока для АРМ оператора.

Совместимо с `src/mchs_ui/app.py`: АРМ опрашивает
``http://127.0.0.1:8080/snapshot.jpg``. Дополнительно доступны:

* ``/``            — страница с живым потоком;
* ``/stream``      — MJPEG-поток;
* ``/snapshot.jpg`` — последний кадр.

Модуль не зависит от ROS: нода simply вызывает :meth:`LatestFrameStreamer.update`
на каждом аннотированном кадре.
"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from socketserver import ThreadingMixIn
from typing import Optional

import cv2
import numpy as np

_INDEX_HTML = """<!DOCTYPE html>
<html lang="ru"><head><meta charset="utf-8">
<title>ОКО МЧС — камера БПЛА</title>
<style>
body { background:#0b0f19; color:#e2e8f0; font-family:'Segoe UI',sans-serif;
       text-align:center; margin:0; padding:20px; }
h1 { color:#38bdf8; font-size:22px; margin-bottom:6px; letter-spacing:1px; }
.sub { color:#94a3b8; font-size:14px; margin-bottom:18px; }
.frame-box { display:inline-block; padding:6px; background:#1e293b;
             border-radius:12px; box-shadow:0 10px 25px rgba(0,0,0,.5);
             border:1px solid #334155; }
img { display:block; border-radius:8px; max-width:92vw; height:auto; }
.badge { display:inline-block; padding:4px 12px; margin-top:14px;
         background:#065f46; color:#34d399; border-radius:9999px;
         font-size:13px; font-weight:bold; }
</style></head><body>
<h1>ОКО МЧС — ПРЯМОЙ ЭФИР С БОРТОВОЙ КАМЕРЫ БПЛА</h1>
<div class="sub">ROS 2 <code>/mchs/camera/annotated</code> &bull; OpenCV AI Detection</div>
<div class="frame-box"><img src="/stream" alt="Live Stream"></div>
<br><div class="badge">&bull; ТАКТИЧЕСКИЙ ЭФИР АКТИВЕН</div>
</body></html>"""


class _StreamHandler(BaseHTTPRequestHandler):
    timeout = 10.0
    streamer: "LatestFrameStreamer"

    def log_message(self, format: str, *args) -> None:  # noqa: A002
        pass  # не засоряем лог ROS HTTP-запросами

    def do_GET(self) -> None:  # noqa: N802
        if self.path in ("/", "/index.html"):
            body = _INDEX_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/snapshot.jpg":
            data = self.streamer.snapshot()
            if data is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif self.path == "/state.json":
            payload = json.dumps(self.streamer.state_snapshot(),
                                 ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)
        elif self.path == "/stream":
            self._stream_mjpeg()
        else:
            self.send_response(404)
            self.end_headers()

    def _stream_mjpeg(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        while self.streamer.running:
            data = self.streamer.snapshot()
            if self.rfile is None:
                break
            if data is not None:
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break
            time.sleep(0.033)
        self.close_connection = True


class _IdleGuardMixIn(ThreadingMixIn):
    daemon_threads = True

    def process_request(self, request, client_address):
        request.settimeout(10.0)
        super().process_request(request, client_address)


class LatestFrameStreamer:
    """Потокобезопасный HTTP-сервер последнего JPEG-кадра с ленивым on-demand кодированием."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8080,
                 quality: int = 80, start_server: bool = True):
        self.host = host
        self.port = int(port)
        self.quality = int(quality)
        self.running = True
        self._raw_frame: Optional[np.ndarray] = None
        self._encoded_frame: Optional[bytes] = None
        self._frame_id: int = 0
        self._cached_frame_id: int = -1
        self._lock = threading.Lock()
        self._state: dict = {}

        self._server: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

        if start_server:
            handler = type("BoundStreamHandler", (_StreamHandler,),
                           {"streamer": self})
            server_cls = type("GuardedHTTPServer", (_IdleGuardMixIn, ThreadingHTTPServer), {})
            self._server = server_cls((self.host, self.port), handler)
            self._thread = threading.Thread(target=self._server.serve_forever,
                                            daemon=True)
            self._thread.start()

    @property
    def _frame(self) -> Optional[bytes]:
        """Обратная совместимость для интроспекции."""
        with self._lock:
            return self._encoded_frame

    def update(self, frame_bgr: np.ndarray) -> None:
        """Запомнить кадр без немедленного кодирования в JPEG.

        Вызывается из высокочастотного ROS-колбэка. Должен отрабатывать
        за доли микросекунды (<0.01 мс) без вызова cv2.imencode.
        """
        if frame_bgr is None or frame_bgr.size == 0:
            return
        with self._lock:
            self._raw_frame = frame_bgr
            self._frame_id += 1

    def snapshot(self) -> Optional[bytes]:
        """Получить JPEG-представление последнего кадра.

        Кодирует в JPEG 'на лету' только при реальном запросе и кэширует
        результат для повторных запросов одного и того же кадра.
        """
        with self._lock:
            raw = self._raw_frame
            fid = self._frame_id
            cached_fid = self._cached_frame_id
            cached_bytes = self._encoded_frame

            if raw is None:
                return None
            if fid == cached_fid and cached_bytes is not None:
                return cached_bytes

            ok, encoded = cv2.imencode(
                ".jpg", raw,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.quality],
            )
            if not ok:
                return None
            encoded_bytes = encoded.tobytes()
            self._encoded_frame = encoded_bytes
            self._cached_frame_id = fid
            return encoded_bytes

    def set_state(self, state: dict) -> None:
        """Обновить телеметрию для АРМ (позиция БПЛА, скан, детекции)."""
        if not isinstance(state, dict):
            return
        with self._lock:
            self._state = state

    def state_snapshot(self) -> dict:
        """Копия последней телеметрии (thread-safe)."""
        with self._lock:
            return dict(self._state)

    def stop(self) -> None:
        self.running = False
        if self._server is not None:
            try:
                self._server.shutdown()
            finally:
                self._server.server_close()
                self._server = None
        if self._thread is not None:
            if self._thread.is_alive() and threading.current_thread() != self._thread:
                self._thread.join(timeout=1.0)
            self._thread = None
