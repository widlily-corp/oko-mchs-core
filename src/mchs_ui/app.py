#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""АРМ ОПЕРАТОРА ЦУКС «ОКО МЧС» — тактический пульт мониторинга ЧС.

Что умеет пульт:

1. **Живое видео с борта.** Основной режим — MJPEG-поток ``/stream`` с
   веб-сервера ноды зрения (порт 8080). Если поток недоступен, пульт
   переключается на опрос ``/snapshot.jpg``, а если и его нет — рисует
   синтетический полигон, чтобы демонстрация не зависела от Gazebo.
2. **Мультиспектральная камера.** Оптический режим, тепловизор (палитра
   INFERNO) и ИИ-оверлей с рамкой детекции и подписями.
3. **Тактическая карта.** Зона патрулирования 60×40 м, маршрут «змейка»,
   трек БПЛА, очаги ЧС, дальномер и масштабная линейка.
4. **Телеметрия.** Высота, скорость, заряд АКБ, координаты, курс, фаза.
5. **Приём протокола ЦУКС.** Пульт сам замечает новый
   ``reports/incident_protocol.json`` (его пишет автопилот), включает
   тревогу, показывает карточку инцидента и подсвечивает очаг на карте.
6. **Экспорт.** Протокол в JSON и печатный бланк TXT для дежурного.

Горячие клавиши: ``Space`` — старт/RTL, ``T`` — тепловизор, ``O`` — оверлей,
``E`` — экспорт протокола, ``F11`` — полный экран, ``Esc`` — выйти из него.

Запуск::

    python3 src/mchs_ui/app.py              # обычный режим
    python3 src/mchs_ui/app.py --offline    # без обращения к видеосерверу
    python3 src/mchs_ui/app.py --autostart  # сразу начать демонстрационный полёт
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import sys
import threading
import time
import urllib.request
from pathlib import Path

# --- Подключение mchs_core и scripts для централизованного логирования и диагностики ---
REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "src"
SCRIPTS_DIR = REPO_ROOT / "scripts"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

try:
    from mchs_core import get_logger, log_event as core_log_event, install_crash_handler
    logger = get_logger("mchs_gcs", subsystem="GCS_UI")
    install_crash_handler("GCS_UI")
except ImportError:
    import logging
    logger = logging.getLogger("mchs_gcs")
    core_log_event = None
    install_crash_handler = None

try:
    from diagnose_logs import create_diagnostic_bundle
except ImportError:
    create_diagnostic_bundle = None



# --- Windows DPI: окно должно помещаться на экран при масштабе 100-150% ---
if sys.platform == "win32":
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(2)  # Per-Monitor V2
        except Exception:
            ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

import cv2
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk

try:  # customtkinter есть на Windows-машине оператора
    import customtkinter as ctk
except ImportError:  # pragma: no cover - подсказка при запуске без зависимости
    print("[ОШИБКА] Не установлен customtkinter. Выполните: pip install customtkinter")
    raise

try:  # звуковая сирена доступна только на Windows
    import winsound
except ImportError:  # pragma: no cover
    winsound = None


# ---------------------------------------------------------------------------
# Оформление и данные полигона
# ---------------------------------------------------------------------------

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

APP_TITLE = "ОКО МЧС — АРМ ОПЕРАТОРА ЦУКС"

COLORS = {
    "bg": "#0B0F14",
    "panel": "#121821",
    "panel_alt": "#0E141C",
    "border": "#1E2733",
    "grid": "#18212C",
    "text": "#E6EDF3",
    "muted": "#7C8DA6",
    "accent": "#FF6B35",
    "accent_dark": "#E85924",
    "ok": "#22C55E",
    "warn": "#F59E0B",
    "crit": "#EF4444",
    "info": "#38BDF8",
    "drone": "#FFFFFF",
}

FONT_UI = "Segoe UI"
FONT_MONO = "Consolas"

#: Размеры арены в метрах: 200×200, база МЧС в центре (0, 0).
WORLD_WIDTH_M = 200.0
WORLD_HEIGHT_M = 200.0
WORLD_HALF_M = 100.0

#: Демонстрационные очаги ЧС (координаты в метрах полигона).
HAZARDS_DB = {
    "FIRE": {
        "title": "Очаг лесного пожара",
        "short": "ПОЖАР",
        "threat": "CRITICAL",
        "color": "#FF4D2E",
        "x": 60.0,
        "y": 28.0,
        "area_sq_m": 64.2,
        "details": "Верховое распространение, пламя до 850 °C",
        "response": "Рекомендован наряд: 2 АЦ-40, автолестница АЛ-30, личный состав 8 чел.",
    },
    "OIL": {
        "title": "Разлив нефтепродуктов",
        "short": "НЕФТЬ",
        "threat": "WARNING",
        "color": "#8D6E63",
        "x": -58.0,
        "y": 30.0,
        "area_sq_m": 118.0,
        "details": "Утечка дизельного топлива, угроза водоёму",
        "response": "Рекомендован наряд: 1 АЦ-40, скиммер, боновое ограждение, 6 чел.",
    },
    "FLOOD": {
        "title": "Зона затопления",
        "short": "ПАВОДОК",
        "threat": "WARNING",
        "color": "#29B6F6",
        "x": 45.0,
        "y": -56.0,
        "area_sq_m": 340.5,
        "details": "Подъём уровня воды +1,8 м, затоплена дорога",
        "response": "Рекомендован наряд: 1 АСМ, 2 плавсредства, личный состав 6 чел.",
    },
    "WASTE": {
        "title": "Токсичные отходы",
        "short": "СВАЛКА",
        "threat": "CRITICAL",
        "color": "#FFD600",
        "x": 68.0,
        "y": -72.0,
        "area_sq_m": 28.0,
        "details": "Несанкционированная свалка, 12 бочек химреагентов",
        "response": "Рекомендован наряд: 1 АЦ-40, разведгруппа 4 чел., вывоз отходов.",
    },
}

#: Пороговые значения уровня угрозы (как в модуле зрения).
THREAT_WARNING_M2 = 15.0
THREAT_CRITICAL_M2 = 50.0

#: Где искать протокол инцидента: автопилот пишет в reports/, ручной экспорт — рядом с АРМ.
PROTOCOL_CANDIDATES = (
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "reports",
                 "incident_protocol.json"),
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "incident_protocol.json"),
)

THREAT_LABELS = {
    "CRITICAL": ("КРИТИЧЕСКИЙ", COLORS["crit"]),
    "WARNING": ("ПРЕДУПРЕЖДЕНИЕ", COLORS["warn"]),
    "ADVISORY": ("ВНИМАНИЕ", COLORS["info"]),
}


def classify_threat(area_sq_m: float) -> str:
    """Уровень угрозы по площади очага — та же шкала, что у модуля зрения."""
    if area_sq_m >= THREAT_CRITICAL_M2:
        return "CRITICAL"
    if area_sq_m >= THREAT_WARNING_M2:
        return "WARNING"
    return "ADVISORY"


def now_str(fmt: str = "%H:%M:%S") -> str:
    """Текущее время строкой (для логов и HUD)."""
    return datetime.datetime.now().strftime(fmt)


# ---------------------------------------------------------------------------
# Протокол инцидента
# ---------------------------------------------------------------------------


def normalize_protocol(raw: dict, path: str = "") -> dict:
    """Приводит протокол к единому виду.

    Поддерживаются оба формата: тот, что пишет автопилот
    (``mchs_incident_report`` с вложенной телеметрией), и тот, что АРМ
    выгружает вручную. Пульт не должен падать из-за разницы в схеме.
    """
    data = raw.get("mchs_incident_report", raw) or {}
    geo = data.get("target_geo") or {}
    telemetry = data.get("drone_telemetry") or {}
    hazard = str(data.get("hazard_type") or data.get("hazard_code") or "ЧС")
    threat = str(data.get("threat_level") or "WARNING").upper()

    area = data.get("estimated_area_sq_m", data.get("estimated_area_sq_meters", 0.0))
    try:
        area = float(area)
    except (TypeError, ValueError):
        area = 0.0

    return {
        "incident_id": str(data.get("incident_id") or data.get("protocol_id") or "—"),
        "hazard_type": hazard,
        "hazard_code": str(data.get("hazard_code") or "").upper(),
        "threat_level": threat,
        "area_sq_m": area,
        "latitude": geo.get("latitude", geo.get("lat")),
        "longitude": geo.get("longitude", geo.get("lon")),
        "x": geo.get("abs_x", telemetry.get("grid_x_m")),
        "y": geo.get("abs_y", telemetry.get("grid_y_m")),
        "altitude": telemetry.get("altitude_m"),
        "battery": telemetry.get("battery_percent"),
        "recommended_response": data.get("recommended_response") or "",
        "timestamp": data.get("timestamp_msk") or data.get("timestamp") or "",
        "path": path,
    }


class ProtocolWatcher:
    """Следит за файлом протокола и сообщает, когда появился новый инцидент."""

    def __init__(self, candidates=PROTOCOL_CANDIDATES):
        self.candidates = list(candidates)
        self.signature = None
        self.last = None

    def poll(self):
        """Возвращает нормализованный протокол, если файл обновился, иначе None.

        Приоритет у файла автопилота (``reports/incident_protocol.json``) —
        его пишет борт. Резервный файл рядом с АРМ используется, только если
        протокола от автопилота ещё нет.
        """
        path = None
        for candidate in self.candidates:
            if os.path.exists(candidate):
                path = candidate
                break
        if path is None:
            return None

        try:
            stamp = os.path.getmtime(path)
        except OSError:
            return None
        signature = (os.path.abspath(path), round(stamp, 3))
        if signature == self.signature:
            return None

        try:
            with open(path, "r", encoding="utf-8") as handle:
                raw = json.load(handle)
        except (OSError, ValueError):
            return None

        self.signature = signature
        self.last = normalize_protocol(raw, path)
        return self.last


# ---------------------------------------------------------------------------
# Видеопоток с борта
# ---------------------------------------------------------------------------


class VideoClient(threading.Thread):
    """Тянет кадры с веб-сервера ноды зрения: MJPEG, затем резервный снимок."""

    def __init__(self, host: str = "127.0.0.1", port: int = 8080, enabled: bool = True):
        super().__init__(daemon=True)
        self.base_url = f"http://{host}:{port}"
        self.enabled = enabled
        self.frame = None
        self.lock = threading.Lock()
        self.status = "OFFLINE"
        self.fps = 0.0
        self._stop = False

    def stop(self) -> None:
        """Просит поток завершиться."""
        self._stop = True

    def snapshot(self):
        """Копия последнего кадра (или None)."""
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def run(self) -> None:
        """Основной цикл: MJPEG, при неудаче — опрос снимков."""
        while not self._stop:
            if not self.enabled:
                self.status = "СИМУЛЯЦИЯ"
                time.sleep(0.5)
                continue
            if self._consume_mjpeg():
                continue
            self._poll_snapshot()
            time.sleep(0.5)

    def _consume_mjpeg(self) -> bool:
        """Читает multipart-поток /stream. Возвращает False, если поток недоступен."""
        try:
            response = urllib.request.urlopen(f"{self.base_url}/stream", timeout=2.0)
        except Exception:
            self.status = "SNAPSHOT"
            return False

        buffer = b""
        frames = 0
        started = time.time()
        try:
            while not self._stop:
                chunk = response.read(4096)
                if not chunk:
                    break
                buffer += chunk
                start = buffer.find(b"\xff\xd8")
                end = buffer.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                if start >= 0 and end > start:
                    jpeg = buffer[start:end + 2]
                    buffer = buffer[end + 2:]
                    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self.lock:
                            self.frame = frame
                        frames += 1
                        elapsed = max(1e-6, time.time() - started)
                        self.fps = frames / elapsed
                        if self.status != "MJPEG":
                            self.status = "MJPEG"
                            if logger:
                                logger.info(f"Установлена связь с MJPEG видеопотоком: {self.base_url}/stream")
                            if core_log_event:
                                core_log_event("GCS_UI", "STREAM_CONNECTED", {"endpoint": f"{self.base_url}/stream", "mode": "MJPEG"})
                if len(buffer) > 4_000_000:  # защита от мусора в потоке
                    buffer = buffer[-1_000_000:]
        except Exception:
            pass
        finally:
            try:
                response.close()
            except Exception:
                pass

        if frames == 0:
            self.status = "SNAPSHOT"
            return False
        return True

    def _poll_snapshot(self) -> None:
        """Резервный режим: отдельные JPEG-снимки /snapshot.jpg."""
        try:
            with urllib.request.urlopen(f"{self.base_url}/snapshot.jpg", timeout=0.4) as response:
                payload = np.frombuffer(response.read(), dtype=np.uint8)
            frame = cv2.imdecode(payload, cv2.IMREAD_COLOR)
            if frame is not None:
                with self.lock:
                    self.frame = frame
                if self.status != "SNAPSHOT":
                    self.status = "SNAPSHOT"
                    if logger:
                        logger.info(f"Установлена связь со snapshot видеопотоком: {self.base_url}/snapshot.jpg")
                    if core_log_event:
                        core_log_event("GCS_UI", "STREAM_CONNECTED", {"endpoint": f"{self.base_url}/snapshot.jpg", "mode": "SNAPSHOT"})
                self.fps = 0.0
        except Exception:
            if self.status != "OFFLINE":
                self.status = "OFFLINE"
                if logger:
                    logger.warning(f"Потеряно соединение с видеосервером {self.base_url}")
                if core_log_event:
                    core_log_event("GCS_UI", "STREAM_DISCONNECTED", {"endpoint": self.base_url}, level="WARNING")


# ---------------------------------------------------------------------------
# Основное окно
# ---------------------------------------------------------------------------


class OkoMchsApp(ctk.CTk):
    """Тактический пульт оперативного дежурного ЦУКС."""

    def __init__(self, offline: bool = False, autostart: bool = False):
        super().__init__()

        self.title(APP_TITLE)
        self.configure(fg_color=COLORS["bg"])
        self._fit_window()

        # Состояние миссии
        self.mission_active = False
        self.phase = "ДЕЖУРСТВО НА БАЗЕ"
        self.altitude = 0.0
        self.speed = 0.0
        self.battery = 98.4
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.heading_deg = 0.0
        self.trail: list[tuple[float, float]] = []
        self.hazard_key = "FIRE"
        self.thermal = False
        self.overlay = True
        self.frame_no = 0
        self.alarm_until = 0.0
        self.detected = False

        # Данные с борта
        self.video = VideoClient(enabled=not offline)
        self.video.start()
        self.watcher = ProtocolWatcher()
        self.protocol: dict | None = None

        if logger:
            logger.info("Запуск АРМ Оператора ЦУКС 'Око МЧС' (тактический пульт v2.0)")
        if core_log_event:
            core_log_event("GCS_UI", "OPERATOR_ACTION", {"action": "APP_START", "status": "INITIALIZED"})

        self._build_ui()
        self._bind_keys()

        self.log("Пульт ЦУКС «Око МЧС» запущен. Ожидание данных с борта…")
        if offline:
            self.log("Режим без видеосервера: используется синтетический полигон.")
        if autostart:
            self.after(800, self.toggle_mission)

        self.after(40, self.update_loop)

    # -- построение интерфейса -------------------------------------------

    def _fit_window(self) -> None:
        """Подгоняет окно под реальный экран (масштаб Windows 125-150 %)."""
        self.update_idletasks()
        screen_w = self.winfo_screenwidth()
        screen_h = self.winfo_screenheight()
        width = max(1100, min(1400, screen_w - 60))
        height = max(720, min(900, screen_h - 100))
        x = max(0, (screen_w - width) // 2)
        y = max(0, (screen_h - height) // 3)
        self.geometry(f"{width}x{height}+{x}+{y}")
        self.minsize(1100, 720)

    def _card(self, parent, **kwargs) -> ctk.CTkFrame:
        """Карточка с одинаковым оформлением."""
        return ctk.CTkFrame(
            parent,
            fg_color=COLORS["panel"],
            corner_radius=10,
            border_width=1,
            border_color=COLORS["border"],
            **kwargs,
        )

    def _build_ui(self) -> None:
        """Собирает шапку, рабочую область и нижнюю панель."""
        self._build_header()

        body = ctk.CTkFrame(self, fg_color="transparent")
        body.pack(fill="both", expand=True, padx=12, pady=(8, 4))
        body.grid_columnconfigure(0, weight=3, uniform="body")
        body.grid_columnconfigure(1, weight=2, uniform="body")
        body.grid_rowconfigure(0, weight=1)

        self._build_map(body)
        self._build_camera(body)
        self._build_footer()

    def _build_header(self) -> None:
        """Верхняя строка: бренд, фаза миссии, статус связи, часы."""
        header = ctk.CTkFrame(self, fg_color=COLORS["panel_alt"], corner_radius=0, height=64)
        header.pack(fill="x")
        header.pack_propagate(False)

        brand = ctk.CTkFrame(header, fg_color="transparent")
        brand.pack(side="left", padx=18, pady=8)

        ctk.CTkLabel(
            brand, text="◉ ОКО МЧС", font=ctk.CTkFont(FONT_UI, 22, "bold"), text_color=COLORS["accent"]
        ).pack(side="left")
        ctk.CTkLabel(
            brand,
            text="  АРМ ОПЕРАТОРА ЦУКС  •  автономный БПЛА-мониторинг ЧС",
            font=ctk.CTkFont(FONT_UI, 12),
            text_color=COLORS["muted"],
        ).pack(side="left", pady=(6, 0))

        right = ctk.CTkFrame(header, fg_color="transparent")
        right.pack(side="right", padx=16, pady=10)

        self.lbl_clock = ctk.CTkLabel(
            right, text=now_str(), font=ctk.CTkFont(FONT_MONO, 13, "bold"), text_color=COLORS["muted"]
        )
        self.lbl_clock.pack(side="right", padx=(10, 0))

        self.lbl_link = ctk.CTkLabel(
            right,
            text="● СВЯЗЬ: ПОИСК",
            font=ctk.CTkFont(FONT_UI, 12, "bold"),
            fg_color=COLORS["panel"],
            corner_radius=6,
            padx=12,
            pady=5,
            text_color=COLORS["muted"],
        )
        self.lbl_link.pack(side="right", padx=6)

        self.lbl_phase = ctk.CTkLabel(
            right,
            text="ДЕЖУРСТВО",
            font=ctk.CTkFont(FONT_UI, 12, "bold"),
            fg_color=COLORS["panel"],
            corner_radius=6,
            padx=12,
            pady=5,
            text_color=COLORS["info"],
        )
        self.lbl_phase.pack(side="right", padx=6)

        self.lbl_threat = ctk.CTkLabel(
            right,
            text="УГРОЗА: НЕТ",
            font=ctk.CTkFont(FONT_UI, 12, "bold"),
            fg_color=COLORS["panel"],
            corner_radius=6,
            padx=12,
            pady=5,
            text_color=COLORS["ok"],
        )
        self.lbl_threat.pack(side="right", padx=6)

    def _build_map(self, parent) -> None:
        """Левая колонка: тактическая карта и выбор объекта."""
        card = self._card(parent)
        card.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        head = ctk.CTkFrame(card, fg_color="transparent")
        head.pack(fill="x", padx=12, pady=(10, 4))

        ctk.CTkLabel(
            head, text="▣ ТАКТИЧЕСКАЯ КАРТА ПОЛИГОНА", font=ctk.CTkFont(FONT_UI, 12, "bold"),
            text_color=COLORS["text"],
        ).pack(side="left")

        self.selector = ctk.CTkSegmentedButton(
            head,
            values=[HAZARDS_DB[key]["short"] for key in HAZARDS_DB],
            command=self.on_hazard_select,
            font=ctk.CTkFont(FONT_UI, 11, "bold"),
            selected_color=COLORS["accent"],
            selected_hover_color=COLORS["accent_dark"],
            height=26,
        )
        self.selector.set(HAZARDS_DB[self.hazard_key]["short"])
        self.selector.pack(side="right")

        self.map_canvas = tk.Canvas(card, bg=COLORS["panel_alt"], highlightthickness=0, bd=0)
        self.map_canvas.pack(fill="both", expand=True, padx=12, pady=(0, 12))

    def _build_camera(self, parent) -> None:
        """Правая колонка: бортовая камера и режимы съёмки."""
        card = self._card(parent)
        card.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        head = ctk.CTkFrame(card, fg_color="transparent")
        head.pack(fill="x", padx=12, pady=(10, 4))

        ctk.CTkLabel(
            head, text="▣ БОРТОВАЯ КАМЕРА", font=ctk.CTkFont(FONT_UI, 12, "bold"),
            text_color=COLORS["text"],
        ).pack(side="left")

        self.btn_overlay = ctk.CTkButton(
            head, text="ИИ-ОВЕРЛЕЙ", width=110, height=26,
            font=ctk.CTkFont(FONT_UI, 11, "bold"),
            fg_color=COLORS["panel_alt"], hover_color=COLORS["border"], text_color=COLORS["ok"],
            command=self.toggle_overlay,
        )
        self.btn_overlay.pack(side="right", padx=(6, 0))

        self.btn_thermal = ctk.CTkButton(
            head, text="ТЕПЛОВИЗОР", width=120, height=26,
            font=ctk.CTkFont(FONT_UI, 11, "bold"),
            fg_color=COLORS["panel_alt"], hover_color=COLORS["border"], text_color=COLORS["text"],
            command=self.toggle_thermal,
        )
        self.btn_thermal.pack(side="right")

        self.cam_canvas = tk.Canvas(card, bg="#05080C", highlightthickness=0, bd=0)
        self.cam_canvas.pack(fill="both", expand=True, padx=12, pady=(0, 6))

        foot = ctk.CTkFrame(card, fg_color="transparent")
        foot.pack(fill="x", padx=12, pady=(0, 10))

        self.lbl_mode = ctk.CTkLabel(
            foot, text="РЕЖИМ: ОПТИЧЕСКИЙ 4K", font=ctk.CTkFont(FONT_MONO, 11), text_color=COLORS["muted"]
        )
        self.lbl_mode.pack(side="left")

        self.lbl_fps = ctk.CTkLabel(
            foot, text="FPS: —", font=ctk.CTkFont(FONT_MONO, 11), text_color=COLORS["muted"]
        )
        self.lbl_fps.pack(side="right")

    def _build_footer(self) -> None:
        """Нижняя панель: телеметрия, карточка инцидента, журнал и кнопки."""
        footer = ctk.CTkFrame(self, fg_color="transparent", height=250)
        footer.pack(fill="x", padx=12, pady=(2, 12))
        footer.grid_columnconfigure(0, weight=0, minsize=330)
        footer.grid_columnconfigure(1, weight=2, uniform="footer")
        footer.grid_columnconfigure(2, weight=3, uniform="footer")
        footer.grid_rowconfigure(0, weight=1)

        self._build_telemetry(footer)
        self._build_incident_card(footer)
        self._build_log(footer)

    def _build_telemetry(self, parent) -> None:
        """Карточка телеметрии."""
        card = self._card(parent)
        card.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        ctk.CTkLabel(
            card, text="ТЕЛЕМЕТРИЯ БПЛА", font=ctk.CTkFont(FONT_UI, 11, "bold"), text_color=COLORS["muted"]
        ).pack(anchor="w", padx=14, pady=(10, 4))

        grid = ctk.CTkFrame(card, fg_color="transparent")
        grid.pack(fill="both", expand=True, padx=14, pady=(0, 10))
        grid.grid_columnconfigure(1, weight=1)

        self.tele_labels = {}
        rows = (
            ("alt", "ВЫСОТА", "0,0 м"),
            ("speed", "СКОРОСТЬ", "0,0 м/с"),
            ("coords", "КООРДИНАТЫ", "X 6,0 / Y 6,0 м"),
            ("heading", "КУРС", "0°"),
        )
        for index, (key, title, value) in enumerate(rows):
            ctk.CTkLabel(
                grid, text=title, font=ctk.CTkFont(FONT_UI, 11), text_color=COLORS["muted"], anchor="w"
            ).grid(row=index, column=0, sticky="w", pady=2)
            label = ctk.CTkLabel(
                grid, text=value, font=ctk.CTkFont(FONT_MONO, 12, "bold"), text_color=COLORS["text"], anchor="e"
            )
            label.grid(row=index, column=1, sticky="e", pady=2)
            self.tele_labels[key] = label

        ctk.CTkLabel(
            card, text="ЗАРЯД АКБ", font=ctk.CTkFont(FONT_UI, 11), text_color=COLORS["muted"]
        ).pack(anchor="w", padx=14)

        self.battery_bar = ctk.CTkProgressBar(
            card, height=12, corner_radius=6, progress_color=COLORS["ok"], fg_color=COLORS["panel_alt"]
        )
        self.battery_bar.pack(fill="x", padx=14, pady=(2, 2))
        self.battery_bar.set(0.98)

        self.lbl_battery = ctk.CTkLabel(
            card, text="98 %", font=ctk.CTkFont(FONT_MONO, 12, "bold"), text_color=COLORS["ok"]
        )
        self.lbl_battery.pack(anchor="e", padx=14, pady=(0, 8))

    def _build_incident_card(self, parent) -> None:
        """Карточка инцидента и кнопки управления."""
        card = self._card(parent)
        card.grid(row=0, column=1, sticky="nsew", padx=6)

        head = ctk.CTkFrame(card, fg_color="transparent")
        head.pack(fill="x", padx=14, pady=(10, 4))
        ctk.CTkLabel(
            head, text="ПРОТОКОЛ ИНЦИДЕНТА", font=ctk.CTkFont(FONT_UI, 11, "bold"), text_color=COLORS["muted"]
        ).pack(side="left")

        self.lbl_incident_id = ctk.CTkLabel(
            head, text="—", font=ctk.CTkFont(FONT_MONO, 12, "bold"), text_color=COLORS["info"]
        )
        self.lbl_incident_id.pack(side="right")

        self.lbl_incident_type = ctk.CTkLabel(
            card, text="Инцидентов не зафиксировано", font=ctk.CTkFont(FONT_UI, 15, "bold"),
            text_color=COLORS["text"], anchor="w",
        )
        self.lbl_incident_type.pack(fill="x", padx=14, pady=(2, 2))

        self.lbl_incident_meta = ctk.CTkLabel(
            card, text="Ожидание данных от борта", font=ctk.CTkFont(FONT_UI, 11),
            text_color=COLORS["muted"], anchor="w", justify="left",
        )
        self.lbl_incident_meta.pack(fill="x", padx=14, pady=(0, 4))

        self.lbl_incident_response = ctk.CTkLabel(
            card, text="", font=ctk.CTkFont(FONT_UI, 11), text_color=COLORS["warn"],
            anchor="w", justify="left", wraplength=420,
        )
        self.lbl_incident_response.pack(fill="x", padx=14, pady=(0, 6))

        buttons = ctk.CTkFrame(card, fg_color="transparent")
        buttons.pack(fill="x", padx=14, pady=(0, 12), side="bottom")

        self.btn_mission = ctk.CTkButton(
            buttons, text="▶ СТАРТ ПАТРУЛИРОВАНИЯ", height=38,
            font=ctk.CTkFont(FONT_UI, 12, "bold"),
            fg_color=COLORS["accent"], hover_color=COLORS["accent_dark"],
            command=self.toggle_mission,
        )
        self.btn_mission.pack(fill="x", pady=(0, 6))

        row = ctk.CTkFrame(buttons, fg_color="transparent")
        row.pack(fill="x")

        self.btn_json = ctk.CTkButton(
            row, text="JSON", height=30, font=ctk.CTkFont(FONT_UI, 11, "bold"),
            fg_color=COLORS["info"], hover_color="#0284C7", command=lambda: self.export_protocol("json"),
        )
        self.btn_json.pack(side="left", expand=True, fill="x", padx=(0, 3))

        self.btn_txt = ctk.CTkButton(
            row, text="БЛАНК TXT", height=30, font=ctk.CTkFont(FONT_UI, 11, "bold"),
            fg_color=COLORS["panel_alt"], hover_color=COLORS["border"], command=lambda: self.export_protocol("txt"),
        )
        self.btn_txt.pack(side="left", expand=True, fill="x", padx=2)

        self.btn_diag = ctk.CTkButton(
            row, text="📦 ЛОГИ", height=30, font=ctk.CTkFont(FONT_UI, 11, "bold"),
            fg_color="#37474F", hover_color="#455A64", command=self.build_diagnostic_bundle,
        )
        self.btn_diag.pack(side="left", expand=True, fill="x", padx=(2, 0))

        ctk.CTkLabel(
            card,
            text="Space — старт/RTL  ·  T — тепловизор  ·  O — оверлей  ·  E — экспорт",
            font=ctk.CTkFont(FONT_UI, 9),
            text_color=COLORS["muted"],
        ).pack(side="bottom", pady=(0, 4))

    def _build_log(self, parent) -> None:
        """Карточка журнала оперативного дежурного."""
        card = self._card(parent)
        card.grid(row=0, column=2, sticky="nsew", padx=(6, 0))

        ctk.CTkLabel(
            card, text="ЖУРНАЛ ОПЕРАТИВНОГО ДЕЖУРНОГО", font=ctk.CTkFont(FONT_UI, 11, "bold"),
            text_color=COLORS["muted"],
        ).pack(anchor="w", padx=14, pady=(10, 4))

        self.log_box = ctk.CTkTextbox(
            card, font=ctk.CTkFont(FONT_MONO, 11), fg_color=COLORS["panel_alt"],
            corner_radius=8, wrap="word",
        )
        self.log_box.pack(fill="both", expand=True, padx=14, pady=(0, 12))
        self.log_box.configure(state="disabled")

    # -- действия оператора -----------------------------------------------

    def _bind_keys(self) -> None:
        """Горячие клавиши."""
        self.bind("<space>", lambda _event: self.toggle_mission())
        self.bind("<Key-t>", lambda _event: self.toggle_thermal())
        self.bind("<Key-o>", lambda _event: self.toggle_overlay())
        self.bind("<Key-e>", lambda _event: self.export_protocol("json"))
        self.bind("<Key-d>", lambda _event: self.build_diagnostic_bundle())
        self.bind("<Control-d>", lambda _event: self.build_diagnostic_bundle())
        self.bind("<F11>", lambda _event: self.attributes("-fullscreen", True))
        self.bind("<Escape>", lambda _event: self.attributes("-fullscreen", False))

    def log(self, text: str, level: str = "INFO") -> None:
        """Добавляет строку в журнал дежурного и в подсистему логирования mchs_core."""
        line = f"[{now_str()}] {text}\n"

        def append():
            if self.log_box is not None:
                self.log_box.configure(state="normal")
                self.log_box.insert("end", line)
                self.log_box.see("end")
                self.log_box.configure(state="disabled")

        try:
            self.after(0, append)
        except Exception:
            pass

        if logger:
            lvl = level.upper()
            if lvl == "WARNING":
                logger.warning(text)
            elif lvl == "ERROR":
                logger.error(text)
            elif lvl == "CRITICAL":
                logger.critical(text)
            else:
                logger.info(text)

    def on_hazard_select(self, value: str) -> None:
        """Переключение демонстрационного объекта."""
        for key, hazard in HAZARDS_DB.items():
            if hazard["short"] == value:
                self.hazard_key = key
                self.log(f"Объект инспекции: {hazard['title']} ({hazard['details']}).")
                if core_log_event:
                    core_log_event("GCS_UI", "OPERATOR_ACTION", {
                        "action": "SELECT_HAZARD",
                        "hazard_key": key,
                        "title": hazard["title"],
                        "threat": hazard["threat"],
                    })
                return

    def toggle_thermal(self) -> None:
        """Включает и выключает палитру тепловизора."""
        self.thermal = not self.thermal
        if self.thermal:
            self.btn_thermal.configure(fg_color="#6B21A8", text_color="#FFFFFF", text="ОПТИКА 4K")
            self.lbl_mode.configure(text="РЕЖИМ: ТЕПЛОВИЗОР LWIR 8–14 мкм", text_color=COLORS["warn"])
            self.log("Камера переключена в тепловизионный режим (FLIR LWIR).")
        else:
            self.btn_thermal.configure(fg_color=COLORS["panel_alt"], text="ТЕПЛОВИЗОР")
            self.lbl_mode.configure(text="РЕЖИМ: ОПТИЧЕСКИЙ 4K", text_color=COLORS["muted"])
            self.log("Камера переключена в оптический режим (Sony IMX 4K).")
        if core_log_event:
            core_log_event("GCS_UI", "OPERATOR_ACTION", {
                "action": "TOGGLE_THERMAL",
                "thermal": self.thermal,
                "sensor": "FLIR LWIR" if self.thermal else "Sony IMX 4K",
            })

    def toggle_overlay(self) -> None:
        """Включает и выключает ИИ-оверлей на видео."""
        self.overlay = not self.overlay
        color = COLORS["ok"] if self.overlay else COLORS["muted"]
        self.btn_overlay.configure(text_color=color)
        self.log(f"ИИ-оверлей детекции: {'включён' if self.overlay else 'выключен'}.")
        if core_log_event:
            core_log_event("GCS_UI", "OPERATOR_ACTION", {
                "action": "TOGGLE_OVERLAY",
                "overlay": self.overlay,
            })

    def toggle_mission(self) -> None:
        """Старт патрулирования и аварийный возврат (RTL)."""
        if not self.mission_active:
            self.mission_active = True
            self.detected = False
            self.altitude = 0.0
            self.phase = "ВЗЛЁТ И ПАТРУЛИРОВАНИЕ"
            self.btn_mission.configure(text="■ АВАРИЙНЫЙ ВОЗВРАТ (RTL)", fg_color=COLORS["crit"])
            self.log("Команда: взлёт на эшелон 8,0 м, начало патрулирования сектора.")
        else:
            self.mission_active = False
            self.phase = "ВОЗВРАТ НА БАЗУ (RTL)"
            self.btn_mission.configure(text="▶ СТАРТ ПАТРУЛИРОВАНИЯ", fg_color=COLORS["accent"])
            self.log("Команда: возврат на базу (RTL), посадка в точке старта.")
        if core_log_event:
            core_log_event("GCS_UI", "OPERATOR_ACTION", {
                "action": "START_PATROL" if self.mission_active else "ABORT_RTL",
                "phase": self.phase,
                "altitude_m": round(self.altitude, 1),
                "battery_pct": int(self.battery),
            })

    # -- протокол ----------------------------------------------------------

    def apply_protocol(self, protocol: dict) -> None:
        """Показывает новый протокол: тревога, карточка, подсветка на карте."""
        self.protocol = protocol
        threat = protocol["threat_level"]
        label, color = THREAT_LABELS.get(threat, (threat, COLORS["warn"]))

        self.lbl_threat.configure(text=f"УГРОЗА: {label}", fg_color=color, text_color="#0B0F14")
        self.lbl_incident_id.configure(text=protocol["incident_id"], text_color=color)
        self.lbl_incident_type.configure(text=protocol["hazard_type"], text_color=color)

        line1 = f"Уровень: {label}"
        if protocol["area_sq_m"]:
            line1 += f"   •   Площадь: {protocol['area_sq_m']:.1f} м²"
        coords = []
        if protocol["latitude"] is not None and protocol["longitude"] is not None:
            coords.append(f"WGS-84: {protocol['latitude']:.5f}, {protocol['longitude']:.5f}")
        if protocol["x"] is not None and protocol["y"] is not None:
            coords.append(f"полигон: X {protocol['x']:.1f} / Y {protocol['y']:.1f} м")
        self.lbl_incident_meta.configure(text=line1 + "\n" + "   •   ".join(coords))
        self.lbl_incident_response.configure(text=protocol["recommended_response"] or "")

        self.alarm_until = time.time() + 8.0
        self.log(f"⚠ ПРОТОКОЛ {protocol['incident_id']}: {protocol['hazard_type']}, {label}.")
        self._beep()
        if core_log_event:
            core_log_event("GCS_UI", "OPERATOR_ACTION", {
                "action": "PROTOCOL_APPLIED",
                "incident_id": protocol.get("incident_id"),
                "hazard_type": protocol.get("hazard_type"),
                "threat_level": protocol.get("threat_level"),
            })

        # Если пришёл протокол, значит борт на месте — показываем инспекцию.
        self.phase = "ИНСПЕКЦИЯ ОЧАГА"
        # Телеметрию берём из протокола только когда своего полёта нет:
        # иначе цифры борта «дёргали» бы демонстрационную модель.
        if not self.mission_active:
            if protocol["altitude"]:
                self.altitude = float(protocol["altitude"])
            if protocol["battery"]:
                self.battery = float(protocol["battery"])

    def _beep(self) -> None:
        """Короткий звуковой сигнал тревоги (Windows)."""
        if winsound is None:
            return
        try:
            winsound.Beep(1400, 180)
            winsound.Beep(1000, 180)
        except Exception:
            pass

    def build_protocol_dict(self) -> dict:
        """Собирает протокол для ручного экспорта (схема совпадает с автопилотом)."""
        hazard = HAZARDS_DB[self.hazard_key]
        stamp = datetime.datetime.now()
        return {
            "incident_id": f"INC-{stamp:%Y}-{stamp:%H%M}",
            "timestamp": stamp.isoformat(timespec="seconds"),
            "timestamp_msk": stamp.strftime("%d.%m.%Y %H:%M:%S"),
            "station": "ЦУКС ГУ МЧС России",
            "hazard_type": hazard["title"].upper(),
            "hazard_code": self.hazard_key,
            "threat_level": hazard["threat"],
            "estimated_area_sq_m": hazard["area_sq_m"],
            "target_geo": {
                "abs_x": round(self.drone_x, 1),
                "abs_y": round(self.drone_y, 1),
                "rel_x": 0.0,
                "rel_y": 0.0,
            },
            "confidence": 0.96,
            "drone_telemetry": {
                "drone_id": "MCHS-UAV-04",
                "altitude_m": round(self.altitude, 1),
                "battery_percent": round(self.battery, 0),
                "grid_x_m": round(self.drone_x, 1),
                "grid_y_m": round(self.drone_y, 1),
            },
            "recommended_response": hazard["response"],
            "recommended_actions": [
                "Передать координаты в ЦУКС и ЕДДС района",
                "Оповестить оперативного дежурного ПСЧ",
                f"Учесть характер ЧС: {hazard['details']}",
            ],
            "protocol_version": "3.0",
        }

    def build_diagnostic_bundle(self) -> None:
        """Сборка полного диагностического пакета логов и отчетов инцидента."""
        self.log("📦 Запуск экспресс-диагностики и сборки лог-пакета...")
        try:
            if create_diagnostic_bundle:
                bundle_path = create_diagnostic_bundle()
            else:
                from scripts.diagnose_logs import create_diagnostic_bundle as cdb
                bundle_path = cdb()
            msg = f"✅ Диагностический бандл собран: {bundle_path.name}"
            self.log(msg)
            if logger:
                logger.info(f"Сформирован аварийный архив: {bundle_path}")
            if core_log_event:
                core_log_event("GCS_UI", "OPERATOR_ACTION", {
                    "action": "BUILD_DIAGNOSTIC_BUNDLE",
                    "status": "SUCCESS",
                    "bundle_path": str(bundle_path),
                    "bundle_name": bundle_path.name,
                })
        except Exception as e:
            err_msg = f"❌ Ошибка формирования лог-бандла: {e}"
            self.log(err_msg, level="ERROR")
            if core_log_event:
                core_log_event("GCS_UI", "OPERATOR_ACTION", {
                    "action": "BUILD_DIAGNOSTIC_BUNDLE",
                    "status": "ERROR",
                    "error": str(e),
                }, level="ERROR")

    def export_protocol(self, kind: str = "json") -> None:
        """Сохраняет протокол в reports/ (JSON или печатный бланк TXT)."""
        data = self.build_protocol_dict()
        out_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "reports"
        )
        try:
            os.makedirs(out_dir, exist_ok=True)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            if kind == "json":
                path = os.path.join(out_dir, f"incident_protocol_{stamp}.json")
                with open(path, "w", encoding="utf-8") as handle:
                    json.dump(data, handle, ensure_ascii=False, indent=2)
            else:
                path = os.path.join(out_dir, f"incident_protocol_{stamp}.txt")
                with open(path, "w", encoding="utf-8") as handle:
                    handle.write(self.render_blank(data))
            self.log(f"Протокол сохранён: {path}")
            if core_log_event:
                core_log_event("GCS_UI", "OPERATOR_ACTION", {
                    "action": "EXPORT_PROTOCOL",
                    "status": "SUCCESS",
                    "path": path,
                    "kind": kind,
                })
        except OSError as error:
            self.log(f"Не удалось сохранить протокол: {error}", level="ERROR")
            if core_log_event:
                core_log_event("GCS_UI", "OPERATOR_ACTION", {
                    "action": "EXPORT_PROTOCOL",
                    "status": "ERROR",
                    "error": str(error),
                }, level="ERROR")

    @staticmethod
    def render_blank(data: dict) -> str:
        """Печатный бланк оперативного дежурного."""
        geo = data["target_geo"]
        telemetry = data["drone_telemetry"]
        lines = [
            "=" * 72,
            f"ПРОТОКОЛ ИНЦИДЕНТА № {data['incident_id']}".center(72),
            data["station"].center(72),
            "=" * 72,
            f"Дата и время фиксации ....... {data['timestamp_msk']} (МСК)",
            f"Тип чрезвычайной ситуации ... {data['hazard_type']}",
            f"Уровень угрозы .............. {data['threat_level']}",
            f"Координаты очага (X, Y) ..... {geo['abs_x']:.1f} м, {geo['abs_y']:.1f} м",
            f"Площадь поражения ........... {data['estimated_area_sq_m']:.1f} м²",
            f"Борт ........................ {telemetry['drone_id']}",
            f"Высота / заряд АКБ .......... {telemetry['altitude_m']:.1f} м / {telemetry['battery_percent']:.0f} %",
            "-" * 72,
            "Рекомендуемый наряд спасательных сил:",
            "   " + data["recommended_response"],
            "-" * 72,
            "Рекомендуемые действия:",
        ]
        lines += [f"   {index}. {action}" for index, action in enumerate(data["recommended_actions"], 1)]
        lines += [
            "=" * 72,
            "Оперативный дежурный ЦУКС: ______________ / ____________________ /",
            "Документ сформирован автоматически АРМ «Око МЧС»",
            "=" * 72,
            "",
        ]
        return "\n".join(lines)

    # -- главный цикл ------------------------------------------------------

    def update_loop(self) -> None:
        """Обновляет данные, карту и видео ~25 раз в секунду."""
        try:
            self.frame_no += 1

            protocol = self.watcher.poll()
            if protocol is not None:
                self.apply_protocol(protocol)

            self._step_flight_model()
            self._refresh_labels()
            self._draw_map()
            self._draw_camera()
        except Exception as error:  # пульт не должен падать из-за одного кадра
            if self.frame_no % 100 == 0:
                self.log(f"Сбой отрисовки: {type(error).__name__}: {error}")

        self.after(40, self.update_loop)

    def _step_flight_model(self) -> None:
        """Простая модель полёта для демонстрации, пока нет телеметрии с борта."""
        hazard = HAZARDS_DB[self.hazard_key]

        if self.mission_active:
            if self.altitude < 8.0:
                self.altitude = min(8.0, self.altitude + 0.35)
                self.speed = 1.2
                self.phase = "НАБОР ВЫСОТЫ 8,0 м"
            else:
                dx, dy = hazard["x"] - self.drone_x, hazard["y"] - self.drone_y
                distance = math.hypot(dx, dy)
                if distance > 2.5:
                    step = min(2.2, distance)
                    self.drone_x += dx / distance * step
                    self.drone_y += dy / distance * step
                    self.heading_deg = math.degrees(math.atan2(dy, dx)) % 360.0
                    self.speed = 12.0
                    self.phase = f"ПАТРУЛИРОВАНИЕ: КУРС НА {hazard['short']}"
                else:
                    self.speed = 0.0
                    self.phase = f"ИНСПЕКЦИЯ: {hazard['short']}"
                    if not self.detected:
                        self.detected = True
                        self._simulate_detection()
            self.battery = max(8.0, self.battery - 0.012)
        else:
            if self.altitude > 0.0:
                self.altitude = max(0.0, self.altitude - 0.45)
                self.drone_x += (0.0 - self.drone_x) * 0.04
                self.drone_y += (0.0 - self.drone_y) * 0.04
                self.speed = 4.0
                self.phase = "ВОЗВРАТ И ПОСАДКА (RTL)"
            else:
                self.speed = 0.0
                if self.alarm_until < time.time():
                    self.phase = "ДЕЖУРСТВО НА БАЗЕ"

        if self.altitude > 0.5:
            self.trail.append((self.drone_x, self.drone_y))
            if len(self.trail) > 400:
                self.trail.pop(0)

    def _simulate_detection(self) -> None:
        """Демонстрационная детекция: пульт сам формирует протокол по очагу.

        Нужна, чтобы АРМ можно было показать без запущенного борта: дрон
        выходит на объект, пульт пишет ``reports/incident_protocol.json`` и
        включает тревогу точно так же, как при реальном событии от зрения.
        """
        hazard = HAZARDS_DB[self.hazard_key]
        data = self.build_protocol_dict()
        data["estimated_area_sq_m"] = hazard["area_sq_m"]
        data["threat_level"] = hazard["threat"]
        data["target_geo"].update(
            {"abs_x": round(hazard["x"], 1), "abs_y": round(hazard["y"], 1)}
        )

        out_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "reports"
        )
        try:
            os.makedirs(out_dir, exist_ok=True)
            path = os.path.join(out_dir, "incident_protocol.json")
            with open(path, "w", encoding="utf-8") as handle:
                json.dump(data, handle, ensure_ascii=False, indent=2)
            # Файл написали мы сами — watcher не должен поднимать тревогу второй раз.
            self.watcher.signature = (os.path.abspath(path), round(os.path.getmtime(path), 3))
        except OSError as error:
            self.log(f"Не удалось сохранить протокол: {error}")

        self.apply_protocol(normalize_protocol(data, locals().get("path", "")))

    def _refresh_labels(self) -> None:
        """Обновляет плашки, телеметрию и индикатор связи."""
        self.lbl_clock.configure(text=now_str())

        link = self.video.status
        link_color = COLORS["ok"] if link in ("MJPEG", "SNAPSHOT") else COLORS["warn"]
        self.lbl_link.configure(text=f"● СВЯЗЬ: {link}", text_color=link_color)
        self.lbl_phase.configure(text=self.phase.split(":")[0])

        self.tele_labels["alt"].configure(text=f"{self.altitude:.1f} м")
        self.tele_labels["speed"].configure(text=f"{self.speed:.1f} м/с")
        self.tele_labels["coords"].configure(text=f"X {self.drone_x:.1f} / Y {self.drone_y:.1f} м")
        self.tele_labels["heading"].configure(text=f"{self.heading_deg:.0f}°")

        self.battery_bar.set(max(0.0, min(1.0, self.battery / 100.0)))
        battery_color = COLORS["ok"] if self.battery > 40 else (COLORS["warn"] if self.battery > 20 else COLORS["crit"])
        self.battery_bar.configure(progress_color=battery_color)
        self.lbl_battery.configure(text=f"{self.battery:.0f} %", text_color=battery_color)

        if self.alarm_until > time.time():
            # Мигание тревоги в шапке.
            if int(self.frame_no / 8) % 2 == 0:
                self.lbl_threat.configure(fg_color=COLORS["crit"], text_color="#0B0F14")
            else:
                self.lbl_threat.configure(fg_color=COLORS["panel"], text_color=COLORS["crit"])

        fps = f"{self.video.fps:.0f}" if self.video.fps else "—"
        self.lbl_fps.configure(text=f"FPS: {fps}  ·  {self.video.status}")

    # -- отрисовка ---------------------------------------------------------

    def _world_to_canvas(self, x_m: float, y_m: float) -> tuple[float, float]:
        """Переводит метры полигона в пиксели карты (ось Y направлена вверх)."""
        width = max(1, self.map_canvas.winfo_width())
        height = max(1, self.map_canvas.winfo_height())
        left, top, right, bottom = 34, 26, width - 18, height - 30
        px = left + ((x_m + WORLD_HALF_M) / WORLD_WIDTH_M) * (right - left)
        py = top + ((WORLD_HALF_M - y_m) / WORLD_HEIGHT_M) * (bottom - top)
        return px, py

    def _draw_map(self) -> None:
        """Рисует полигон, маршрут, очаги, трек и борт."""
        canvas = self.map_canvas
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 80 or height < 80:
            return

        canvas.delete("all")
        left, top, right, bottom = 34, 26, width - 18, height - 30

        # Сетка каждые 25 м; подписи от -100 до 100
        for meter in range(-100, 101, 25):
            px, _ = self._world_to_canvas(float(meter), 0.0)
            canvas.create_line(px, top, px, bottom, fill=COLORS["grid"])
            canvas.create_text(px, bottom + 12, text=f"{meter}", fill=COLORS["muted"],
                               font=(FONT_MONO, 8))
        for meter in range(-100, 101, 25):
            _, py = self._world_to_canvas(0.0, float(meter))
            canvas.create_line(left, py, right, py, fill=COLORS["grid"])
            canvas.create_text(left - 16, py, text=f"{meter}", fill=COLORS["muted"],
                               font=(FONT_MONO, 8))

        # Компас: север вверху карты
        canvas.create_line(left + 14, top + 12, left + 14, top + 34, fill=COLORS["muted"], arrow=tk.LAST)
        canvas.create_text(left + 22, top + 12, text="С", fill=COLORS["muted"], font=(FONT_UI, 8, "bold"))

        canvas.create_rectangle(left, top, right, bottom, outline="#26364A", dash=(4, 4))
        canvas.create_text(left + 6, top - 10, text="АРЕНА 200×200 м · БАЗА В ЦЕНТРЕ",
                           fill=COLORS["muted"], font=(FONT_UI, 9, "bold"), anchor="w")

        # Маршрут облёта очагов (точки наблюдения из mchs_pilot_route.py)
        route = [(42.0, 28.0), (-76.0, 30.0), (27.0, -56.0), (50.0, -72.0), (0.0, 0.0)]
        route_points = []
        for waypoint_x, waypoint_y in route:
            route_points.extend(self._world_to_canvas(waypoint_x, waypoint_y))
        canvas.create_line(*route_points, fill="#1F3A5F", width=1, dash=(4, 6))
        for waypoint_x, waypoint_y in route[:-1]:
            wx, wy = self._world_to_canvas(waypoint_x, waypoint_y)
            canvas.create_oval(wx - 3, wy - 3, wx + 3, wy + 3, outline="#1F3A5F")

        # База
        base = self._world_to_canvas(0.0, 0.0)
        canvas.create_rectangle(base[0] - 10, base[1] - 10, base[0] + 10, base[1] + 10,
                                fill="#1D4ED8", outline="#60A5FA")
        canvas.create_text(base[0], base[1] + 20, text="БАЗА", fill="#93C5FD", font=(FONT_UI, 8, "bold"))

        # Очаги ЧС
        for key, hazard in HAZARDS_DB.items():
            px, py = self._world_to_canvas(hazard["x"], hazard["y"])
            selected = key == self.hazard_key
            radius = 14 if selected else 9
            if selected:
                radius += int(math.sin(self.frame_no * 0.25) * 3)
            canvas.create_oval(px - radius, py - radius, px + radius, py + radius,
                               fill=hazard["color"], outline="#FFFFFF" if selected else hazard["color"])
            canvas.create_text(px, py + radius + 10, text=hazard["short"], fill=hazard["color"],
                               font=(FONT_UI, 8, "bold"))

        # Отметка очага из протокола
        if self.protocol and self.protocol.get("x") is not None and self.protocol.get("y") is not None:
            px, py = self._world_to_canvas(float(self.protocol["x"]), float(self.protocol["y"]))
            canvas.create_oval(px - 24, py - 24, px + 24, py + 24, outline=COLORS["crit"], width=2)
            canvas.create_text(px, py - 32, text=self.protocol["incident_id"], fill=COLORS["crit"],
                               font=(FONT_MONO, 9, "bold"))

        # Трек
        if len(self.trail) > 1:
            points = []
            for x_m, y_m in self.trail:
                points.extend(self._world_to_canvas(x_m, y_m))
            canvas.create_line(*points, fill=COLORS["info"], width=2, smooth=True)

        # Конус обзора и борт
        dx, dy = self._world_to_canvas(self.drone_x, self.drone_y)
        if self.altitude > 1.0:
            cone = 8 + self.altitude * 2.4
            canvas.create_oval(dx - cone, dy - cone, dx + cone, dy + cone,
                               outline="#0E7490", dash=(2, 5))

        radius = 11
        angle = math.radians(self.heading_deg)
        nose_x = dx + math.cos(angle) * (radius + 6)
        nose_y = dy - math.sin(angle) * (radius + 6)
        canvas.create_line(dx, dy, nose_x, nose_y, fill=COLORS["accent"], width=2)

        canvas.create_line(dx - radius, dy, dx + radius, dy, fill=COLORS["drone"], width=2)
        canvas.create_line(dx, dy - radius, dx, dy + radius, fill=COLORS["drone"], width=2)
        canvas.create_oval(dx - 5, dy - 5, dx + 5, dy + 5, fill=COLORS["accent"], outline="#FFFFFF")

        # Масштабная линейка в правом верхнем углу
        scale_x0, scale_x1, scale_y = right - 96, right - 26, top + 14
        canvas.create_line(scale_x0, scale_y, scale_x1, scale_y, fill=COLORS["text"], width=2)
        canvas.create_line(scale_x0, scale_y - 4, scale_x0, scale_y + 4, fill=COLORS["text"])
        canvas.create_line(scale_x1, scale_y - 4, scale_x1, scale_y + 4, fill=COLORS["text"])
        canvas.create_text((scale_x0 + scale_x1) / 2, scale_y - 10, text="10 м",
                           fill=COLORS["muted"], font=(FONT_UI, 8))

    def _draw_camera(self) -> None:
        """Выводит кадр с камеры (или синтетическую сцену) в правую панель."""
        canvas = self.cam_canvas
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 80 or height < 80:
            return

        offset_x = offset_y = 0
        frame = self.video.snapshot()
        if frame is None:
            frame = self._render_synthetic_scene(width, height)
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)
        else:
            in_h, in_w = frame.shape[:2]
            scale = min(width / in_w, height / in_h)
            new_w, new_h = max(1, int(in_w * scale)), max(1, int(in_h * scale))
            frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            offset_x = (width - new_w) // 2
            offset_y = (height - new_h) // 2

        if self.thermal:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
            frame = cv2.applyColorMap(clahe.apply(gray), cv2.COLORMAP_INFERNO)

        if self.overlay:
            frame = self._draw_hud(frame, frame.shape[1], frame.shape[0])

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = ImageTk.PhotoImage(Image.fromarray(rgb))
        canvas.create_image(offset_x, offset_y, anchor="nw", image=image)
        canvas.image = image  # защита от сборщика мусора

    def _render_synthetic_scene(self, width: int, height: int) -> np.ndarray:
        """Синтетический кадр полигона: лес, прогалина, очаг и дым.

        Рисуется, когда видеопоток с борта недоступен — демонстрация не
        зависит от запущенного Gazebo.
        """
        hazard = HAZARDS_DB[self.hazard_key]
        rng = np.random.default_rng(2026)
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[:] = (34, 48, 28)

        # Трава: мелкий шум и редкие тёмные пятна
        noise = rng.integers(0, 18, (height, width, 1), dtype=np.uint8)
        frame = cv2.add(frame, np.repeat(noise, 3, axis=2))
        for _ in range(120):
            x, y = int(rng.integers(0, width)), int(rng.integers(0, height))
            cv2.circle(frame, (x, y), int(rng.integers(8, 30)), (26, 40, 22), -1)

        # Кроны деревьев по краям кадра
        for _ in range(26):
            x = int(rng.integers(0, width))
            y = int(rng.integers(0, height))
            radius_tree = int(rng.integers(14, 34))
            cv2.circle(frame, (x, y), radius_tree, (18, 34, 18), -1)
            cv2.circle(frame, (x - radius_tree // 3, y - radius_tree // 3),
                       max(3, radius_tree // 2), (26, 48, 24), -1)

        distance = math.hypot(self.drone_x - hazard["x"], self.drone_y - hazard["y"])
        zoom = max(0.05, min(1.0, 1.0 - distance / 140.0))
        cx, cy = width // 2, height // 2

        if zoom > 0.18:
            radius = int(min(width, height) * (0.06 + 0.16 * zoom))
            radius = max(8, radius)
            key = self.hazard_key

            if key == "FIRE":
                self._draw_fire(frame, cx, cy, radius)
            elif key == "OIL":
                cv2.ellipse(frame, (cx, cy), (int(radius * 1.9), int(radius * 1.1)), 18, 0, 360,
                            (22, 26, 30), -1)
                cv2.ellipse(frame, (cx, cy), (int(radius * 1.3), int(radius * 0.75)), 18, 0, 360,
                            (34, 38, 44), -1)
                cv2.ellipse(frame, (cx - radius, cy - radius // 2),
                            (max(4, radius // 2), max(3, radius // 3)), 0, 0, 360, (70, 80, 92), 2)
            elif key == "FLOOD":
                cv2.ellipse(frame, (cx, cy), (int(radius * 2.2), int(radius * 1.4)), 0, 0, 360,
                            (150, 100, 30), -1)
                cv2.ellipse(frame, (cx, cy), (int(radius * 1.7), int(radius * 1.0)), 0, 0, 360,
                            (190, 140, 45), -1)
                cv2.ellipse(frame, (cx - radius, cy), (int(radius * 0.7), max(3, int(radius * 0.35))),
                            12, 0, 360, (220, 180, 80), -1)
            else:  # WASTE
                cv2.rectangle(frame, (cx - int(radius * 1.7), cy - int(radius * 0.9)),
                              (cx + int(radius * 1.7), cy + int(radius * 0.9)), (60, 58, 52), -1)
                for ox, oy, color in ((-radius, -radius // 2, (0, 190, 200)),
                                      (radius // 2, -radius // 2, (0, 130, 255)),
                                      (0, radius // 2, (40, 170, 50))):
                    cv2.rectangle(frame, (cx + ox - radius // 2, cy + oy - radius // 3),
                                  (cx + ox + radius // 2, cy + oy + radius // 3), color, -1)

        # Виньетка — кадр выглядит как оптика, а не как заливка
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.ellipse(mask, (cx, cy), (int(width * 0.62), int(height * 0.62)), 0, 0, 360, 255, -1)
        mask = cv2.GaussianBlur(mask, (0, 0), max(width, height) / 8.0)
        vignette = (mask.astype(np.float32) / 255.0)[:, :, None]
        return (frame.astype(np.float32) * (0.55 + 0.45 * vignette)).astype(np.uint8)

    @staticmethod
    def _draw_fire(frame: np.ndarray, cx: int, cy: int, radius: int) -> None:
        """Очаг пожара: дым, выгоревшая земля и пламя с градиентом."""
        # Дым рисуем полупрозрачным слоем
        smoke = frame.copy()
        cv2.circle(smoke, (cx + radius // 2, cy - int(radius * 1.6)),
                   max(6, int(radius * 1.3)), (120, 120, 120), -1)
        cv2.circle(smoke, (cx + radius, cy - int(radius * 2.3)),
                   max(5, int(radius * 0.9)), (95, 95, 95), -1)
        frame[:] = cv2.addWeighted(smoke, 0.35, frame, 0.65, 0)

        cv2.circle(frame, (cx, cy), max(6, int(radius * 1.25)), (26, 28, 32), -1)       # гарь
        cv2.circle(frame, (cx, cy), radius, (0, 55, 190), -1)                          # пламя
        cv2.circle(frame, (cx - radius // 4, cy + radius // 5),
                   max(4, int(radius * 0.7)), (0, 120, 255), -1)
        cv2.circle(frame, (cx - radius // 3, cy + radius // 3),
                   max(3, int(radius * 0.38)), (40, 220, 255), -1)

        # Отдельные языки пламени по краю очага
        for angle in range(0, 360, 45):
            offset_x = int(math.cos(math.radians(angle)) * radius * 0.85)
            offset_y = int(math.sin(math.radians(angle)) * radius * 0.85)
            cv2.circle(frame, (cx + offset_x, cy + offset_y), max(2, radius // 8), (0, 90, 255), -1)

    def _draw_hud(self, frame: np.ndarray, width: int, height: int) -> np.ndarray:
        """Тактический HUD: перекрестие, рамка детекции, служебные надписи."""
        result = frame.copy()
        height, width = result.shape[:2]
        hazard = HAZARDS_DB[self.hazard_key]
        distance = math.hypot(self.drone_x - hazard["x"], self.drone_y - hazard["y"])
        detected = self.altitude > 3.0 and distance < 25.0

        # Перекрестие
        cx, cy = width // 2, height // 2
        cv2.line(result, (cx - 16, cy), (cx + 16, cy), (0, 230, 255), 1)
        cv2.line(result, (cx, cy - 16), (cx, cy + 16), (0, 230, 255), 1)

        # Верхняя плашка
        mode = "THERMAL LWIR" if self.thermal else "OPTICAL 4K"
        cv2.putText(result, f"REC {now_str()}  {mode}  {width}x{height}", (12, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 255), 1)

        # Рамка детекции
        if detected:
            bw, bh = int(width * 0.22), int(height * 0.18)
            x1, y1, x2, y2 = cx - bw, cy - bh, cx + bw, cy + bh
            color = (0, 0, 255) if not self.thermal else (0, 255, 255)
            cv2.rectangle(result, (x1, y1), (x2, y2), color, 2)

            badge = f"{self.hazard_key}  {hazard['threat']}  {hazard['area_sq_m']:.0f} m2"
            cv2.rectangle(result, (x1, y1 - 22), (x1 + 300, y1), color, -1)
            cv2.putText(result, badge, (x1 + 6, y1 - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1)

        # Нижняя строка
        cv2.putText(result, f"ALT {self.altitude:.1f} m   BAT {self.battery:.0f}%   HDG {self.heading_deg:.0f}",
                    (12, height - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 220, 235), 1)
        return result


# ---------------------------------------------------------------------------
# Точка входа
# ---------------------------------------------------------------------------


def parse_args(argv=None) -> argparse.Namespace:
    """Аргументы командной строки."""
    parser = argparse.ArgumentParser(description="АРМ оператора ЦУКС «Око МЧС».")
    parser.add_argument("--offline", action="store_true", help="не обращаться к видеосерверу борта")
    parser.add_argument("--autostart", action="store_true", help="сразу начать демонстрационный полёт")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    """Запуск пульта."""
    args = parse_args(argv)
    app = OkoMchsApp(offline=args.offline, autostart=args.autostart)
    try:
        app.mainloop()
    finally:
        app.video.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())


# Совместимость с предыдущими скриптами и тестами
OkoMchsOperatorApp = OkoMchsApp
