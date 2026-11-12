#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""АРМ ОПЕРАТОРА ЦУКС «ОКО МЧС» — тактический пульт мониторинга ЧС.

Что умеет пульт:

1. **Живое видео с борта.** Основной режим — MJPEG-поток ``/stream`` с
   веб-сервера ноды зрения (порт 8080). Если поток недоступен, пульт
   переключается на опрос ``/snapshot.jpg``, а если и его нет — рисует
   синтетический полигон, чтобы демонстрация не зависела от Gazebo.
2. **Живая телеметрия.** Фоновый поток опрашивает ``/state.json`` (порт 8080):
   реальные координаты, высота, курс, скорость и трек БПЛА, фаза сканирования.
   Синтетическая модель полёта остаётся только резервом, когда борта нет.
3. **Мультиспектральная камера.** Оптический режим, тепловизор (палитра
   INFERNO) и ИИ-оверлей с рамкой детекции и подписями.
4. **Тактическая карта.** Арена 200×200 м, плановые ряды сканирования из
   ``config/mission.json``, маршрут облёта, трек БПЛА, очаги ЧС (их можно
   перетаскивать мышью — координаты уходят в Gazebo) и масштабная линейка.
5. **Телеметрия.** Высота, скорость, заряд АКБ, координаты, курс, фаза.
6. **Приём протокола ЦУКС.** Пульт сам замечает новый
   ``reports/incident_protocol.json`` (его пишет автопилот), включает
   тревогу, показывает карточку инцидента, подсвечивает очаг на карте и
   автоматически сохраняет снимок момента тревоги в ``reports/incidents``.
7. **Отчёты.** Снимок (``P``), запись видео MP4 (``R``), открытие папки
   отчётов в Проводнике (``Ctrl+O``); протокол в JSON и печатный бланк TXT.

Горячие клавиши: ``Space`` — старт/RTL, ``T`` — тепловизор, ``O`` — оверлей,
``P`` — снимок, ``R`` — запись видео, ``Ctrl+O`` — папка отчётов,
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
import queue
import random
import subprocess
import sys
import threading
import tempfile
import time
import urllib.request
from pathlib import Path

from wsl_paths import repo_wsl_path

# --- Подключение mchs_core и scripts для централизованного логирования и диагностики ---
REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "src"
SCRIPTS_DIR = REPO_ROOT / "scripts"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

#: Каталоги и файлы проекта: конфигурация (пишет Lead), отчёты, журналы.
CONFIG_DIR = REPO_ROOT / "config"
REPORTS_DIR = REPO_ROOT / "reports"
LOGS_DIR = REPO_ROOT / "logs"
HAZARDS_PATH = CONFIG_DIR / "hazards.json"
MISSION_PATH = CONFIG_DIR / "mission.json"
MISSION_CMD_PATH = REPORTS_DIR / "mission.cmd"


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

try:
    from mchs_core.video_reporter import MissionVideoReporter, VideoReportConfig
except ImportError:  # pragma: no cover
    MissionVideoReporter = None
    VideoReportConfig = None

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
    "HUMAN": {
        "title": "Пострадавший (человек)",
        "short": "ЧЕЛОВЕК",
        "threat": "CRITICAL",
        "color": "#00E5FF",
        "x": 15.0,
        "y": -22.0,
        "area_sq_m": 1.8,
        "details": "Обнаружен человек в сигнальном жилете, требуется эвакуация",
        "response": "Рекомендован наряд: поисково-спасательная группа 4 чел., санитарный вертолёт.",
    },
}

#: Встроенные значения — страховка, если config/hazards.json недоступен.
BUILTIN_HAZARDS = {key: dict(value) for key, value in HAZARDS_DB.items()}

#: Шапка-комментарий config/hazards.json (сохраняется при перетаскивании очагов).
HAZARDS_COMMENT = (
    "Очаги ЧС на арене 200x200 м. Файл — единый источник координат для АРМ, "
    "автопилота и генератора мира. Оператор может перетаскивать очаги прямо "
    "на карте АРМ: изменения сохраняются сюда и применяются в Gazebo через "
    "arena_controller.py."
)


def load_hazards(path: str | os.PathLike | None = None) -> dict:
    """Читает очаги из ``config/hazards.json``, дополняя встроенными значениями.

    Файл может отсутствовать или быть неполным — пульт в любом случае
    получает полный словарь очагов, иначе демонстрацию нечем показывать.
    Координаты и площади приводятся к float, чтобы перетаскивание работало
    даже с «рукописным» JSON.
    """
    hazards = {key: dict(value) for key, value in BUILTIN_HAZARDS.items()}
    file_path = Path(path) if path is not None else HAZARDS_PATH
    try:
        with open(file_path, "r", encoding="utf-8") as handle:
            raw = json.load(handle)
    except (OSError, ValueError):
        return hazards
    if not isinstance(raw, dict):
        return hazards

    for key, value in raw.items():
        key_upper = str(key).upper()
        if not isinstance(value, dict) or key_upper.startswith("_"):
            continue
        entry = hazards.setdefault(key_upper, {})
        for field, field_value in value.items():
            if not str(field).startswith("_"):
                entry[field] = field_value
        for coordinate in ("x", "y", "z", "area_sq_m"):
            if coordinate in entry:
                try:
                    entry[coordinate] = float(entry[coordinate])
                except (TypeError, ValueError):
                    entry[coordinate] = BUILTIN_HAZARDS.get(key_upper, {}).get(coordinate, 0.0)
    return hazards


def save_hazards(hazards: dict, path: str | os.PathLike | None = None) -> Path:
    """Сохраняет очаги в ``config/hazards.json`` (все поля, атомарно).

    Пишем через временный файл: генератор мира и автопилот могут читать
    ``config/hazards.json`` в этот же момент и не должны увидеть обрезанный JSON.
    """
    target = Path(path) if path is not None else HAZARDS_PATH
    payload = {"_comment": HAZARDS_COMMENT}
    try:
        with open(target, "r", encoding="utf-8") as handle:
            existing = json.load(handle)
        if isinstance(existing, dict):
            payload.update(existing)
    except (OSError, ValueError):
        pass
    for key, value in hazards.items():
        previous = payload.get(key)
        entry = dict(previous) if isinstance(previous, dict) else {}
        entry.update(value)
        payload[key] = entry
    target.parent.mkdir(parents=True, exist_ok=True)
    # Each writer owns its temporary file. Never truncate the published config,
    # even when Windows refuses replacement while a reader holds it open.
    tmp_path = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8",
                                         dir=target.parent, prefix=target.name + ".",
                                         suffix=".tmp", delete=False) as handle:
            tmp_path = handle.name
            json.dump(payload, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
        os.replace(tmp_path, target)
    finally:
        if tmp_path is not None:
            try:
                os.remove(tmp_path)
            except FileNotFoundError:
                pass
    return target


#: Порядок очагов для кнопки «🎲 СЛУЧАЙНО» (совпадает с HAZARDS_DB).
RANDOM_HAZARD_ORDER = ("FIRE", "OIL", "FLOOD", "WASTE")


def plan_random_hazards(
    drone_x: float,
    drone_y: float,
    keys=None,
    *,
    seed: int | None = None,
    ahead_min_m: float = 12.0,
    ahead_max_m: float = 58.0,
    lateral_m: float = 22.0,
    min_gap_m: float = 14.0,
) -> dict[str, tuple[float, float]]:
    """Раскидывает очаги ЧС в зону обзора бортовой камеры дрона.

    Камера смотрит строго в +X и вниз на 45°, поэтому очаги, поставленные
    в полосу ``X ∈ [дрон + ahead_min_m, дрон + ahead_max_m]``,
    ``Y ∈ [дрон ± lateral_m]``, детектор видит уже в следующем кадре —
    на демонстрации это доказывает детекцию за считанные секунды.

    Возвращает ``{ключ: (x, y)}``; координаты ограничены ареной ±95 м.
    Если свободного места нет (дрон у края), очаги ставятся с отступом.
    """
    keys = list(keys) if keys else list(RANDOM_HAZARD_ORDER)
    rng = random.Random(seed) if seed is not None else random.Random()
    limit = 95.0
    drone_x = float(drone_x)
    drone_y = float(drone_y)
    center_y = min(limit, max(-limit, drone_y))
    band_low = max(-limit, drone_x + ahead_min_m)
    band_high = min(limit, drone_x + ahead_max_m)
    if band_high < band_low:  # дрон у правого края: впереди места нет
        band_low = band_high = min(limit, max(-limit, drone_x + ahead_min_m))
    placed: dict[str, tuple[float, float]] = {}
    for key in keys:
        for attempt in range(400):
            x = band_low if band_low >= band_high else rng.uniform(band_low, band_high)
            if attempt < 250:
                y = center_y + rng.uniform(-lateral_m, lateral_m)
            else:  # тесно — расширяем полосу по Y на всю арену
                y = rng.uniform(-limit, limit)
            x = min(limit, max(-limit, x))
            y = min(limit, max(-limit, y))
            if any(math.hypot(x - px, y - py) < min_gap_m
                   for px, py in placed.values()):
                continue
            placed[key] = (round(x, 1), round(y, 1))
            break
        else:
            x = band_low
            step = (2.0 * lateral_m) / max(1, len(keys) - 1)
            y = min(limit, max(-limit, center_y - lateral_m + len(placed) * step))
            placed[key] = (round(x, 1), round(y, 1))
    return placed


#: Очаги ЧС: встроенные значения, перекрытые файлом config/hazards.json.
HAZARDS_DB.clear()
HAZARDS_DB.update(load_hazards())

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


def _as_float(value, fallback: float | None = None) -> float | None:
    """Мягкое приведение к float: в конфигурации и телеметрии бывает что угодно."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def resolve_repo_path(value, fallback: Path) -> Path:
    """Путь из конфигурации: относительный считается от корня репозитория."""
    if not value:
        return fallback
    path = Path(str(value))
    return path if path.is_absolute() else REPO_ROOT / path


def read_image_any(path: str | os.PathLike):
    """Читает изображение по пути (в т.ч. UNC/WSL) через numpy + cv2.

    ``cv2.imread`` на UNC-путях Windows иногда возвращает None, поэтому
    читаем байты файла и декодируем вручную.
    """
    try:
        target = Path(str(path))
        if not target.is_absolute():
            target = REPO_ROOT / target
        data = np.fromfile(str(target), dtype=np.uint8)
        if data.size == 0:
            return None
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    except Exception:  # noqa: BLE001 - битый файл не должен ломать отчёт
        return None


def windows_path(path: str | os.PathLike) -> str:
    """Путь в виде, понятном Windows: из WSL — UNC-путь, на Windows — как есть."""
    text = str(path)
    if sys.platform == "win32":
        return text
    if text.startswith("/"):
        distro = os.environ.get("WSL_DISTRO_NAME") or "Ubuntu-22.04"
        return f"\\\\wsl.localhost\\{distro}" + text.replace("/", "\\")
    return text


#: План полёта по умолчанию — совпадает с config/mission.json.
DEFAULT_MISSION = {
    "arena": {"width_m": WORLD_WIDTH_M, "height_m": WORLD_HEIGHT_M},
    "drone": {"altitude_m": 22.0, "cruise_speed_mps": 12.0, "arrive_radius_m": 3.0},
    "scan": {"x_min": -92.0, "x_max": 92.0, "y_min": -84.0, "y_max": 84.0, "row_step_m": 28.0},
    "reports_dir": "reports",
    "mission_cmd_file": "reports/mission.cmd",
}


def _deep_merge(base: dict, override: dict) -> dict:
    """Рекурсивно сливает конфигурацию: значения из ``override`` главнее."""
    result = {key: dict(value) if isinstance(value, dict) else value for key, value in base.items()}
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_mission(path: str | os.PathLike | None = None) -> dict:
    """Читает план полёта из ``config/mission.json`` с откатом на значения по умолчанию."""
    file_path = Path(path) if path is not None else MISSION_PATH
    try:
        with open(file_path, "r", encoding="utf-8") as handle:
            raw = json.load(handle)
    except (OSError, ValueError):
        return _deep_merge(DEFAULT_MISSION, {})
    if not isinstance(raw, dict):
        return _deep_merge(DEFAULT_MISSION, {})
    return _deep_merge(DEFAULT_MISSION, raw)


def scan_rows(mission: dict | None = None) -> list[list[tuple[float, float]]]:
    """Плановые ряды сканирования (boustrophedon) в метрах полигона.

    Каждый ряд — отрезок по оси X; чётные ряды идут слева направо, нечётные —
    справа налево, как змейка автопилота. Именно эти линии рисуются на карте
    пунктиром с подписями «РЯД i».
    """
    scan = (mission or {}).get("scan") or {}
    defaults = DEFAULT_MISSION["scan"]
    x_min = _as_float(scan.get("x_min"), defaults["x_min"]) or defaults["x_min"]
    x_max = _as_float(scan.get("x_max"), defaults["x_max"]) or defaults["x_max"]
    y_min = _as_float(scan.get("y_min"), defaults["y_min"]) or defaults["y_min"]
    y_max = _as_float(scan.get("y_max"), defaults["y_max"]) or defaults["y_max"]
    step = max(1.0, _as_float(scan.get("row_step_m"), defaults["row_step_m"]) or defaults["row_step_m"])

    rows: list[list[tuple[float, float]]] = []
    row_y = y_min
    index = 0
    while row_y <= y_max + 1e-6:
        if index % 2 == 0:
            rows.append([(x_min, row_y), (x_max, row_y)])
        else:
            rows.append([(x_max, row_y), (x_min, row_y)])
        index += 1
        row_y += step
    return rows


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
        "photo": str(data.get("photo") or ""),
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
        self._stop_event = threading.Event()

    def stop(self) -> None:
        """Просит поток завершиться."""
        self._stop_event.set()

    def snapshot(self):
        """Копия последнего кадра (или None)."""
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def run(self) -> None:
        """Основной цикл: MJPEG, при неудаче — опрос снимков."""
        while not self._stop_event.is_set():
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
            while not self._stop_event.is_set():
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
# Живая телеметрия с борта
# ---------------------------------------------------------------------------


class StateClient(threading.Thread):
    """Опрашивает ``/state.json`` ноды зрения и хранит последний живой снимок.

    Контракт терпимый: отсутствующие поля не считаются ошибкой, просто
    остаются значения синтетической модели. Возраст данных проверяется по
    ``time.monotonic``, поэтому «зависший» сервер не выдаётся за живой борт.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 8080,
                 enabled: bool = True, interval: float = 0.2) -> None:
        super().__init__(daemon=True)
        self.url = f"http://{host}:{port}/state.json"
        self.enabled = enabled
        self.interval = max(0.05, interval)
        self.status = "OFFLINE"
        self.state: dict | None = None
        self.stamp = 0.0
        self.lock = threading.Lock()
        self._stop_event = threading.Event()

    def stop(self) -> None:
        """Просит поток завершиться."""
        self._stop_event.set()

    def latest(self, max_age: float = 2.0):
        """Свежий снимок состояния или None, если данных нет или они устарели."""
        with self.lock:
            if self.state is None:
                return None
            if time.monotonic() - self.stamp > max_age:
                return None
            return self.state

    def run(self) -> None:
        """Основной цикл: запрос state.json каждые ~0,2 с."""
        while not self._stop_event.is_set():
            if not self.enabled:
                time.sleep(0.5)
                continue
            try:
                with urllib.request.urlopen(self.url, timeout=0.35) as response:
                    payload = json.loads(response.read().decode("utf-8", "replace"))
            except Exception:
                if self.status != "OFFLINE":
                    self.status = "OFFLINE"
                    if logger:
                        logger.warning(f"Живая телеметрия недоступна: {self.url}")
                time.sleep(self.interval)
                continue
            if isinstance(payload, dict):
                with self.lock:
                    self.state = payload
                    self.stamp = time.monotonic()
                if self.status != "ONLINE":
                    self.status = "ONLINE"
                    if logger:
                        logger.info(f"Живая телеметрия подключена: {self.url}")
            time.sleep(self.interval)


# ---------------------------------------------------------------------------
# Запись видео на диск
# ---------------------------------------------------------------------------


class RecordingWriter(threading.Thread):
    """Пишет кадры в MP4-файл в отдельном потоке, чтобы не тормозить пульт.

    UI-поток только складывает кадры в очередь ограниченной длины: если диск
    не успевает, кадр теряется, но интерфейс не встаёт.
    """

    def __init__(self, path: str | os.PathLike, fps: int = 12,
                 size: tuple[int, int] = (1280, 720), queue_size: int = 30) -> None:
        super().__init__(daemon=True)
        self.path = str(path)
        self.fps = max(1, int(fps))
        self.size = (int(size[0]), int(size[1]))
        self.queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=queue_size)
        self.written = 0
        self.error: str | None = None
        self.started_at = time.time()
        self._stop_event = threading.Event()
        self._finished = threading.Event()

    def offer(self, frame) -> bool:
        """Кладёт кадр в очередь без ожидания. False — очередь переполнена."""
        if self._stop_event.is_set() or frame is None:
            return False
        try:
            self.queue.put_nowait(frame.copy())
        except queue.Full:
            return False
        return True

    def stop(self) -> None:
        """Просит поток дописать файл и завершиться."""
        self._stop_event.set()

    def wait_finished(self, timeout: float = 3.0) -> bool:
        """Ждёт завершения потока записи (файл к этому моменту закрыт)."""
        return self._finished.wait(timeout)

    def run(self) -> None:
        """Основной цикл записи кадров в MP4."""
        writer = None
        try:
            width, height = self.size
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(self.path, fourcc, self.fps, (width, height))
            if not writer.isOpened():
                raise RuntimeError("cv2.VideoWriter не открыл файл")
            while True:
                try:
                    frame = self.queue.get(timeout=0.2)
                except queue.Empty:
                    if self._stop_event.is_set():
                        break
                    continue
                if frame is None:
                    continue
                if frame.shape[1] != width or frame.shape[0] != height:
                    frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)
                writer.write(frame)
                self.written += 1
                if self._stop_event.is_set() and self.queue.empty():
                    break
        except Exception as error:
            self.error = f"{type(error).__name__}: {error}"
            if logger:
                logger.error(f"Ошибка записи видео {self.path}: {self.error}")
        finally:
            if writer is not None:
                try:
                    writer.release()
                except Exception:
                    pass
            self._finished.set()


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
        self.started_at = time.time()
        self._log_queue: "queue.Queue[str]" = queue.Queue()
        self._video_report_queue: "queue.Queue[tuple]" = queue.Queue()
        self._video_report_busy = False
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

        # План полёта из config/mission.json и плановая сетка сканирования
        self.mission = load_mission()
        self.scan_rows = scan_rows(self.mission)
        self.scan_row: int | None = None
        self.scan_total = len(self.scan_rows)
        self.scan_progress: float | None = None

        # Живая телеметрия с борта (state.json)
        self.live_state: dict | None = None
        self.live_ts = 0.0
        self._last_detection_signature = None
        self._last_detection_log = 0.0
        self._mission_cmd_until = 0.0
        self._last_randomize_ts = 0.0

        # Отчёты оператора и перетаскивание очагов
        self.reports_dir = resolve_repo_path(self.mission.get("reports_dir"), REPORTS_DIR)
        self.photos_dir = self.reports_dir / "photos"
        self.video_dir = self.reports_dir / "video"
        self.incidents_dir = self.reports_dir / "incidents"
        self.mission_cmd = resolve_repo_path(self.mission.get("mission_cmd_file"), MISSION_CMD_PATH)
        self._ensure_report_dirs()
        self.recorder: RecordingWriter | None = None
        self.last_frame = None
        self._last_record_ts = 0.0
        # Кадры полёта для видеорапорта: снимок «раз в N секунд», не больше M
        # штук — чтобы ролик содержал патрулирование, а не один стоп-кадр.
        self._flight_frames: list = []
        self._flight_frames_max = 24
        self._flight_frame_interval = 1.5
        self._last_flight_frame_ts = 0.0
        # Фотофиксация инцидентов: кадр момента детекции + фото-улика борта.
        self._incident_shots: list = []
        self._incident_shot_keys: set = set()
        self._drag_key: str | None = None
        self._drag_moved = False

        # Данные с борта и с веб-сервера ноды зрения
        self.video = VideoClient(enabled=not offline)
        self.video.start()
        self.state_client = StateClient(enabled=not offline)
        self.state_client.start()
        self.watcher = ProtocolWatcher()
        # Старый протокол прошлого запуска — уже не новость: «примиряем» watcher.
        try:
            self.watcher.poll()
        except Exception:
            pass
        self.protocol: dict | None = None

        if logger:
            logger.info("Запуск АРМ Оператора ЦУКС 'Око МЧС' (тактический пульт v2.0)")
        if core_log_event:
            core_log_event("GCS_UI", "OPERATOR_ACTION", {"action": "APP_START", "status": "INITIALIZED"})

        self._build_ui()
        self._bind_keys()
        # self.protocol — словарь протокола инцидента, поэтому Tk-метод зовём явно.
        super().protocol("WM_DELETE_WINDOW", self._on_close)

        self.log("Пульт ЦУКС «Око МЧС» запущен. Ожидание данных с борта…")
        self.log(f"Отчёты: {self.reports_dir} (Windows: {windows_path(self.reports_dir)})")
        self.log(f"  · фото: {self.photos_dir}")
        self.log(f"  · видео: {self.video_dir}")
        self.log(f"  · инциденты: {self.incidents_dir}")
        self.log(f"  · журналы: {LOGS_DIR} · папка отчётов: Ctrl+O · видеорапорт: Ctrl+V")
        if self.scan_rows:
            self.log(f"План сканирования: рядов {len(self.scan_rows)}, файл {MISSION_PATH}.")
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

    def _ensure_report_dirs(self) -> None:
        """Создаёт каталоги отчётов и журналов, если их ещё нет.

        ``reports/incidents`` пишет ещё и нода зрения, поэтому каталог создаём
        на старте АРМ, не дожидаясь первого инцидента.
        """
        for directory in (self.reports_dir, self.photos_dir, self.video_dir,
                          self.incidents_dir, LOGS_DIR):
            try:
                directory.mkdir(parents=True, exist_ok=True)
            except OSError as error:
                if logger:
                    logger.warning(f"Не удалось создать каталог {directory}: {error}")

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

        # Статус сканирования: «РЯД k/N · NN%» из state.scan
        self.lbl_scan = ctk.CTkLabel(
            right,
            text="СКАН: —",
            font=ctk.CTkFont(FONT_UI, 12, "bold"),
            fg_color=COLORS["panel"],
            corner_radius=6,
            padx=12,
            pady=5,
            text_color=COLORS["info"],
        )
        self.lbl_scan.pack(side="right", padx=6)

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

        self.btn_random = ctk.CTkButton(
            head, text="🎲 СЛУЧАЙНО", width=118, height=26,
            font=ctk.CTkFont(FONT_UI, 11, "bold"),
            fg_color=COLORS["warn"], hover_color="#D97706", text_color="#0B0F14",
            command=self.randomize_hazards,
        )
        self.btn_random.pack(side="right", padx=(8, 0))

        self.map_canvas = tk.Canvas(card, bg=COLORS["panel_alt"], highlightthickness=0, bd=0)
        self.map_canvas.pack(fill="both", expand=True, padx=12, pady=(0, 12))

        # Перетаскивание очагов ЧС мышью прямо на карте
        self.map_canvas.bind("<Button-1>", self._on_map_press)
        self.map_canvas.bind("<B1-Motion>", self._on_map_drag)
        self.map_canvas.bind("<ButtonRelease-1>", self._on_map_release)

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

        # Компактные кнопки отчётов: снимок, запись, папка отчётов
        actions = ctk.CTkFrame(card, fg_color="transparent")
        actions.pack(fill="x", padx=12, pady=(0, 4))

        self.btn_photo = ctk.CTkButton(
            actions, text="📸 СНИМОК", height=26,
            font=ctk.CTkFont(FONT_UI, 10, "bold"),
            fg_color=COLORS["panel_alt"], hover_color=COLORS["border"], text_color=COLORS["text"],
            command=self.save_snapshot,
        )
        self.btn_photo.pack(side="left", expand=True, fill="x", padx=(0, 3))

        self.btn_record = ctk.CTkButton(
            actions, text="● ЗАПИСЬ", height=26,
            font=ctk.CTkFont(FONT_UI, 10, "bold"),
            fg_color=COLORS["panel_alt"], hover_color=COLORS["border"], text_color=COLORS["crit"],
            command=self.toggle_recording,
        )
        self.btn_record.pack(side="left", expand=True, fill="x", padx=2)

        self.btn_reports = ctk.CTkButton(
            actions, text="📂 ОТЧЁТЫ", height=26,
            font=ctk.CTkFont(FONT_UI, 10, "bold"),
            fg_color=COLORS["panel_alt"], hover_color=COLORS["border"], text_color=COLORS["info"],
            command=self.open_reports_folder,
        )
        self.btn_reports.pack(side="left", expand=True, fill="x", padx=(2, 0))

        foot = ctk.CTkFrame(card, fg_color="transparent")
        foot.pack(fill="x", padx=12, pady=(0, 10))

        self.lbl_mode = ctk.CTkLabel(
            foot, text="РЕЖИМ: ОПТИЧЕСКИЙ 4K", font=ctk.CTkFont(FONT_MONO, 11), text_color=COLORS["muted"]
        )
        self.lbl_mode.pack(side="left")

        self.lbl_rec = ctk.CTkLabel(
            foot, text="", font=ctk.CTkFont(FONT_MONO, 11, "bold"), text_color=COLORS["crit"]
        )
        self.lbl_rec.pack(side="left", padx=10)

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
            anchor="w", justify="left", wraplength=320,
        )
        self.lbl_incident_response.pack(fill="x", padx=14, pady=(0, 6))
        # Ширина карточки меняется вместе с окном: перенос строк обязан
        # следовать за ней, иначе рекомендация по наряду обрезается по краю.
        card.bind("<Configure>", lambda event: self.lbl_incident_response.configure(
            wraplength=max(180, event.width - 44)))

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
        self.btn_diag.pack(side="left", expand=True, fill="x", padx=2)

        video_row = ctk.CTkFrame(buttons, fg_color="transparent")
        video_row.pack(fill="x", pady=(4, 0))
        self.btn_video_rep = ctk.CTkButton(
            video_row, text="🎬 ВИДЕОРАПОРТ", height=30,
            font=ctk.CTkFont(FONT_UI, 11, "bold"),
            fg_color="#0D47A1", hover_color="#1565C0",
            command=self.compile_mission_video_report,
        )
        self.btn_video_rep.pack(fill="x")

        ctk.CTkLabel(
            card,
            text="Space — старт/RTL · T — тепловизор · O — оверлей · E — экспорт\n"
                 "P — снимок · R — запись · G — случайные очаги · Ctrl+O — отчёты",
            wraplength=560,
            font=ctk.CTkFont(FONT_UI, 9),
            text_color=COLORS["muted"],
            justify="center",
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
        self.bind("<Key-p>", lambda _event: self.save_snapshot())
        self.bind("<Key-r>", lambda _event: self.toggle_recording())
        self.bind("<Key-g>", lambda _event: self.randomize_hazards())
        self.bind("<Control-v>", lambda _event: self.compile_mission_video_report())
        self.bind("<Control-o>", lambda _event: self.open_reports_folder())
        self.bind("<Key-d>", lambda _event: self.build_diagnostic_bundle())
        self.bind("<Control-d>", lambda _event: self.build_diagnostic_bundle())
        self.bind("<F11>", lambda _event: self.attributes("-fullscreen", True))
        self.bind("<Escape>", lambda _event: self.attributes("-fullscreen", False))

    def log(self, text: str, level: str = "INFO") -> None:
        """Добавляет строку в журнал дежурного и в подсистему логирования mchs_core."""
        line = f"[{now_str()}] {text}\n"
        try:
            if threading.current_thread() is threading.main_thread():
                self._append_log(line)
            else:
                # Из фоновых потоков нельзя трогать Tk: отдаём строку главному циклу.
                self._log_queue.put(line)
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

    def _append_log(self, line: str) -> None:
        """Дописывает готовую строку в текстовое поле журнала (только главный поток)."""
        if self.log_box is None:
            return
        try:
            self.log_box.configure(state="normal")
            self.log_box.insert("end", line)
            self.log_box.see("end")
            self.log_box.configure(state="disabled")
        except Exception:
            pass

    def _drain_log_queue(self) -> None:
        """Переносит в журнал строки, накопленные фоновыми потоками."""
        while True:
            try:
                line = self._log_queue.get_nowait()
            except queue.Empty:
                return
            self._append_log(line)

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
        """Старт патрулирования и аварийный возврат (RTL).

        В живом режиме кнопка не двигает синтетическую модель, а пишет
        файл-команду ``reports/mission.cmd`` для автопилота: ``scan`` или
        ``rtl`` (автопилот прочитает файл и удалит его сам).
        """
        live = self._live()
        if not self.mission_active:
            self.mission_active = True
            self.detected = False
            self._flight_frames.clear()
            self._incident_shots.clear()
            self._incident_shot_keys.clear()
            self._mission_btn_mode = "scan"
            self.btn_mission.configure(text="■ АВАРИЙНЫЙ ВОЗВРАТ (RTL)", fg_color=COLORS["crit"])
            if live:
                self._mission_cmd_until = time.time() + 3.0
                self.phase = "СКАНИРОВАНИЕ"
                self._write_mission_command("scan")
                self.log("Команда автопилоту: СТАРТ сканирования (mission.cmd → scan).")
            else:
                self.altitude = 0.0
                self.phase = "ВЗЛЁТ И ПАТРУЛИРОВАНИЕ"
                self.log("Команда: взлёт на эшелон 8,0 м, начало патрулирования сектора.")
        else:
            self.mission_active = False
            self._mission_btn_mode = "rtl"
            self.btn_mission.configure(text="▶ СТАРТ ПАТРУЛИРОВАНИЯ", fg_color=COLORS["accent"])
            self.phase = "ВОЗВРАТ НА БАЗУ (RTL)"
            if live:
                self._mission_cmd_until = time.time() + 3.0
                self._write_mission_command("rtl")
                self.log("Команда автопилоту: АВАРИЙНЫЙ ВОЗВРАТ (mission.cmd → rtl).")
            else:
                self.log("Команда: возврат на базу (RTL), посадка в точке старта.")
        if core_log_event:
            core_log_event("GCS_UI", "OPERATOR_ACTION", {
                "action": "START_PATROL" if self.mission_active else "ABORT_RTL",
                "phase": self.phase,
                "live": live,
                "altitude_m": round(self.altitude, 1),
                "battery_pct": int(self.battery),
            })

    def _write_mission_command(self, command: str) -> None:
        """Пишет файл-команду миссии для автопилота (``scan`` или ``rtl``)."""
        try:
            self.mission_cmd.parent.mkdir(parents=True, exist_ok=True)
            self.mission_cmd.write_text(command + "\n", encoding="utf-8")
            self.log(f"Файл-команда миссии: {self.mission_cmd} → {command}")
        except OSError as error:
            self.log(f"Не удалось записать файл-команду миссии: {error}", level="ERROR")

    # -- живая телеметрия --------------------------------------------------

    def _live(self) -> bool:
        """Живой режим: телеметрия с борта есть и ей не больше 2 секунд."""
        return self.live_state is not None and (time.monotonic() - self.live_ts) <= 2.0

    def _poll_live_state(self) -> bool:
        """Забирает свежее ``/state.json`` и обновляет бортовые показания."""
        state = self.state_client.latest()
        if state is None:
            return False

        self.live_state = state
        self.live_ts = time.monotonic()

        drone = state.get("drone") or {}
        try:
            if drone.get("x") is not None:
                self.drone_x = float(drone["x"])
            if drone.get("y") is not None:
                self.drone_y = float(drone["y"])
            if drone.get("z") is not None:
                self.altitude = float(drone["z"])
            if drone.get("speed_mps") is not None:
                self.speed = float(drone["speed_mps"])
            if drone.get("yaw_deg") is not None:
                self.heading_deg = float(drone["yaw_deg"]) % 360.0
            if drone.get("battery_percent") is not None:
                self.battery = float(drone["battery_percent"])
        except (TypeError, ValueError):
            pass

        # Трек накапливается из реальных позиций, точки ближе 0,4 м не плодим.
        if self.altitude > 0.5:
            if not self.trail or math.hypot(self.drone_x - self.trail[-1][0],
                                            self.drone_y - self.trail[-1][1]) > 0.4:
                self.trail.append((self.drone_x, self.drone_y))
                if len(self.trail) > 2000:
                    del self.trail[: len(self.trail) - 2000]

        scan = state.get("scan") or {}
        phase = str(scan.get("phase") or "").strip()
        if phase:
            self.phase = phase
        self.scan_progress = _as_float(scan.get("progress"), None)

        total = self.scan_total or len(self.scan_rows)
        try:
            total = int(scan.get("rows_total") or total)
        except (TypeError, ValueError):
            pass
        self.scan_total = total
        try:
            row = int(scan.get("row"))
        except (TypeError, ValueError):
            row = None
        if row is not None and row != self.scan_row:
            self.scan_row = row
            progress = self.scan_progress
            percent = f" · {progress * 100:.0f}%" if isinstance(progress, (int, float)) else ""
            self.log(f"СКАНИРОВАНИЕ: РЯД {row}/{total}{percent}")

        self._sync_mission_button(phase)
        self._log_detection(state.get("detection") or {})
        self._record_incident_shot(state.get("detection") or {},
                                   state.get("reports") or {})
        return True

    def _sync_mission_button(self, scan_phase: str) -> None:
        """Синхронизирует подпись кнопки миссии с фазой, которую ведёт автопилот."""
        if time.time() < self._mission_cmd_until:
            return  # только что отправлена своя команда — не перебиваем её
        phase = (scan_phase or "").upper()
        if "СКАНИР" in phase:
            self.mission_active = True
            if getattr(self, "_mission_btn_mode", "start") != "scan":
                self._mission_btn_mode = "scan"
                self.btn_mission.configure(text="■ АВАРИЙНЫЙ ВОЗВРАТ (RTL)", fg_color=COLORS["crit"])
        elif "ВОЗВРАТ" in phase:
            self.mission_active = False
            if getattr(self, "_mission_btn_mode", "start") != "rtl":
                self._mission_btn_mode = "rtl"
                self.btn_mission.configure(text="▶ ПРОДОЛЖИТЬ СКАНИРОВАНИЕ", fg_color=COLORS["accent"])

    def _log_detection(self, detection: dict) -> None:
        """Пишет в журнал реальные события детекции борта (не чаще раза в 8 с)."""
        event = detection.get("last_event")
        if not isinstance(event, dict):
            return
        signature = (
            event.get("ts"), event.get("hazard_type"),
            event.get("x"), event.get("y"), event.get("area_sq_m"),
        )
        if signature == self._last_detection_signature:
            return
        self._last_detection_signature = signature
        now = time.time()
        if now - self._last_detection_log < 8.0:
            return
        self._last_detection_log = now
        area = _as_float(event.get("area_sq_m"), 0.0) or 0.0
        x = _as_float(event.get("x"), 0.0) or 0.0
        y = _as_float(event.get("y"), 0.0) or 0.0
        area_text = f"{area:.0f}" if area >= 10.0 else f"{area:.1f}"
        self.log(
            f"БОРТ: {event.get('hazard_type') or 'ЧС'} {event.get('threat_level') or ''}, "
            f"{area_text} м², X {x:.1f} / Y {y:.1f} м".strip()
        )

    def _record_incident_shot(self, detection: dict, reports: dict) -> None:
        """Запоминает кадр и фото-улику момента детекции для видеорапорта."""
        event = detection.get("last_event")
        if not isinstance(event, dict) or not event:
            return
        x = _as_float(event.get("x"), None)
        y = _as_float(event.get("y"), None)
        # Ключ — инцидент и точка, без текстового типа: одно и то же событие
        # приходит и из state.json ("VICTIM"), и из протокола ("ПОСТРАДАВШИЙ").
        key = (
            event.get("incident_id")
            or str(event.get("hazard_type") or "").upper(),
            round(x) if x is not None else None,
            round(y) if y is not None else None,
        )
        if key in self._incident_shot_keys:
            return
        frame = self._capture_frame()
        if frame is None:
            return
        photo = str(event.get("photo") or reports.get("last_photo") or "") or None
        self._incident_shot_keys.add(key)
        self._incident_shots.append({
            "frame": frame.copy(),
            "photo": photo,
            "label": str(event.get("hazard_type") or "ЧС"),
            "threat_level": str(event.get("threat_level") or ""),
            "area": _as_float(event.get("area_sq_m"), None),
            "x": x,
            "y": y,
            "time": str(event.get("ts") or ""),
        })
        if len(self._incident_shots) > 8:
            self._incident_shots.pop(0)
        self.log(
            "🎥 Кадр инцидента сохранён для видеорапорта: "
            f"{event.get('hazard_type')} {event.get('threat_level')} "
            f"X={x if x is not None else '—'} Y={y if y is not None else '—'}"
        )

    # -- отчёты: снимки, запись, папка -------------------------------------

    def _capture_frame(self):
        """Кадр «как видит оператор»: видео/синтетика + тепловизор + HUD."""
        frame = self.video.snapshot()
        live_frame = frame is not None
        if frame is None:
            width = max(640, self.cam_canvas.winfo_width())
            height = max(360, self.cam_canvas.winfo_height())
            frame = self._render_synthetic_scene(width, height)

        if self.thermal:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
            frame = cv2.applyColorMap(clahe.apply(gray), cv2.COLORMAP_INFERNO)
        if self.overlay:
            frame = self._draw_hud(frame, frame.shape[1], frame.shape[0],
                                   include_telemetry=not live_frame)
        return frame

    @staticmethod
    def _write_jpeg(frame, path: Path) -> Path:
        """Сохраняет кадр в JPEG через imencode — надёжно на UNC-путях."""
        ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        if not ok:
            raise RuntimeError("cv2.imencode вернул False")
        path.write_bytes(buffer.tobytes())
        return path

    def save_snapshot(self) -> None:
        """📸 Сохраняет текущий кадр оператора в ``reports/photos`` (клавиша P)."""
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self.photos_dir / f"snapshot_{stamp}.jpg"
        try:
            frame = self._capture_frame()
            if frame is None:
                raise RuntimeError("кадр недоступен")
            self.photos_dir.mkdir(parents=True, exist_ok=True)
            self._write_jpeg(frame, path)
            size_kb = path.stat().st_size / 1024.0
            self.log(f"📸 Снимок сохранён: {path} ({size_kb:.0f} КБ)")
            if core_log_event:
                core_log_event("GCS_UI", "OPERATOR_ACTION", {
                    "action": "SAVE_SNAPSHOT", "status": "SUCCESS", "path": str(path),
                })
        except Exception as error:
            self.log(f"Не удалось сохранить снимок: {error}", level="ERROR")

    def toggle_recording(self) -> None:
        """● Старт/стоп видеозаписи бортовой камеры в ``reports/video`` (R)."""
        if self.recorder is not None:
            self._stop_recording()
            return

        if self.last_frame is None and self.video.snapshot() is None:
            self.log("Нет видеопотока для записи: дождитесь первого кадра.", level="WARNING")
            return

        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self.video_dir / f"record_{stamp}.mp4"
        try:
            self.video_dir.mkdir(parents=True, exist_ok=True)
            writer = RecordingWriter(path, fps=12, size=(1280, 720))
            writer.start()
        except Exception as error:
            self.log(f"Не удалось начать запись видео: {error}", level="ERROR")
            return

        self.recorder = writer
        self._last_record_ts = 0.0
        self.btn_record.configure(text="■ СТОП ЗАПИСЬ", fg_color=COLORS["crit"], text_color="#FFFFFF")
        self.lbl_rec.configure(text="● REC")
        self.log(f"⏺ Начата запись видео: {path} (12 к/с, 1280×720)")
        if core_log_event:
            core_log_event("GCS_UI", "OPERATOR_ACTION", {
                "action": "START_RECORDING", "path": str(path),
            })

    def _pump_recording(self) -> None:
        """Кладёт в писатель кадры оператора ровно с частотой 12 к/с."""
        writer = self.recorder
        if writer is None:
            return
        if not writer.is_alive():
            self.log(f"Запись прервана: {writer.error or 'поток записи завершился'}", level="ERROR")
            self._stop_recording(reason="ошибка записи")
            return
        now = time.monotonic()
        if now - self._last_record_ts < 1.0 / 12.0:
            return
        self._last_record_ts = now
        writer.offer(self._capture_frame())

    def _stop_recording(self, reason: str = "оператор") -> None:
        """Останавливает запись и запускает фоновое дописывание MP4."""
        writer = self.recorder
        if writer is None:
            return
        self.recorder = None
        writer.stop()
        threading.Thread(target=self._finalize_recording, args=(writer,), daemon=True).start()
        self.btn_record.configure(text="● ЗАПИСЬ", fg_color=COLORS["panel_alt"], text_color=COLORS["crit"])
        self.lbl_rec.configure(text="")
        self.log(f"⏹ Запись видео остановлена ({reason}), файл дописывается…")

    def _finalize_recording(self, writer: "RecordingWriter") -> None:
        """Дожидается закрытия MP4 и пишет в журнал путь, размер и длительность."""
        writer.wait_finished(5.0)
        if writer.error:
            self.log(f"Ошибка записи видео: {writer.error}", level="ERROR")
            return
        try:
            size_kb = os.path.getsize(writer.path) / 1024.0
        except OSError:
            size_kb = 0.0
        duration = max(0.0, time.time() - writer.started_at)
        self.log(
            f"✅ Видео сохранено: {writer.path} "
            f"({size_kb:.0f} КБ, {duration:.1f} с, {writer.written} кадров)"
        )
        if core_log_event:
            core_log_event("GCS_UI", "OPERATOR_ACTION", {
                "action": "STOP_RECORDING", "status": "SUCCESS",
                "path": writer.path, "frames": writer.written,
                "size_kb": round(size_kb, 1), "duration_s": round(duration, 1),
            })

    def _save_incident_photo(self, protocol: dict) -> str | None:
        """Авто-снимок момента тревоги в ``reports/incidents`` (как видит оператор)."""
        try:
            frame = self._capture_frame()
            if frame is None:
                return None
            stamp = datetime.datetime.now().strftime("%H%M%S")
            incident_id = str(protocol.get("incident_id") or "INC").replace("/", "_").replace("\\", "_")
            path = self.incidents_dir / f"{incident_id}_{stamp}.jpg"
            self.incidents_dir.mkdir(parents=True, exist_ok=True)
            self._write_jpeg(frame, path)
            self.log(f"📷 Авто-снимок тревоги: {path}")
            return str(path)
        except Exception as error:
            self.log(f"Не удалось сохранить авто-снимок тревоги: {error}", level="WARNING")
            return None

    def open_reports_folder(self) -> None:
        """📂 Открывает папку отчётов в Проводнике Windows (Ctrl+O)."""
        try:
            self.reports_dir.mkdir(parents=True, exist_ok=True)
        except OSError:
            pass
        target = windows_path(self.reports_dir)
        if hasattr(os, "startfile"):
            try:
                os.startfile(target)  # type: ignore[attr-defined]
                self.log(f"📂 Открыта папка отчётов: {target}")
                return
            except OSError as error:
                self.log(f"Не удалось открыть Проводник: {error}", level="WARNING")
        self.log(f"📂 Папка отчётов: {target}")

    def compile_mission_video_report(self) -> None:
        """🎬 Собирает тактический видеорапорт в фоне, не блокируя пульт."""
        if self._video_report_busy:
            self.log("🎬 Видеорапорт уже собирается — дождитесь завершения.")
            return
        if MissionVideoReporter is None:
            self.log("Видеорапорт недоступен: не установлен mchs_core.video_reporter",
                     level="ERROR")
            return

        frame = self._capture_frame()
        if frame is None:
            self.log("Нечего записывать: нет ни кадра с борта, ни резервного кадра.",
                     level="WARNING")
            return

        snapshot = frame.copy()
        # Ролик собираем из накопленных кадров патруля; если их ещё нет —
        # хотя бы текущий кадр, чтобы кнопка работала сразу после старта.
        frames = [item.copy() for item in self._flight_frames][-12:]
        if not frames:
            frames = [snapshot]
        # Фотофиксация: приоритет у снимка борта (там рамка и HUD детекции).
        shots = []
        for item in list(self._incident_shots)[-8:]:
            shot_frame = item.get("frame")
            photo = item.get("photo")
            if photo:
                loaded = read_image_any(photo)
                if loaded is not None:
                    shot_frame = loaded
            if shot_frame is None:
                continue
            shots.append({**item, "frame": shot_frame})
        protocol = dict(self.protocol or {})
        self._video_report_busy = True
        self.btn_video_rep.configure(state="disabled", text="🎬 СБОРКА…")
        self.log(
            f"🎬 Формирую тактический видеорапорт: кадров полёта {len(frames)}, "
            f"кадров инцидентов {len(shots)}…"
        )

        def worker() -> None:
            try:
                width, height = snapshot.shape[1], snapshot.shape[0]
                reporter = MissionVideoReporter(VideoReportConfig(
                    width=width, height=height, fps=10.0,
                ))
                incidents = []
                if protocol:
                    geo = protocol.get("target_geo") or {}
                    coords = ""
                    if geo.get("abs_x") is not None:
                        coords = f"X={geo['abs_x']:.1f}, Y={geo['abs_y']:.1f}"
                    incidents.append({
                        "type": protocol.get("hazard_type", "ЧС"),
                        "label": protocol.get("hazard_type", "Инцидент"),
                        "coords": coords,
                        "area": f"{protocol.get('area_sq_m', 0):.1f} м²",
                    })
                progress = 0.0 if self.scan_progress is None else self.scan_progress
                stats = {
                    "время_полёта": f"{max(0.0, time.time() - self.started_at):.0f} с",
                    "прогресс_сканирования": f"{progress * 100:.0f}%",
                    "кадров_обработано": self.frame_no,
                    "инцидентов_зафиксировано": len(shots) or len(incidents),
                    "статус": self.phase,
                }
                stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                self.video_dir.mkdir(parents=True, exist_ok=True)
                path = self.video_dir / f"mission_report_{stamp}.mp4"
                result = reporter.compile_report(
                    frames=frames,
                    incidents=[] if shots else incidents,
                    stats=stats,
                    output_path=str(path),
                    shots=shots,
                )
                self._video_report_queue.put(("ok", result))
            except Exception as error:  # noqa: BLE001 - показываем оператору
                self._video_report_queue.put(("error", f"{type(error).__name__}: {error}"))

        threading.Thread(target=worker, daemon=True,
                         name="mission-video-report").start()

    def _poll_video_report(self) -> None:
        """Забирает результат фоновой сборки видеорапорта в главном потоке."""
        while True:
            try:
                status, payload = self._video_report_queue.get_nowait()
            except queue.Empty:
                return
            self._video_report_busy = False
            self.btn_video_rep.configure(state="normal", text="🎬 ВИДЕОРАПОРТ")
            if status == "ok":
                self.log(f"🎬 Тактический видеорапорт сформирован: {payload}")
                if core_log_event:
                    core_log_event("GCS_UI", "OPERATOR_ACTION", {
                        "action": "VIDEO_REPORT", "status": "SUCCESS",
                        "path": str(payload),
                    })
                try:
                    # MCHS_NO_VIDEO_AUTOPLAY=1 — не открывать плеер автоматически
                    # (большой экран, запись демонстрации, автотесты).
                    if os.environ.get("MCHS_NO_VIDEO_AUTOPLAY") == "1":
                        return
                    if sys.platform == "win32" and hasattr(os, "startfile"):
                        os.startfile(str(payload))  # type: ignore[attr-defined]
                    else:
                        subprocess.Popen(["xdg-open", str(payload)],
                                         stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL)
                except Exception:
                    pass
            else:
                self.log(f"Не удалось собрать видеорапорт: {payload}", level="ERROR")

    def _on_close(self) -> None:
        """Корректное закрытие пульта: дописать видео и остановить потоки."""
        try:
            if self.recorder is not None:
                writer = self.recorder
                self.recorder = None
                writer.stop()
                writer.wait_finished(2.5)
                if writer.error:
                    if logger:
                        logger.error(f"Ошибка записи видео при выходе: {writer.error}")
        except Exception:
            pass
        try:
            self.video.stop()
        except Exception:
            pass
        try:
            self.state_client.stop()
        except Exception:
            pass
        self.destroy()

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
        self._save_incident_photo(protocol)
        self._record_incident_shot({
            "last_event": {
                "incident_id": protocol.get("incident_id"),
                "hazard_type": protocol.get("hazard_type"),
                "threat_level": protocol.get("threat_level"),
                "area_sq_m": protocol.get("area_sq_m"),
                "x": protocol.get("x"),
                "y": protocol.get("y"),
                "ts": time.strftime("%H:%M:%S"),
            },
        }, {"last_photo": protocol.get("photo") or ""})
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

            live = self._poll_live_state()
            self._drain_log_queue()
            self._poll_video_report()

            protocol = self.watcher.poll()
            if protocol is not None:
                self.apply_protocol(protocol)

            self._step_flight_model(live)
            self._refresh_labels()
            self._draw_map()
            self._draw_camera()
            self._pump_recording()
        except Exception as error:  # пульт не должен падать из-за одного кадра
            if self.frame_no % 100 == 0:
                self.log(f"Сбой отрисовки: {type(error).__name__}: {error}")

        self.after(40, self.update_loop)

    def _step_flight_model(self, live: bool = False) -> None:
        """Простая модель полёта для демонстрации, пока нет телеметрии с борта."""
        if live:
            # Живую телеметрию ведёт борт: модель не смеет двигать дрон и трек.
            if self.mission_active:
                self.battery = max(8.0, self.battery - 0.004)
            return

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
        В живом режиме синтетика запрещена: протоколы пишет нода зрения.
        """
        if self._live():
            return
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

        live = self._live()
        link = self.video.status
        link_color = COLORS["ok"] if link in ("MJPEG", "SNAPSHOT") else COLORS["warn"]
        telemetry = "ЖИВАЯ" if live else "МОДЕЛЬ"
        self.lbl_link.configure(text=f"● СВЯЗЬ: {link} · ТМ: {telemetry}", text_color=link_color)
        self.lbl_phase.configure(text=self.phase.split(":")[0])

        if self.scan_row is not None:
            progress = self.scan_progress
            percent = f" · {progress * 100:.0f}%" if isinstance(progress, (int, float)) else ""
            total = self.scan_total or len(self.scan_rows)
            self.lbl_scan.configure(
                text=f"СКАН: РЯД {self.scan_row}/{total}{percent}", text_color=COLORS["info"]
            )
        else:
            self.lbl_scan.configure(text="СКАН: —", text_color=COLORS["muted"])

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

        if self.recorder is not None:
            self.lbl_rec.configure(text="● REC" if int(self.frame_no / 8) % 2 == 0 else "")

        fps = f"{self.video.fps:.0f}" if self.video.fps else "—"
        self.lbl_fps.configure(text=f"FPS: {fps}  ·  {self.video.status}")

    # -- очаги: перетаскивание на карте -----------------------------------

    def _canvas_to_world(self, px: float, py: float) -> tuple[float, float]:
        """Обратное к :meth:`_world_to_canvas`: пиксели карты → метры полигона.

        Рамка карты та же, что в :meth:`_world_to_canvas`: отступы
        34/26/18/30 пкс, ось Y направлена вверх.
        """
        width = max(1, self.map_canvas.winfo_width())
        height = max(1, self.map_canvas.winfo_height())
        left, top, right, bottom = 34, 26, width - 18, height - 30
        span_x = max(1.0, float(right - left))
        span_y = max(1.0, float(bottom - top))
        x_m = (px - left) / span_x * WORLD_WIDTH_M - WORLD_HALF_M
        y_m = WORLD_HALF_M - (py - top) / span_y * WORLD_HEIGHT_M
        return x_m, y_m

    def _hazard_at_pixel(self, px: float, py: float, radius: float = 16.0) -> str | None:
        """Ключ ближайшего очага в радиусе клика (или None)."""
        best_key = None
        best_distance = float(radius)
        for key, hazard in HAZARDS_DB.items():
            x = _as_float(hazard.get("x"), None)
            y = _as_float(hazard.get("y"), None)
            if x is None or y is None:
                continue
            hazard_px, hazard_py = self._world_to_canvas(x, y)
            distance = math.hypot(px - hazard_px, py - hazard_py)
            if distance <= best_distance:
                best_key, best_distance = key, distance
        return best_key

    def _on_map_press(self, event) -> None:
        """Захват очага левой кнопкой мыши для перетаскивания."""
        key = self._hazard_at_pixel(event.x, event.y)
        if key is None:
            return
        self._drag_key = key
        self._drag_moved = False
        self.hazard_key = key
        try:
            self.selector.set(HAZARDS_DB[key]["short"])
        except Exception:
            pass
        self.map_canvas.configure(cursor="fleur")
        self.log(f"Очаг {HAZARDS_DB[key]['short']} выбран: перетащите его по карте и отпустите.")

    def _on_map_drag(self, event) -> None:
        """Двигает захваченный очаг; координаты пересчитываются в метры."""
        if self._drag_key is None or self._drag_key not in HAZARDS_DB:
            return
        x_m, y_m = self._canvas_to_world(event.x, event.y)
        x_m = max(-WORLD_HALF_M, min(WORLD_HALF_M, x_m))
        y_m = max(-WORLD_HALF_M, min(WORLD_HALF_M, y_m))
        hazard = HAZARDS_DB[self._drag_key]
        old_x = _as_float(hazard.get("x"), 0.0) or 0.0
        old_y = _as_float(hazard.get("y"), 0.0) or 0.0
        hazard["x"] = round(x_m, 1)
        hazard["y"] = round(y_m, 1)
        if abs(hazard["x"] - old_x) >= 0.1 or abs(hazard["y"] - old_y) >= 0.1:
            self._drag_moved = True

    def _on_map_release(self, event) -> None:
        """Фиксирует очаг: сохраняет config/hazards.json и применяет его в Gazebo."""
        key = self._drag_key
        self._drag_key = None
        try:
            self.map_canvas.configure(cursor="")
        except Exception:
            pass
        if key is None or key not in HAZARDS_DB:
            return
        if not self._drag_moved:
            return
        self._drag_moved = False

        hazard = HAZARDS_DB[key]
        x = _as_float(hazard.get("x"), 0.0) or 0.0
        y = _as_float(hazard.get("y"), 0.0) or 0.0
        self.log(f"Очаг {hazard['short']} перемещён в X {x:.1f} / Y {y:.1f} м.")
        self._save_hazards()
        if core_log_event:
            core_log_event("GCS_UI", "OPERATOR_ACTION", {
                "action": "MOVE_HAZARD",
                "hazard_key": key,
                "x": round(x, 1),
                "y": round(y, 1),
            })
        self._apply_hazard_in_gazebo(key, x, y)

    def _save_hazards(self) -> None:
        """Сохраняет текущие координаты очагов в ``config/hazards.json``."""
        try:
            path = save_hazards(HAZARDS_DB)
            self.log(f"Очаги сохранены: {path}")
            if core_log_event:
                core_log_event("GCS_UI", "OPERATOR_ACTION", {
                    "action": "SAVE_HAZARDS", "status": "SUCCESS", "path": str(path),
                })
        except OSError as error:
            self.log(f"Не удалось сохранить очаги: {error}", level="ERROR")

    def _apply_hazard_in_gazebo(self, key: str, x: float, y: float) -> None:
        """Применяет координаты очага в Gazebo через WSL (в фоне, не блокируя UI)."""
        if sys.platform != "win32":
            self.log("Применение очага в Gazebo пропущено: wsl.exe доступен только на Windows.")
            return
        distro = os.environ.get("WSL_DISTRO_NAME") or "Ubuntu-22.04"
        repo = repo_wsl_path()
        command = [
            "wsl.exe", "-d", distro, "-e", "bash", "-lc",
            f"cd '{repo}' && python3 src/mchs_simulation/scripts/arena_controller.py"
            f" --hazard {key.lower()} --x {x:.1f} --y {y:.1f}",
        ]
        threading.Thread(target=self._run_gazebo_command, args=(key, command), daemon=True).start()

    # -- случайное перемещение очагов (доказательство детекции) -----------

    def randomize_hazards(self) -> None:
        """🎲 Переставляет все очаги в зону обзора дрона и применяет в Gazebo.

        Кнопка «🎲 СЛУЧАЙНО» (или клавиша ``G``): координаты генерируются
        рядом с текущим дроном, сохраняются в ``config/hazards.json`` и одним
        вызовом ``arena_controller.py --positions`` переносят модели в Gazebo.
        Нода зрения перечитывает конфиг очагов каждые 2 с, поэтому новая
        тревога в АРМ появляется почти сразу.
        """
        now = time.time()
        if now - getattr(self, "_last_randomize_ts", 0.0) < 5.0:
            self.log("🎲 Подождите: предыдущее перемещение ещё применяется в Gazebo.")
            return
        self._last_randomize_ts = now
        positions = plan_random_hazards(self.drone_x, self.drone_y, list(HAZARDS_DB))
        for key, (x, y) in positions.items():
            hazard = HAZARDS_DB.get(key)
            if hazard is None:
                continue
            hazard["x"] = x
            hazard["y"] = y
        self._save_hazards()
        summary = " · ".join(
            f"{HAZARDS_DB[key]['short']} X {x:.0f} / Y {y:.0f}"
            for key, (x, y) in positions.items() if key in HAZARDS_DB
        )
        self.log(f"🎲 СЛУЧАЙНОЕ ПЕРЕМЕЩЕНИЕ ОЧАГОВ: {summary}")
        self.log("    Очаги в полосе обзора камеры — детекция сработает в ближайшие секунды.")
        self._last_detection_signature = None
        if core_log_event:
            core_log_event("GCS_UI", "OPERATOR_ACTION", {
                "action": "RANDOMIZE_HAZARDS",
                "positions": {key: [x, y] for key, (x, y) in positions.items()},
                "drone_x": round(self.drone_x, 1),
                "drone_y": round(self.drone_y, 1),
            })
        self._apply_hazards_in_gazebo(positions)

    def _apply_hazards_in_gazebo(self, positions: dict) -> None:
        """Двигает все очаги в Gazebo одним процессом arena_controller.py."""
        if sys.platform != "win32":
            self.log("Перемещение в Gazebo пропущено: wsl.exe доступен только на Windows.")
            return
        payload = {key.lower(): [float(x), float(y)]
                   for key, (x, y) in positions.items()}
        json_payload = json.dumps(payload, ensure_ascii=False)
        distro = os.environ.get("WSL_DISTRO_NAME") or "Ubuntu-22.04"
        repo = repo_wsl_path()
        command = [
            "wsl.exe", "-d", distro, "-e", "bash", "-lc",
            f"cd '{repo}' && python3 src/mchs_simulation/scripts/arena_controller.py"
            f" --positions '{json_payload}'",
        ]
        threading.Thread(target=self._run_gazebo_command,
                         args=("СЛУЧАЙНЫЕ ОЧАГИ", command), daemon=True).start()

    def _run_gazebo_command(self, key: str, command: list[str]) -> None:
        """Выполняет wsl.exe-команду и пишет её результат в журнал дежурного."""
        try:
            result = subprocess.run(command, timeout=25, capture_output=True, text=True)
        except Exception as error:
            self.log(f"Не удалось применить очаг {key} в Gazebo: {error}", level="ERROR")
            return
        output = (result.stdout or "").strip()
        tail = output.splitlines()[-1] if output else ""
        if result.returncode == 0:
            self.log(f"Очаг {key} применён в Gazebo. {tail}".strip())
        else:
            detail = (result.stderr or output or "без вывода").strip().splitlines()
            self.log(
                f"Очаг {key} не применён (код {result.returncode}): "
                f"{detail[-1][:180] if detail else ''}",
                level="WARNING",
            )

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

        # Плановые ряды сканирования (змейка) из config/mission.json
        for index, row in enumerate(self.scan_rows, 1):
            active = self.scan_row is not None and index == self.scan_row
            color = COLORS["info"] if active else "#1B3550"
            points = []
            for x_m, y_m in row:
                points.extend(self._world_to_canvas(x_m, y_m))
            if len(points) < 4:
                continue
            options = {"fill": color, "width": 2 if active else 1}
            if not active:
                options["dash"] = (3, 5)
            canvas.create_line(*points, **options)
            label_x, label_y = self._world_to_canvas(row[0][0], row[0][1])
            anchor = "w" if row[0][0] < row[-1][0] else "e"
            canvas.create_text(
                label_x + (4 if anchor == "w" else -4), label_y - 7,
                text=f"РЯД {index}", fill=color, font=(FONT_UI, 7, "bold"), anchor=anchor,
            )

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

        # Отметка очага из протокола и анимированная пульсирующая подсветка в реальном времени
        if self.protocol and self.protocol.get("x") is not None and self.protocol.get("y") is not None:
            px, py = self._world_to_canvas(float(self.protocol["x"]), float(self.protocol["y"]))
            is_victim = (
                str(self.protocol.get("hazard_type", "")).upper() in ("VICTIM", "HUMAN")
                or "ЧЕЛОВЕК" in str(self.protocol.get("title", "")).upper()
            )
            ripple_color = "#00E5FF" if is_victim else COLORS["crit"]

            # Анимированные концентрические круги радара (Radar Ripple)
            pulse_phase = (self.frame_no % 30) / 30.0
            for i in range(3):
                phase_i = (pulse_phase + i / 3.0) % 1.0
                r_i = int(12 + phase_i * 32)
                canvas.create_oval(px - r_i, py - r_i, px + r_i, py + r_i, outline=ripple_color, width=2 if phase_i < 0.5 else 1)

            canvas.create_oval(px - 14, py - 14, px + 14, py + 14, fill=ripple_color, outline="#FFFFFF", width=2)
            label_y = py - 38 if py - 38 > top + 10 else py + 32
            canvas.create_text(px, label_y, text=self.protocol["incident_id"], fill=ripple_color,
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
        canvas.create_text(
            dx + radius + 6, dy - 2, text=f"БПЛА · {self.altitude:.0f} м",
            fill=COLORS["drone"], font=(FONT_UI, 8, "bold"), anchor="w",
        )

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
        live_frame = frame is not None
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
            # У живого MJPEG уже есть HUD ноды; не дублируем ALT/YAW/FPS.
            frame = self._draw_hud(frame, frame.shape[1], frame.shape[0],
                                   include_telemetry=not live_frame)

        # Кадр «как видит оператор» — используется снимками и записью.
        self.last_frame = frame.copy()

        # Накопитель кадров полёта для видеорапорта миссии.
        now_mono = time.monotonic()
        if (now_mono - self._last_flight_frame_ts >= self._flight_frame_interval
                and len(self._flight_frames) < self._flight_frames_max):
            self._last_flight_frame_ts = now_mono
            self._flight_frames.append(self.last_frame.copy())

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

        # Виньетка — кадр выглядит как оптика, а не как заливка.
        # Маску считаем в 4 раза мельче: GaussianBlur большого радиуса на полном
        # кадре стоил секунды, а на плавном градиенте разницы не видно.
        small_w = max(16, width // 4)
        small_h = max(16, height // 4)
        mask = np.zeros((small_h, small_w), dtype=np.uint8)
        cv2.ellipse(mask, (small_w // 2, small_h // 2),
                    (int(small_w * 0.62), int(small_h * 0.62)), 0, 0, 360, 255, -1)
        mask = cv2.GaussianBlur(mask, (0, 0), max(small_w, small_h) / 8.0)
        mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_LINEAR)
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

    def _draw_hud(self, frame: np.ndarray, width: int, height: int,
                  include_telemetry: bool = True) -> np.ndarray:
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

        if include_telemetry:
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
        if include_telemetry:
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
        app.state_client.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())


# Совместимость с предыдущими скриптами и тестами
OkoMchsOperatorApp = OkoMchsApp
