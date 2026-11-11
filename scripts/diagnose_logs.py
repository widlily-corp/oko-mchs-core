#!/usr/bin/env python3
"""
ОКО МЧС — УТИЛИТА ЭКСПРЕСС-ДИАГНОСТИКИ И АНАЛИЗА ЛОГОВ (diagnose_logs.py)

Автономный инструмент расследования инцидентов, анализа телеметрии и сбора бандлов:
  --errors     : Поиск WARNING, ERROR, CRITICAL и краш-дампов (exit code 1 при аварии)
  --timeline   : Объединенная хронологическая лента событий (system.log, events.jsonl, blackbox)
  --blackbox   : Статистика летного происшествия по telemetry_blackbox.csv
  --bundle     : Сборка аварийного zip-архива logs/ и reports/
  --summary    : Сводный статус здоровья комплекса (по умолчанию)
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import math
import os
from pathlib import Path
import re
import sys
import zipfile
from typing import Any, Dict, List, Optional, Tuple

# Безопасная настройка вывода UTF-8 для Windows консоли
if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
if sys.stderr and hasattr(sys.stderr, "reconfigure"):
    try:
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass



def get_default_repo_root() -> Path:
    """Определить корень репозитория относительно scripts/."""
    return Path(__file__).resolve().parents[1]


# Регулярное выражение для лог-строк формата:
# [YYYY-MM-DD HH:MM:SS.mmm] [SUBSYSTEM] [LEVEL] Message
LOG_LINE_REGEX = re.compile(
    r"^\[(?P<timestamp>\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}(?:\.\d+)?)\]\s+"
    r"\[(?P<subsystem>[^\]]+)\]\s+"
    r"\[(?P<level>[A-Z]+)\]\s+"
    r"(?P<message>.*)$"
)


def parse_timestamp_to_epoch(ts_str: str) -> Optional[float]:
    """Парсинг различных форматов даты-времени в epoch float."""
    ts_str = ts_str.strip()
    # 1. ISO 8601: 2026-11-11T14:14:59.123Z or without Z
    for iso_fmt in (
        "%Y-%m-%dT%H:%M:%S.%fZ",
        "%Y-%m-%dT%H:%M:%SZ",
        "%Y-%m-%dT%H:%M:%S.%f",
        "%Y-%m-%dT%H:%M:%S",
    ):
        try:
            dt = datetime.datetime.strptime(ts_str, iso_fmt)
            return dt.replace(tzinfo=datetime.timezone.utc).timestamp()
        except ValueError:
            pass

    # 2. Log format: 2026-11-11 14:14:59.123 or 2026-11-11 14:14:59
    for log_fmt in (
        "%Y-%m-%d %H:%M:%S.%f",
        "%Y-%m-%d %H:%M:%S",
    ):
        try:
            dt = datetime.datetime.strptime(ts_str, log_fmt)
            return dt.replace(tzinfo=datetime.timezone.utc).timestamp()
        except ValueError:
            pass

    # 3. Epoch float string
    try:
        return float(ts_str)
    except ValueError:
        return None


# ==============================================================================
# 1. СКАНИРОВАНИЕ ОШИБОК (--errors)
# ==============================================================================

def scan_errors(logs_dir: Path) -> Dict[str, Any]:
    """
    Сканирует директорию логов на предмет записей WARNING, ERROR, CRITICAL
    и файлов краш-дампов в crashes/.
    
    Возвращает словарь:
    {
        "warnings": [...],
        "errors": [...],
        "criticals": [...],
        "crashes": [...],
        "has_critical_or_crash": bool,
    }
    """
    logs_dir = Path(logs_dir)
    results: Dict[str, Any] = {
        "warnings": [],
        "errors": [],
        "criticals": [],
        "crashes": [],
        "has_critical_or_crash": False,
    }

    if not logs_dir.exists():
        return results

    # 1. Сканирование текстовых .log файлов
    for log_file in sorted(logs_dir.glob("*.log")):
        try:
            with open(log_file, "r", encoding="utf-8", errors="replace") as f:
                for line_no, raw_line in enumerate(f, 1):
                    line = raw_line.strip()
                    if not line:
                        continue
                    m = LOG_LINE_REGEX.match(line)
                    if m:
                        level = m.group("level").upper()
                        entry = {
                            "file": log_file.name,
                            "line": line_no,
                            "timestamp": m.group("timestamp"),
                            "subsystem": m.group("subsystem"),
                            "level": level,
                            "message": m.group("message"),
                        }
                        if level == "CRITICAL":
                            results["criticals"].append(entry)
                        elif level == "ERROR":
                            results["errors"].append(entry)
                        elif level == "WARNING":
                            results["warnings"].append(entry)
        except Exception as e:
            results["errors"].append({
                "file": log_file.name,
                "line": 0,
                "timestamp": "",
                "subsystem": "DIAG",
                "level": "ERROR",
                "message": f"Ошибка чтения лог-файла {log_file.name}: {e}",
            })

    # 2. Сканирование events.jsonl
    events_file = logs_dir / "events.jsonl"
    if events_file.exists():
        try:
            with open(events_file, "r", encoding="utf-8", errors="replace") as f:
                for line_no, raw_line in enumerate(f, 1):
                    line = raw_line.strip()
                    if not line:
                        continue
                    try:
                        obj = json.loads(line)
                        level = str(obj.get("level", "INFO")).upper()
                        if level in ("CRITICAL", "ERROR", "WARNING"):
                            entry = {
                                "file": "events.jsonl",
                                "line": line_no,
                                "timestamp": obj.get("timestamp", ""),
                                "subsystem": obj.get("subsystem", "CORE"),
                                "level": level,
                                "message": f"[{obj.get('event_type')}] {json.dumps(obj.get('data', {}), ensure_ascii=False)}",
                            }
                            if level == "CRITICAL":
                                results["criticals"].append(entry)
                            elif level == "ERROR":
                                results["errors"].append(entry)
                            elif level == "WARNING":
                                results["warnings"].append(entry)
                    except json.JSONDecodeError:
                        continue
        except Exception as e:
            results["errors"].append({
                "file": "events.jsonl",
                "line": 0,
                "timestamp": "",
                "subsystem": "DIAG",
                "level": "ERROR",
                "message": f"Ошибка парсинга events.jsonl: {e}",
            })

    # 3. Сканирование crashes/
    crashes_dir = logs_dir / "crashes"
    if crashes_dir.exists():
        for crash_file in sorted(crashes_dir.glob("*.log")):
            crash_info = {
                "file": crash_file.name,
                "path": str(crash_file),
                "summary": "Неизвестное аварийное завершение",
                "subsystem": "UNKNOWN",
                "timestamp": "",
            }
            try:
                with open(crash_file, "r", encoding="utf-8", errors="replace") as cf:
                    header_lines = [cf.readline() for _ in range(15)]
                    for hl in header_lines:
                        if hl.startswith("Subsystem:"):
                            crash_info["subsystem"] = hl.split(":", 1)[1].strip()
                        elif hl.startswith("Timestamp:"):
                            crash_info["timestamp"] = hl.split(":", 1)[1].strip()
                        elif "Exception:" in hl or "Error:" in hl:
                            crash_info["summary"] = hl.strip()
            except Exception:
                pass
            results["crashes"].append(crash_info)

    results["has_critical_or_crash"] = bool(results["criticals"] or results["crashes"])
    return results


def print_errors_report(data: Dict[str, Any]) -> None:
    """Вывод отчета об ошибках в консоль с цветовой дифференциацией."""
    print("=" * 80)
    print(" 🚨 АУДИТ ОШИБОК И СБОЕВ СИСТЕМЫ «ОКО МЧС»")
    print("=" * 80)

    # 1. Краши
    crashes = data.get("crashes", [])
    print(f"\n💥 АВАРИЙНЫЕ КРАШ-ДАМПЫ (CRASHES): {len(crashes)}")
    if crashes:
        for c in crashes:
            print(f"  • [{c['subsystem']}] {c['file']}")
            if c['timestamp']:
                print(f"    Время: {c['timestamp']}")
            print(f"    Суть:  {c['summary']}")
            print(f"    Путь:  {c.get('path')}")
    else:
        print("  ✓ Не зарегистрировано аварийных завершений.")

    # 2. Critical
    criticals = data.get("criticals", [])
    print(f"\n🔴 КРИТИЧЕСКИЕ ОШИБКИ (CRITICAL): {len(criticals)}")
    if criticals:
        for item in criticals:
            print(f"  • [{item['timestamp']}] [{item['subsystem']}] {item['file']}:{item['line']}")
            print(f"    {item['message']}")
    else:
        print("  ✓ Критических ошибок уровня CRITICAL нет.")

    # 3. Errors
    errors = data.get("errors", [])
    print(f"\n❌ ОШИБКИ (ERROR): {len(errors)}")
    if errors:
        for item in errors:
            print(f"  • [{item['timestamp']}] [{item['subsystem']}] {item['file']}:{item['line']}")
            print(f"    {item['message']}")
    else:
        print("  ✓ Ошибок уровня ERROR нет.")

    # 4. Warnings
    warnings = data.get("warnings", [])
    print(f"\n⚠️ ПРЕДУПРЕЖДЕНИЯ (WARNING): {len(warnings)}")
    if warnings:
        for item in warnings[:20]:  # Ограничение вывода до первых 20
            print(f"  • [{item['timestamp']}] [{item['subsystem']}] {item['file']}:{item['line']}")
            print(f"    {item['message']}")
        if len(warnings) > 20:
            print(f"  ... и еще {len(warnings) - 20} предупреждений.")
    else:
        print("  ✓ Предупреждений нет.")

    print("\n" + "-" * 80)
    if data["has_critical_or_crash"]:
        print("🚨 ВЕРДИКТ: ОБНАРУЖЕНЫ КРИТИЧЕСКИЕ ИНЦИДЕНТЫ ИЛИ ПАДЕНИЯ! ТРЕБУЕТСЯ ВМЕШАТЕЛЬСТВО.")
    elif errors:
        print("⚠️ ВЕРДИКТ: Обнаружены системные ошибки (ERROR), но падений не зафиксировано.")
    elif warnings:
        print("ℹ️ ВЕРДИКТ: Комплекс стабилен, имеются предупреждения (WARNING).")
    else:
        print("✅ ВЕРДИКТ: Все подсистемы работают в штатном бессбойном режиме.")
    print("-" * 80)


# ==============================================================================
# 2. ОБЪЕДИНЕННЫЙ ТАЙМЛАЙН (--timeline)
# ==============================================================================

def build_timeline(
    logs_dir: Path,
    since_minutes: Optional[float] = None,
) -> List[Dict[str, Any]]:
    """
    Формирует объединенную хронологическую ленту событий из:
      - system.log (текстовые записи)
      - events.jsonl (структурированные события)
      - telemetry_blackbox.csv (ключевые вехи телеметрии и алерты)
    
    Сортирует все записи по времени.
    """
    logs_dir = Path(logs_dir)
    timeline: List[Dict[str, Any]] = []

    # 1. Чтение system.log
    sys_log = logs_dir / "system.log"
    if sys_log.exists():
        try:
            with open(sys_log, "r", encoding="utf-8", errors="replace") as f:
                for line in f:
                    line = line.strip()
                    m = LOG_LINE_REGEX.match(line)
                    if m:
                        ts_str = m.group("timestamp")
                        epoch = parse_timestamp_to_epoch(ts_str) or 0.0
                        timeline.append({
                            "epoch": epoch,
                            "timestamp": ts_str,
                            "source": "system.log",
                            "subsystem": m.group("subsystem"),
                            "type": m.group("level"),
                            "message": m.group("message"),
                        })
        except Exception:
            pass

    # 2. Чтение events.jsonl
    events_file = logs_dir / "events.jsonl"
    if events_file.exists():
        try:
            with open(events_file, "r", encoding="utf-8", errors="replace") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        rec = json.loads(line)
                        epoch = float(rec.get("epoch_s", 0.0))
                        if epoch == 0.0:
                            epoch = parse_timestamp_to_epoch(rec.get("timestamp", "")) or 0.0
                        data_str = json.dumps(rec.get("data", {}), ensure_ascii=False)
                        timeline.append({
                            "epoch": epoch,
                            "timestamp": rec.get("timestamp", ""),
                            "source": "events.jsonl",
                            "subsystem": rec.get("subsystem", "CORE"),
                            "type": f"EVENT:{rec.get('event_type')}",
                            "message": data_str,
                        })
                    except Exception:
                        continue
        except Exception:
            pass

    # 3. Чтение telemetry_blackbox.csv (события со сменой состояния или алертом)
    bb_csv = logs_dir / "telemetry_blackbox.csv"
    if bb_csv.exists():
        try:
            with open(bb_csv, "r", encoding="utf-8", errors="replace") as f:
                reader = csv.DictReader(f)
                prev_state = None
                for row in reader:
                    state = row.get("state", "")
                    hazard = row.get("hazard_alert", "False").strip().lower() in ("true", "1")
                    msg = row.get("message", "")
                    # Фильтруем только знаковые моменты: смена состояния, алерт или сообщение
                    is_significant = (state != prev_state) or hazard or (msg and msg != "System ready")
                    if is_significant:
                        epoch = float(row.get("epoch_s", 0.0)) if row.get("epoch_s") else 0.0
                        if epoch == 0.0:
                            epoch = parse_timestamp_to_epoch(row.get("timestamp", "")) or 0.0
                        detail = f"State: {state} | Alt: {row.get('z')}m | Bat: {row.get('battery_pct')}%"
                        if hazard:
                            detail = f"🚨 HAZARD! {detail}"
                        if msg:
                            detail = f"{detail} | {msg}"
                        timeline.append({
                            "epoch": epoch,
                            "timestamp": row.get("timestamp", ""),
                            "source": "blackbox.csv",
                            "subsystem": "PILOT",
                            "type": f"FSM:{state}",
                            "message": detail,
                        })
                        prev_state = state
        except Exception:
            pass

    # Сортировка по epoch
    timeline.sort(key=lambda x: x["epoch"])

    # Фильтрация по since_minutes
    if since_minutes is not None and since_minutes > 0 and timeline:
        max_epoch = timeline[-1]["epoch"]
        cutoff = max_epoch - (since_minutes * 60.0)
        timeline = [t for t in timeline if t["epoch"] >= cutoff]

    return timeline


def print_timeline_report(timeline: List[Dict[str, Any]]) -> None:
    """Печать таймлайна в консоль."""
    print("=" * 80)
    print(" ⏱ ОБЪЕДИНЕННАЯ ХРОНОЛОГИЯ СОБЫТИЙ МЧС (TIMELINE)")
    print("=" * 80)

    if not timeline:
        print("  (Записи событий отсутствуют)")
        return

    for t in timeline:
        ts = t["timestamp"][:23] if t["timestamp"] else "NO_TIME"
        src = f"[{t['source']}]".ljust(15)
        sub = f"[{t['subsystem']}]".ljust(10)
        typ = f"[{t['type']}]".ljust(18)
        print(f"{ts} {src} {sub} {typ} {t['message']}")

    print("-" * 80)
    print(f"Всего событий в ленте: {len(timeline)}")


# ==============================================================================
# 3. АНАЛИЗ ЧЕРНОГО ЯЩИКА (--blackbox)
# ==============================================================================

def analyze_blackbox(csv_path: Path) -> Dict[str, Any]:
    """
    Глубокий анализ полетного регистратора telemetry_blackbox.csv.
    
    Метрики:
    - Общее время полета (start, end, duration_s)
    - Пройденный путь (траекторная 3D интеграция)
    - Максимальная высота (max z)
    - Максимальная скорость (max sqrt(vx^2 + vy^2 + vz^2))
    - Средний расход батареи (%/мин)
    - Распределение времени по FSM-состояниям (%)
    - Количество зафиксированных ЧС / алертов
    """
    csv_path = Path(csv_path)
    if not csv_path.exists():
        return {"error": f"Файл черного ящика не найден: {csv_path}"}

    rows: List[Dict[str, Any]] = []
    try:
        with open(csv_path, "r", encoding="utf-8", errors="replace") as f:
            reader = csv.DictReader(f)
            for r in reader:
                rows.append(r)
    except Exception as e:
        return {"error": f"Ошибка чтения CSV: {e}"}

    if not rows:
        return {"error": "Черный ящик пуст (0 записей)"}

    # Анализ метрик
    total_records = len(rows)
    start_time_str = rows[0].get("timestamp", "")
    end_time_str = rows[-1].get("timestamp", "")

    start_epoch = float(rows[0].get("epoch_s", 0.0)) or parse_timestamp_to_epoch(start_time_str) or 0.0
    end_epoch = float(rows[-1].get("epoch_s", 0.0)) or parse_timestamp_to_epoch(end_time_str) or 0.0
    duration_s = max(0.0, end_epoch - start_epoch)

    # Интеграция пути и поиск максимумов
    total_distance_m = 0.0
    max_altitude_m = 0.0
    max_speed_ms = 0.0
    prev_pos: Optional[Tuple[float, float, float]] = None

    state_counts: Dict[str, int] = {}
    hazard_events = 0

    first_bat = None
    last_bat = None

    for row in rows:
        try:
            x = float(row.get("x", 0.0))
            y = float(row.get("y", 0.0))
            z = float(row.get("z", 0.0))
            vx = float(row.get("vx", 0.0))
            vy = float(row.get("vy", 0.0))
            vz = float(row.get("vz", 0.0))
            bat = float(row.get("battery_pct", 100.0))
            state = row.get("state", "UNKNOWN").strip().upper()
            hazard = row.get("hazard_alert", "False").strip().lower() in ("true", "1")

            if first_bat is None:
                first_bat = bat
            last_bat = bat

            if prev_pos is not None:
                dx = x - prev_pos[0]
                dy = y - prev_pos[1]
                dz = z - prev_pos[2]
                step = math.sqrt(dx * dx + dy * dy + dz * dz)
                # Отсекаем явные телепортационные выбросы (> 100м за кадр)
                if step < 100.0:
                    total_distance_m += step
            prev_pos = (x, y, z)

            if z > max_altitude_m:
                max_altitude_m = z

            speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            if speed > max_speed_ms:
                max_speed_ms = speed

            state_counts[state] = state_counts.get(state, 0) + 1
            if hazard:
                hazard_events += 1

        except (ValueError, TypeError):
            continue

    # Расход батареи
    bat_consumed = (first_bat - last_bat) if (first_bat is not None and last_bat is not None) else 0.0
    duration_min = duration_s / 60.0
    bat_rate_per_min = (bat_consumed / duration_min) if duration_min > 0.05 else 0.0

    # Проценты по состояниям
    state_percentages = {}
    for st, count in state_counts.items():
        state_percentages[st] = round((count / total_records) * 100.0, 1)

    return {
        "total_records": total_records,
        "start_time": start_time_str,
        "end_time": end_time_str,
        "duration_s": round(duration_s, 2),
        "total_distance_m": round(total_distance_m, 2),
        "max_altitude_m": round(max_altitude_m, 2),
        "max_speed_ms": round(max_speed_ms, 2),
        "start_battery_pct": first_bat or 100.0,
        "end_battery_pct": last_bat or 100.0,
        "battery_consumed_pct": round(bat_consumed, 1),
        "battery_rate_per_min": round(bat_rate_per_min, 2),
        "state_percentages": state_percentages,
        "hazard_alerts_count": hazard_events,
    }


def print_blackbox_report(stats: Dict[str, Any]) -> None:
    """Печать отчета по черному ящику в консоль."""
    print("=" * 80)
    print(" 📦 АНАЛИЗ БОРТОВОГО РЕГИСТРАТОРА ТЕЛЕМЕТРИИ (BLACKBOX)")
    print("=" * 80)

    if "error" in stats:
        print(f"❌ {stats['error']}")
        return

    dur = stats["duration_s"]
    mins = int(dur // 60)
    secs = int(dur % 60)

    print(f"• Всего записей телеметрии:    {stats['total_records']}")
    print(f"• Начало записи:               {stats['start_time']}")
    print(f"• Окончание записи:            {stats['end_time']}")
    print(f"• Общая длительность полета:   {mins} мин {secs:02d} сек ({dur} с)")
    print(f"• Пройденная дистанция:        {stats['total_distance_m']} м")
    print(f"• Максимальная высота (Z):     {stats['max_altitude_m']} м")
    print(f"• Максимальная скорость:       {stats['max_speed_ms']} м/с ({stats['max_speed_ms']*3.6:.1f} км/ч)")
    print(f"• Заряд батареи:               {stats['start_battery_pct']:.1f}% -> {stats['end_battery_pct']:.1f}% (дельта: {stats['battery_consumed_pct']:.1f}%)")
    print(f"• Темп разряда АКБ:            {stats['battery_rate_per_min']} %/мин")
    print(f"• Зафиксировано алертов ЧС:    {stats['hazard_alerts_count']}")

    print("\nРаспределение времени по режимам автопилота (FSM):")
    for state, pct in stats["state_percentages"].items():
        bar_len = int(pct / 2.5)
        bar = "█" * bar_len + "░" * (40 - bar_len)
        print(f"  {state.ljust(15)} |{bar}| {pct:5.1f}%")
    print("-" * 80)


# ==============================================================================
# 4. СОЗДАНИЕ ДИАГНОСТИЧЕСКОГО БАНДЛА (--bundle)
# ==============================================================================

def create_diagnostic_bundle(
    logs_dir: Optional[Path] = None,
    reports_dir: Optional[Path] = None,
    output_dir: Optional[Path] = None,
) -> Path:
    """
    Архивирует logs/ и reports/ в zip-архив вида:
      incident_bundle_<YYYYMMDD_HHMMSS>.zip
    
    Возвращает абсолютный путь к созданному архиву.
    """
    repo_root = get_default_repo_root()
    logs_dir = Path(logs_dir) if logs_dir else repo_root / "logs"
    reports_dir = Path(reports_dir) if reports_dir else repo_root / "reports"
    output_dir = Path(output_dir) if output_dir else repo_root / "reports"

    output_dir.mkdir(parents=True, exist_ok=True)
    now_tag = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bundle_name = f"incident_bundle_{now_tag}.zip"
    bundle_path = output_dir / bundle_name

    with zipfile.ZipFile(bundle_path, "w", zipfile.ZIP_DEFLATED) as zf:
        # Упаковка логов
        if logs_dir.exists():
            for root, _, files in os.walk(logs_dir):
                for f in files:
                    full_p = Path(root) / f
                    # Не упаковываем временные файлы
                    if f.endswith(".tmp") or f.endswith(".lock"):
                        continue
                    arcname = Path("logs") / full_p.relative_to(logs_dir)
                    zf.write(full_p, arcname=str(arcname))

        # Упаковка отчетов (исключая сам создаваемый архив)
        if reports_dir.exists():
            for root, _, files in os.walk(reports_dir):
                for f in files:
                    full_p = Path(root) / f
                    if full_p.resolve() == bundle_path.resolve():
                        continue
                    arcname = Path("reports") / full_p.relative_to(reports_dir)
                    zf.write(full_p, arcname=str(arcname))

        # Добавим метаданные сборки бандла
        manifest = {
            "created_at": datetime.datetime.now().isoformat(),
            "bundle_version": "1.0",
            "source_logs": str(logs_dir),
            "source_reports": str(reports_dir),
        }
        zf.writestr("bundle_manifest.json", json.dumps(manifest, indent=2, ensure_ascii=False))

    return bundle_path.resolve()


# ==============================================================================
# 5. СВОДНЫЙ СТАТУС ЗДОРОВЬЯ (--summary)
# ==============================================================================

def get_system_summary(logs_dir: Path) -> Dict[str, Any]:
    """Собирает комплексный статус всех подсистем и файлов логов."""
    logs_dir = Path(logs_dir)
    errors_info = scan_errors(logs_dir)
    bb_path = logs_dir / "telemetry_blackbox.csv"
    bb_info = analyze_blackbox(bb_path) if bb_path.exists() else {}

    # Подсчет размера файлов
    log_files_count = 0
    total_bytes = 0
    if logs_dir.exists():
        for f in logs_dir.rglob("*"):
            if f.is_file():
                log_files_count += 1
                total_bytes += f.stat().st_size

    # Определение статуса здоровья
    if errors_info["has_critical_or_crash"]:
        verdict = "CRITICAL"
    elif errors_info.get("errors"):
        verdict = "DEGRADED"
    elif errors_info.get("warnings"):
        verdict = "WARNING"
    else:
        verdict = "HEALTHY"

    return {
        "verdict": verdict,
        "log_files_count": log_files_count,
        "total_log_size_kb": round(total_bytes / 1024.0, 1),
        "crashes_count": len(errors_info.get("crashes", [])),
        "critical_count": len(errors_info.get("criticals", [])),
        "error_count": len(errors_info.get("errors", [])),
        "warning_count": len(errors_info.get("warnings", [])),
        "blackbox_available": "error" not in bb_info and bool(bb_info),
        "blackbox_records": bb_info.get("total_records", 0),
        "blackbox_last_battery": bb_info.get("end_battery_pct", None),
    }


def print_summary_report(summary: Dict[str, Any]) -> None:
    """Печать сводного статуса здоровья."""
    print("=" * 80)
    print(" 🏥 СВОДНЫЙ СТАТУС ЗДОРОВЬЯ КОМПЛЕКСА «ОКО МЧС»")
    print("=" * 80)

    verdict = summary["verdict"]
    badge_map = {
        "HEALTHY": "🟢 [OK] ВСЕ СИСТЕМЫ В НОРМЕ",
        "WARNING": "🟡 [WARN] ЕСТЬ ПРЕДУПРЕЖДЕНИЯ",
        "DEGRADED": "🟠 [DEGRADED] СИСТЕМНЫЕ ОШИБКИ",
        "CRITICAL": "🔴 [CRITICAL] АВАРИЙНЫЕ СБОИ И КРАШИ",
    }
    print(f"Статус комплекса:        {badge_map.get(verdict, verdict)}")
    print(f"Каталог логов:           {summary['log_files_count']} файлов ({summary['total_log_size_kb']} КБ)")
    print(f"Аварийных дампов (crash): {summary['crashes_count']}")
    print(f"Критических инцидентов:   {summary['critical_count']}")
    print(f"Ошибок (ERROR):           {summary['error_count']}")
    print(f"Предупреждений (WARN):    {summary['warning_count']}")

    if summary["blackbox_available"]:
        print(f"Черный ящик:             ДОСТУПЕН ({summary['blackbox_records']} строк, АКБ: {summary['blackbox_last_battery']}%)")
    else:
        print("Черный ящик:             ДАННЫЕ ТЕЛЕМЕТРИИ ОТСУТСТВУЮТ")

    print("-" * 80)


# ==============================================================================
# MAIN CLI ENTRYPOINT
# ==============================================================================

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Утилита экспресс-диагностики логов комплекса «Око МЧС»"
    )
    repo_root = get_default_repo_root()
    default_logs_dir = repo_root / "logs"

    parser.add_argument(
        "--log-dir",
        type=Path,
        default=default_logs_dir,
        help="Путь к директории логов (по умолчанию: <repo_root>/logs)",
    )
    parser.add_argument(
        "--reports-dir",
        type=Path,
        default=repo_root / "reports",
        help="Путь к директории отчетов (по умолчанию: <repo_root>/reports)",
    )
    parser.add_argument(
        "--errors",
        action="store_true",
        help="Сканировать логи на наличие WARNING, ERROR, CRITICAL и крашей",
    )
    parser.add_argument(
        "--timeline",
        action="store_true",
        help="Построить объединенный хронологический таймлайн событий",
    )
    parser.add_argument(
        "--minutes",
        type=float,
        default=None,
        help="Ограничить таймлайн последними N минутами",
    )
    parser.add_argument(
        "--blackbox",
        action="store_true",
        help="Проанализировать полетный регистратор telemetry_blackbox.csv",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Указать конкретный CSV-файл для анализа черного ящика",
    )
    parser.add_argument(
        "--bundle",
        action="store_true",
        help="Упаковать logs/ и reports/ в zip-архив инцидента",
    )
    parser.add_argument(
        "--summary",
        action="store_true",
        help="Вывести сводный статус здоровья системы",
    )

    args = parser.parse_args(argv)

    # Если ни один флаг действия не указан — выполняем --summary
    has_action = args.errors or args.timeline or args.blackbox or args.bundle or args.summary
    if not has_action:
        args.summary = True

    exit_code = 0

    if args.errors:
        data = scan_errors(args.log_dir)
        print_errors_report(data)
        if data["has_critical_or_crash"]:
            exit_code = 1

    if args.timeline:
        tl = build_timeline(args.log_dir, since_minutes=args.minutes)
        print_timeline_report(tl)

    if args.blackbox:
        bb_file = args.csv if args.csv else (args.log_dir / "telemetry_blackbox.csv")
        stats = analyze_blackbox(bb_file)
        print_blackbox_report(stats)

    if args.bundle:
        b_path = create_diagnostic_bundle(
            logs_dir=args.log_dir,
            reports_dir=args.reports_dir,
            output_dir=args.reports_dir,
        )
        print(f"📦 Диагностический бандл успешно создан:")
        print(f"   {b_path}")

    if args.summary:
        summary = get_system_summary(args.log_dir)
        print_summary_report(summary)
        if summary["verdict"] == "CRITICAL":
            exit_code = 1

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
