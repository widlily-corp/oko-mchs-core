"""
Юнит- и интеграционные тесты для утилиты экспресс-диагностики scripts/diagnose_logs.py.
"""

from __future__ import annotations

import csv
import json
import os
from pathlib import Path
import sys
import zipfile
import pytest

# Подключение корня репозитория и скриптов в sys.path
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(REPO_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "scripts"))

from scripts.diagnose_logs import (
    analyze_blackbox,
    build_timeline,
    create_diagnostic_bundle,
    get_system_summary,
    main,
    parse_timestamp_to_epoch,
    scan_errors,
)


@pytest.fixture
def temp_logs_dir(tmp_path: Path) -> Path:
    """Создает временную директорию с логами для тестов."""
    logs_dir = tmp_path / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    return logs_dir


@pytest.fixture
def populated_logs_dir(temp_logs_dir: Path) -> Path:
    """Заполняет временную директорию синтетическими логами, событиями и крашем."""
    # 1. system.log
    sys_log = temp_logs_dir / "system.log"
    sys_log.write_text(
        "[2026-11-11 12:00:00.100] [CORE] [INFO] System started.\n"
        "[2026-11-11 12:00:01.200] [PILOT] [INFO] Takeoff command issued.\n"
        "[2026-11-11 12:00:02.300] [PILOT] [WARNING] GPS signal degradation HDOP=2.8.\n"
        "[2026-11-11 12:00:03.400] [VISION] [ERROR] Optical camera frame drop detected.\n"
        "[2026-11-11 12:00:04.500] [PILOT] [CRITICAL] Motor #3 desync failure!\n",
        encoding="utf-8",
    )

    # 2. events.jsonl
    events_file = temp_logs_dir / "events.jsonl"
    events = [
        {
            "timestamp": "2026-11-11T12:00:00.150Z",
            "epoch_s": 1794408000.15,
            "subsystem": "CORE",
            "event_type": "BOOT",
            "level": "INFO",
            "data": {"status": "OK"},
        },
        {
            "timestamp": "2026-11-11T12:00:02.350Z",
            "epoch_s": 1794408002.35,
            "subsystem": "PILOT",
            "event_type": "NAV_WARN",
            "level": "WARNING",
            "data": {"hdop": 2.8},
        },
        {
            "timestamp": "2026-11-11T12:00:04.550Z",
            "epoch_s": 1794408004.55,
            "subsystem": "PILOT",
            "event_type": "MOTOR_FAIL",
            "level": "CRITICAL",
            "data": {"motor_id": 3, "rpm": 0},
        },
    ]
    with open(events_file, "w", encoding="utf-8") as f:
        for ev in events:
            f.write(json.dumps(ev, ensure_ascii=False) + "\n")

    # 3. crashes/
    crashes_dir = temp_logs_dir / "crashes"
    crashes_dir.mkdir(parents=True, exist_ok=True)
    crash_file = crashes_dir / "crash_pilot_20261111_120005.log"
    crash_file.write_text(
        "=== MCHS CRASH DUMP ===\n"
        "Timestamp: 2026-11-11 12:00:05\n"
        "Subsystem: PILOT\n"
        "Traceback (most recent call last):\n"
        "  File 'mchs_pilot_node.py', line 120, in control_loop\n"
        "    raise RuntimeError('Motor hardware desync')\n"
        "RuntimeError: Motor hardware desync\n",
        encoding="utf-8",
    )

    # 4. telemetry_blackbox.csv
    bb_file = temp_logs_dir / "telemetry_blackbox.csv"
    with open(bb_file, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "epoch_s", "x", "y", "z", "yaw_deg",
            "vx", "vy", "vz", "battery_pct", "state", "hazard_alert", "message"
        ])
        # t=0s
        writer.writerow([
            "2026-11-11T12:00:00.000Z", "1794408000.0000",
            "0.000", "0.000", "0.000", "0.00", "0.000", "0.000", "0.000",
            "100.0", "IDLE", "False", "System ready"
        ])
        # t=10s: takeoff
        writer.writerow([
            "2026-11-11T12:00:10.000Z", "1794408010.0000",
            "0.000", "0.000", "10.000", "0.00", "0.000", "0.000", "1.000",
            "99.0", "TAKEOFF", "False", "Ascending"
        ])
        # t=20s: fly
        writer.writerow([
            "2026-11-11T12:00:20.000Z", "1794408020.0000",
            "30.000", "40.000", "10.000", "45.00", "3.000", "4.000", "0.000",
            "97.0", "PATROL", "False", "En route"
        ])
        # t=30s: hazard inspect
        writer.writerow([
            "2026-11-11T12:00:30.000Z", "1794408030.0000",
            "30.000", "40.000", "10.000", "45.00", "0.000", "0.000", "0.000",
            "95.0", "INSPECT", "True", "Fire detected"
        ])
        # t=40s: RTL
        writer.writerow([
            "2026-11-11T12:00:40.000Z", "1794408040.0000",
            "0.000", "0.000", "5.000", "180.00", "-3.000", "-4.000", "-0.500",
            "93.0", "RTL", "False", "Returning"
        ])
        # t=60s: Landed
        writer.writerow([
            "2026-11-11T12:01:00.000Z", "1794408060.0000",
            "0.000", "0.000", "0.000", "0.00", "0.000", "0.000", "0.000",
            "90.0", "LANDED", "False", "Touchdown"
        ])

    return temp_logs_dir


def test_parse_timestamp():
    """Проверка корректности парсинга различных строковых форматов времени."""
    epoch1 = parse_timestamp_to_epoch("2026-11-11T12:00:00.000Z")
    assert epoch1 is not None and epoch1 > 0

    epoch2 = parse_timestamp_to_epoch("2026-11-11 12:00:00.123")
    assert epoch2 is not None and epoch2 > 0

    epoch3 = parse_timestamp_to_epoch("1794408000.5")
    assert epoch3 == 1794408000.5

    assert parse_timestamp_to_epoch("invalid-string") is None


def test_scan_errors_with_faults(populated_logs_dir: Path):
    """Проверка поиска ошибок WARN, ERROR, CRITICAL и крашей."""
    results = scan_errors(populated_logs_dir)

    assert results["has_critical_or_crash"] is True
    assert len(results["crashes"]) == 1
    assert results["crashes"][0]["subsystem"] == "PILOT"

    # CRITICAL: 1 из system.log + 1 из events.jsonl = 2
    assert len(results["criticals"]) == 2
    critical_msgs = [c["message"] for c in results["criticals"]]
    assert any("Motor #3 desync failure" in m for m in critical_msgs)
    assert any("MOTOR_FAIL" in m for m in critical_msgs)

    # ERROR: 1 из system.log
    assert len(results["errors"]) == 1
    assert "frame drop" in results["errors"][0]["message"]

    # WARNING: 1 из system.log + 1 из events.jsonl = 2
    assert len(results["warnings"]) == 2


def test_scan_errors_clean_directory(temp_logs_dir: Path):
    """Проверка сканирования чистой директории без ошибок."""
    clean_log = temp_logs_dir / "system.log"
    clean_log.write_text(
        "[2026-11-11 12:00:00.100] [CORE] [INFO] All subsystems initialized.\n"
        "[2026-11-11 12:00:01.200] [GCS_UI] [INFO] UI opened.\n",
        encoding="utf-8",
    )
    results = scan_errors(temp_logs_dir)
    assert results["has_critical_or_crash"] is False
    assert len(results["crashes"]) == 0
    assert len(results["criticals"]) == 0
    assert len(results["errors"]) == 0
    assert len(results["warnings"]) == 0


def test_build_timeline(populated_logs_dir: Path):
    """Проверка построения объединенного таймлайна событий со всех источников."""
    timeline = build_timeline(populated_logs_dir)

    assert len(timeline) > 0
    # Проверка источников
    sources = {t["source"] for t in timeline}
    assert "system.log" in sources
    assert "events.jsonl" in sources
    assert "blackbox.csv" in sources

    # Проверка хронологической сортировки
    epochs = [t["epoch"] for t in timeline]
    assert epochs == sorted(epochs)

    # Проверка фильтрации по since_minutes
    # Все записи у нас в районе 12:00:00 - 12:01:00 (разброс 60 сек = 1 минута)
    # Если фильтруем за последние 0.2 минуты (12 сек), записей должно быть меньше
    filtered = build_timeline(populated_logs_dir, since_minutes=0.2)
    assert len(filtered) < len(timeline)


def test_analyze_blackbox(populated_logs_dir: Path):
    """Проверка расчетных характеристик полетного регистратора."""
    csv_file = populated_logs_dir / "telemetry_blackbox.csv"
    stats = analyze_blackbox(csv_file)

    assert "error" not in stats
    assert stats["total_records"] == 6
    assert stats["duration_s"] == 60.0  # 12:00:00 -> 12:01:00
    assert stats["max_altitude_m"] == 10.0
    assert stats["start_battery_pct"] == 100.0
    assert stats["end_battery_pct"] == 90.0
    assert stats["battery_consumed_pct"] == 10.0
    # Расход: 10% за 1 мин = 10 %/мин
    assert stats["battery_rate_per_min"] == 10.0
    assert stats["hazard_alerts_count"] == 1

    # Проверка расстояния: (0,0,0)->(0,0,10): 10м; ->(30,40,10): 50м; ->(0,0,5): 50.25м; ->(0,0,0): 5м => ~115м
    assert stats["total_distance_m"] > 100.0

    # Проверка FSM-состояний
    assert "PATROL" in stats["state_percentages"]
    assert "INSPECT" in stats["state_percentages"]


def test_create_diagnostic_bundle(populated_logs_dir: Path, tmp_path: Path):
    """Проверка сборки zip-архива logs/ и reports/."""
    reports_dir = tmp_path / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    sample_report = reports_dir / "mission_protocol.json"
    sample_report.write_text('{"mission": "test"}', encoding="utf-8")

    out_dir = tmp_path / "bundles"
    bundle_path = create_diagnostic_bundle(
        logs_dir=populated_logs_dir,
        reports_dir=reports_dir,
        output_dir=out_dir,
    )

    assert bundle_path.exists()
    assert bundle_path.suffix == ".zip"
    assert "incident_bundle_" in bundle_path.name

    # Проверка содержимого архива
    with zipfile.ZipFile(bundle_path, "r") as zf:
        names = zf.namelist()
        assert "logs/system.log" in names
        assert "logs/events.jsonl" in names
        assert any(n.startswith("logs/crashes/") for n in names)
        assert "reports/mission_protocol.json" in names
        assert "bundle_manifest.json" in names

        # Чтение манифеста
        manifest = json.loads(zf.read("bundle_manifest.json").decode("utf-8"))
        assert "created_at" in manifest
        assert manifest["bundle_version"] == "1.0"


def test_system_summary(populated_logs_dir: Path):
    """Проверка формирования сводного статуса здоровья."""
    summary = get_system_summary(populated_logs_dir)
    assert summary["verdict"] == "CRITICAL"
    assert summary["crashes_count"] == 1
    assert summary["critical_count"] == 2
    assert summary["error_count"] == 1
    assert summary["warning_count"] == 2
    assert summary["blackbox_available"] is True
    assert summary["blackbox_records"] == 6


def test_cli_execution_with_args(populated_logs_dir: Path, tmp_path: Path):
    """Проверка CLI-интерфейса main() и кодов завершения."""
    # 1. Запуск с ошибками -> exit code 1 (есть краш и CRITICAL)
    code = main(["--log-dir", str(populated_logs_dir), "--errors"])
    assert code == 1

    # 2. Запуск таймлайна -> exit code 0
    code = main(["--log-dir", str(populated_logs_dir), "--timeline"])
    assert code == 0

    # 3. Запуск анализа черного ящика -> exit code 0
    code = main(["--log-dir", str(populated_logs_dir), "--blackbox"])
    assert code == 0

    # 4. Запуск сборки бандла -> exit code 0
    bundles_dir = tmp_path / "cli_bundles"
    code = main([
        "--log-dir", str(populated_logs_dir),
        "--reports-dir", str(tmp_path),
        "--bundle",
    ])
    assert code == 0

    # 5. Запуск с чистой директорией -> exit code 0
    clean_dir = tmp_path / "clean_logs"
    clean_dir.mkdir(parents=True, exist_ok=True)
    code = main(["--log-dir", str(clean_dir), "--errors"])
    assert code == 0
