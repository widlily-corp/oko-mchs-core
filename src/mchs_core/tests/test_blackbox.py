"""
Unit and Integration Tests for BlackBoxRecorder Telemetry Subsystem.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
import sys
import threading
import time

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from mchs_core.blackbox import CSV_HEADER, BlackBoxRecorder, TelemetryRecord


def test_blackbox_header_and_csv_creation(tmp_path):
    """Verify that CSV file is created with the exact required header."""
    log_dir = tmp_path / "bb_logs"
    with BlackBoxRecorder(log_dir=log_dir, flush_interval_s=0.05) as bb:
        pass

    csv_path = log_dir / "telemetry_blackbox.csv"
    assert csv_path.exists()

    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader)
        assert header == CSV_HEADER


def test_blackbox_record_telemetry_csv_and_jsonl(tmp_path):
    """Verify recording of telemetry frames to both CSV and JSONL."""
    log_dir = tmp_path / "bb_logs"
    with BlackBoxRecorder(log_dir=log_dir, flush_interval_s=0.05) as bb:
        rec = bb.record_telemetry(
            x=12.5,
            y=-3.2,
            z=15.0,
            yaw_deg=180.0,
            vx=2.5,
            vy=0.0,
            vz=-0.2,
            battery_pct=92.5,
            state="CRUISING",
            hazard_alert=False,
            message="En route to waypoint Bravo",
        )
        assert isinstance(rec, TelemetryRecord)
        bb.flush()

    csv_path = log_dir / "telemetry_blackbox.csv"
    jsonl_path = log_dir / "telemetry_blackbox.jsonl"

    assert csv_path.exists()
    assert jsonl_path.exists()

    # Read CSV row
    with open(csv_path, "r", encoding="utf-8") as f:
        rows = list(csv.reader(f))
        assert len(rows) == 2  # header + 1 row
        data_row = rows[1]
        assert data_row[2] == "12.500"  # x
        assert data_row[3] == "-3.200"  # y
        assert data_row[4] == "15.000"  # z
        assert data_row[5] == "180.00"  # yaw_deg
        assert data_row[6] == "2.500"   # vx
        assert data_row[7] == "0.000"   # vy
        assert data_row[8] == "-0.200"  # vz
        assert data_row[9] == "92.5"    # battery_pct
        assert data_row[10] == "CRUISING"
        assert data_row[11] == "False"
        assert data_row[12] == "En route to waypoint Bravo"

    # Read JSONL line
    lines = jsonl_path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 1
    record_dict = json.loads(lines[0])
    assert record_dict["x"] == 12.5
    assert record_dict["y"] == -3.2
    assert record_dict["z"] == 15.0
    assert record_dict["yaw_deg"] == 180.0
    assert record_dict["battery_pct"] == 92.5
    assert record_dict["state"] == "CRUISING"
    assert record_dict["hazard_alert"] is False
    assert record_dict["message"] == "En route to waypoint Bravo"


def test_blackbox_record_event(tmp_path):
    """Verify operational events recorded via record_event."""
    log_dir = tmp_path / "bb_logs"
    with BlackBoxRecorder(log_dir=log_dir, flush_interval_s=0.05) as bb:
        bb.record_event(
            event_type="GEO_FENCE_BREACH",
            message="Boundary exceeded at sector 4",
            hazard_alert=True,
            state="EMERGENCY_RTH",
            x=100.2,
            y=200.5,
            z=30.0,
        )
        bb.flush()

    jsonl_path = log_dir / "telemetry_blackbox.jsonl"
    lines = jsonl_path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 1
    ev = json.loads(lines[0])

    assert ev["state"] == "EMERGENCY_RTH"
    assert ev["hazard_alert"] is True
    assert "[GEO_FENCE_BREACH]" in ev["message"]
    assert "Boundary exceeded at sector 4" in ev["message"]
    assert ev["extra"]["event_type"] == "GEO_FENCE_BREACH"


def test_blackbox_multithreaded_stress(tmp_path):
    """Stress test concurrent telemetry writes from multiple threads."""
    log_dir = tmp_path / "bb_logs"
    num_threads = 6
    records_per_thread = 100

    with BlackBoxRecorder(log_dir=log_dir, flush_interval_s=0.05) as bb:
        def flight_thread(tid: int):
            for i in range(records_per_thread):
                bb.record_telemetry(
                    x=float(tid),
                    y=float(i),
                    z=10.0 + tid,
                    state="TEST_CONCURRENCY",
                    message=f"Thread {tid} tick {i}",
                )

        threads = [threading.Thread(target=flight_thread, args=(i,)) for i in range(num_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        bb.flush()

    csv_path = log_dir / "telemetry_blackbox.csv"
    jsonl_path = log_dir / "telemetry_blackbox.jsonl"

    expected_records = num_threads * records_per_thread

    with open(csv_path, "r", encoding="utf-8") as f:
        rows = list(csv.reader(f))
        # 1 header + expected_records
        assert len(rows) == expected_records + 1

    lines = jsonl_path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == expected_records

    # Validate JSON integrity
    for line in lines:
        data = json.loads(line)
        assert data["state"] == "TEST_CONCURRENCY"


def test_blackbox_closed_error(tmp_path):
    """Verify that writing to closed recorder raises RuntimeError."""
    log_dir = tmp_path / "bb_logs"
    bb = BlackBoxRecorder(log_dir=log_dir)
    bb.close()

    with pytest.raises(RuntimeError):
        bb.record_telemetry(x=1.0, y=2.0)
