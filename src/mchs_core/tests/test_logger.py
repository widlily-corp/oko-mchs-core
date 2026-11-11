"""
Unit and Integration Tests for MchsLogger and Crash Reporting.
"""

from __future__ import annotations

import json
from logging.handlers import RotatingFileHandler
from pathlib import Path
import sys
import threading
import time

import pytest

# Ensure mchs_core can be imported from any working directory
REPO_ROOT = Path(__file__).resolve().parents[3]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from mchs_core.logger import (
    MchsConsoleFormatter,
    MchsFileFormatter,
    MchsLogger,
    get_logger,
    install_crash_handler,
    log_event,
    reset_logging,
)


@pytest.fixture(autouse=True)
def clean_logging_state(tmp_path):
    """Isolate logging directory for each test."""
    reset_logging()
    test_logs_dir = tmp_path / "logs"
    test_logs_dir.mkdir(parents=True, exist_ok=True)
    MchsLogger.get_instance(log_dir=test_logs_dir, use_color=False)
    yield test_logs_dir
    reset_logging()


def test_logger_creation_and_singleton(clean_logging_state):
    """Test singleton behavior and logger initialization."""
    log_dir = clean_logging_state
    logger1 = get_logger("test_node", subsystem="PILOT")
    logger2 = get_logger("test_node", subsystem="PILOT")

    assert logger1 is logger2
    assert logger1.name == "mchs.PILOT.test_node"

    # Verify log files exist
    system_log = log_dir / "system.log"
    pilot_log = log_dir / "pilot.log"
    assert system_log.exists()
    assert pilot_log.exists()


def test_logger_formatting_and_subsystem_tag(clean_logging_state):
    """Test formatting with milliseconds and subsystem tag."""
    log_dir = clean_logging_state
    logger = get_logger("navigation", subsystem="PILOT")
    logger.info("Takeoff altitude reached: 15.0m")

    # Flush handlers
    for h in logger.handlers:
        h.flush()

    system_log = log_dir / "system.log"
    pilot_log = log_dir / "pilot.log"

    content_sys = system_log.read_text(encoding="utf-8")
    content_pilot = pilot_log.read_text(encoding="utf-8")

    assert "[PILOT]" in content_sys
    assert "[INFO]" in content_sys
    assert "Takeoff altitude reached: 15.0m" in content_sys

    assert "[PILOT]" in content_pilot
    assert "[INFO]" in content_pilot
    assert "Takeoff altitude reached: 15.0m" in content_pilot


def test_subsystem_isolation(clean_logging_state):
    """Test that logs are routed to correct subsystem files without cross-contamination."""
    log_dir = clean_logging_state
    pilot_logger = get_logger("flight_ctrl", subsystem="PILOT")
    vision_logger = get_logger("detector", subsystem="VISION")

    pilot_logger.info("Motor speed calibrated")
    vision_logger.warning("Hazard detected: thermal hotspot")

    for h in pilot_logger.handlers + vision_logger.handlers:
        h.flush()

    pilot_content = (log_dir / "pilot.log").read_text(encoding="utf-8")
    vision_content = (log_dir / "vision.log").read_text(encoding="utf-8")
    system_content = (log_dir / "system.log").read_text(encoding="utf-8")

    # Pilot log checks
    assert "Motor speed calibrated" in pilot_content
    assert "Hazard detected" not in pilot_content

    # Vision log checks
    assert "Hazard detected: thermal hotspot" in vision_content
    assert "Motor speed calibrated" not in vision_content

    # Central system log checks (must contain both)
    assert "Motor speed calibrated" in system_content
    assert "Hazard detected: thermal hotspot" in system_content


def test_rotating_handler_configuration(clean_logging_state):
    """Verify RotatingFileHandler attributes: 10MB maxBytes, 5 backup count, utf-8."""
    logger = get_logger("test_rot", subsystem="CORE")
    file_handlers = [h for h in logger.handlers if isinstance(h, RotatingFileHandler)]

    assert len(file_handlers) >= 2  # system.log and core.log
    for fh in file_handlers:
        assert fh.maxBytes == 10 * 1024 * 1024
        assert fh.backupCount == 5
        assert fh.encoding.lower() in ("utf-8", "utf8")


def test_log_event_jsonl(clean_logging_state):
    """Test structured events recorded in logs/events.jsonl."""
    log_dir = clean_logging_state
    event_data = {
        "sector": "SE-4",
        "temperature_c": 87.5,
        "perimeter_m": 42.0,
    }

    record = log_event(
        subsystem="VISION",
        event_type="FIRE_SECTOR_IDENTIFIED",
        data=event_data,
        level="WARNING",
    )

    events_file = log_dir / "events.jsonl"
    assert events_file.exists()

    lines = events_file.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) >= 1

    last_event = json.loads(lines[-1])
    assert last_event["subsystem"] == "VISION"
    assert last_event["event_type"] == "FIRE_SECTOR_IDENTIFIED"
    assert last_event["level"] == "WARNING"
    assert last_event["data"]["temperature_c"] == 87.5
    assert "timestamp" in last_event
    assert "epoch_s" in last_event


def test_crash_handler_generation(clean_logging_state):
    """Test that install_crash_handler generates a crash file with traceback and locals."""
    log_dir = clean_logging_state
    orig_excepthook = sys.excepthook

    try:
        handler = install_crash_handler(subsystem="PILOT", chain=False)
        assert sys.excepthook == handler

        # Artificially trigger an exception in a nested function with local variables
        def faulty_function():
            drone_altitude = 25.4
            mission_target = "WAYPOINT_DELTA"
            battery_volts = 22.2
            raise ZeroDivisionError("Simulated critical mathematical error: division by zero")

        try:
            faulty_function()
        except ZeroDivisionError:
            exc_type, exc_val, exc_tb = sys.exc_info()
            sys.excepthook(exc_type, exc_val, exc_tb)

        crashes_dir = log_dir / "crashes"
        assert crashes_dir.exists()

        crash_files = list(crashes_dir.glob("crash_pilot_*.log"))
        assert len(crash_files) == 1

        crash_text = crash_files[0].read_text(encoding="utf-8")
        assert "OKO MCHS EMERGENCY COMPLEX - CRASH REPORT [PILOT]" in crash_text
        assert "ZeroDivisionError: Simulated critical mathematical error" in crash_text
        assert "drone_altitude = 25.4" in crash_text
        assert "mission_target = 'WAYPOINT_DELTA'" in crash_text
        assert "battery_volts = 22.2" in crash_text
        assert "TRACEBACK:" in crash_text

    finally:
        sys.excepthook = orig_excepthook


def test_thread_safety_logging(clean_logging_state):
    """Stress test logging from multiple concurrent threads."""
    log_dir = clean_logging_state
    logger = get_logger("stress", subsystem="CORE")

    num_threads = 8
    msgs_per_thread = 50

    def worker(tid: int):
        for i in range(msgs_per_thread):
            logger.info(f"Thread {tid} message {i}")
            log_event(
                subsystem="CORE",
                event_type="THREAD_TICK",
                data={"tid": tid, "tick": i},
            )

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(num_threads)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    for h in logger.handlers:
        h.flush()

    events_file = log_dir / "events.jsonl"
    lines = events_file.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == num_threads * msgs_per_thread

    # Ensure all lines are valid JSON
    for line in lines:
        parsed = json.loads(line)
        assert parsed["subsystem"] == "CORE"
        assert parsed["event_type"] == "THREAD_TICK"
