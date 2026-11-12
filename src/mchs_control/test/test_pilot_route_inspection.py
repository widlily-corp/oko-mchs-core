"""Автотесты реакции демо-автопилота ``mchs_pilot_route`` на тревоги зрения.

Проверяется то, что видно жюри на демонстрации: при событии WARNING дрон
зависает на 5 с, при CRITICAL — на 10 с, один очаг инспектируется один раз,
а повторные события не продлевают зависание.
"""

import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_PKG_DIR = os.path.dirname(SCRIPT_DIR)  # src/mchs_control
if CONTROL_PKG_DIR not in sys.path:
    sys.path.insert(0, CONTROL_PKG_DIR)

from mchs_control.route_inspection import (  # noqa: E402
    InspectionController,
    InspectionManeuver,
    link_loss_needs_rtl,
    parse_detection_event,
)


def _event(incident_id="INC-2026-001", threat="CRITICAL"):
    return {
        "incident_id": incident_id,
        "hazard_type": "VICTIM",
        "threat_level": threat,
        "area_sq_m": 0.48,
        "x": 20.0,
        "y": 10.0,
    }


def _payload(incident_id="INC-2026-001", threat="CRITICAL"):
    return json.dumps({
        "incident_id": incident_id,
        "hazard_type": "VICTIM",
        "threat_level": threat,
        "area_sq_m": 0.48,
        "target_geo": {"abs_x": 20.0, "abs_y": 10.0},
    }, ensure_ascii=False)


def test_parse_detection_event_valid():
    event = parse_detection_event(_payload())
    assert event is not None
    assert event["incident_id"] == "INC-2026-001"
    assert event["threat_level"] == "CRITICAL"
    assert event["x"] == 20.0
    assert event["y"] == 10.0


def test_parse_detection_event_rejects_garbage():
    assert parse_detection_event("не json") is None
    assert parse_detection_event("[1, 2, 3]") is None
    assert parse_detection_event('{"incident_id": "X"}') is None


def test_parse_detection_event_ignores_advisory():
    assert parse_detection_event(_payload(threat="ADVISORY")) is None


def test_warning_hovers_five_seconds():
    controller = InspectionController(warning_s=5.0, critical_s=10.0,
                                      cooldown_s=0.0)
    assert controller.on_event(_event(threat="WARNING"), 0.0) is True
    assert controller.is_active(4.9) is True
    assert controller.is_active(5.1) is False


def test_critical_hovers_ten_seconds():
    controller = InspectionController(warning_s=5.0, critical_s=10.0,
                                      cooldown_s=0.0)
    assert controller.on_event(_event(threat="CRITICAL"), 0.0) is True
    assert controller.is_active(9.9) is True
    assert controller.is_active(10.1) is False


def test_repeated_events_do_not_extend_hover():
    controller = InspectionController(warning_s=5.0, critical_s=10.0,
                                      cooldown_s=0.0)
    assert controller.on_event(_event(), 0.0) is True
    assert controller.on_event(_event(), 3.0) is False
    assert controller.is_active(10.1) is False


def test_one_inspection_per_incident():
    controller = InspectionController(warning_s=5.0, critical_s=10.0,
                                      cooldown_s=0.0)
    assert controller.on_event(_event(), 0.0) is True
    controller.is_active(10.1)  # зависание завершилось
    assert controller.on_event(_event(), 20.0) is False
    assert controller.inspections == 1


def test_cooldown_between_incidents():
    controller = InspectionController(warning_s=5.0, critical_s=10.0,
                                      cooldown_s=5.0)
    assert controller.on_event(_event("INC-1"), 0.0) is True
    controller.is_active(10.1)  # конец первой инспекции
    assert controller.on_event(_event("INC-2"), 12.0) is False  # пауза
    assert controller.on_event(_event("INC-2"), 16.0) is True


def test_cancel_stops_hover():
    controller = InspectionController(warning_s=5.0, critical_s=10.0,
                                      cooldown_s=0.0)
    assert controller.on_event(_event(), 0.0) is True
    controller.cancel()
    assert controller.is_active(1.0) is False


def test_maneuver_targets_standoff_point():
    """Точка съёмки — не над очагом, а в 12 м от него (камера 45°)."""
    maneuver = InspectionManeuver(standoff_m=12.0)
    maneuver.start(20.0, 10.0, 0.0, 10.0, hold_s=10.0)
    assert maneuver.target == (8.0, 10.0)
    assert maneuver.phase == "APPROACH"
    assert maneuver.active is True


def test_maneuver_turns_toward_hazard():
    """Очаг справа — доворот по часовой, слева — против."""
    maneuver = InspectionManeuver()
    maneuver.start(0.0, 10.0, 0.0, 0.0, hold_s=5.0)  # очаг на север (+Y)
    cmd = maneuver.command(0.0, 0.0, 0.0, 0.0)
    assert cmd["yaw_rate"] > 0

    maneuver.start(0.0, -10.0, 0.0, 0.0, hold_s=5.0)  # очаг на юг (-Y)
    cmd = maneuver.command(0.0, 0.0, 0.0, 0.0)
    assert cmd["yaw_rate"] < 0


def test_maneuver_approaches_holds_and_finishes():
    maneuver = InspectionManeuver(standoff_m=12.0, arrive_m=2.5,
                                  speed_mps=6.0)
    maneuver.start(20.0, 10.0, 0.0, 10.0, hold_s=10.0)
    cmd = maneuver.command(0.0, 0.0, 10.0, 0.0)
    assert cmd["phase"] == "APPROACH"
    assert cmd["vx"] > 0  # летим на восток, к точке съёмки

    cmd = maneuver.command(1.0, 8.0, 10.0, 0.0)  # прибыли в точку
    assert cmd["phase"] == "HOLD"
    assert cmd["vx"] == 0.0 and cmd["vy"] == 0.0

    assert maneuver.command(10.9, 8.0, 10.0, 0.0)["done"] is False
    assert maneuver.command(11.1, 8.0, 10.0, 0.0)["done"] is True
    assert maneuver.active is False


def test_maneuver_stop_cancels():
    maneuver = InspectionManeuver()
    maneuver.start(5.0, 5.0, 0.0, 0.0, hold_s=10.0)
    maneuver.stop()
    assert maneuver.active is False
    assert maneuver.command(1.0, 0.0, 0.0, 0.0)["active"] is False


def test_link_loss_triggers_rtl_only_after_odom_in_scan():
    """Потеря телеметрии > 15 с в сканировании — автономный RTL."""
    assert link_loss_needs_rtl(True, 16.0, "SCAN") is True
    assert link_loss_needs_rtl(True, 5.0, "SCAN") is False
    assert link_loss_needs_rtl(False, 30.0, "SCAN") is False  # взлёт
    assert link_loss_needs_rtl(True, 30.0, "RTL") is False  # уже возвращаемся
    assert link_loss_needs_rtl(True, 30.0, "OUTBOUND") is False
