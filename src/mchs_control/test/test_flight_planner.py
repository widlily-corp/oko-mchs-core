#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Автотесты автопилота «Око МЧС» (FSM + протокол инцидента).

Проверяется то, что можно проверить без Gazebo:

* геометрия маршрута «змейка» и времена разворотов;
* порядок фаз FSM и возврат в точку старта;
* реакция на события ЧС: инспекция только в патруле, только WARNING/CRITICAL,
  одно зависание на один очаг;
* аварийный возврат при низком заряде АКБ;
* разбор JSON-события ``/mchs/detection_event``;
* формирование ``incident_protocol.json`` и печатного бланка ``.txt``.

Запуск:
    python3 -m pytest src/mchs_control/test -v
    python3 src/mchs_control/test/test_flight_planner.py
"""

import json
import math
import os
import sys
import tempfile

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from generate_report import (  # noqa: E402
    build_incident_protocol,
    classify_threat,
    generate_incident_protocol,
    render_protocol_text,
)
from mchs_pilot_node import Detection, FlightPlanner, Phase  # noqa: E402

DT = 0.1
EPS = 1e-6


def run_mission(planner: FlightPlanner, detection: Detection | None = None, detect_at=None, limit=600.0):
    """Прогоняет миссию до конца (или до лимита) и собирает события FSM."""
    events = []
    sent = False
    while not planner.is_finished and planner.elapsed < limit:
        current = None
        if detection is not None and detect_at is not None and not sent and planner.elapsed >= detect_at:
            current = detection
            sent = True
        elif detection is not None and detect_at is None:
            current = detection
        _, tick_events = planner.tick(DT, detection=current)
        events.extend(tick_events)
    return events


def phases(planner: FlightPlanner) -> list[str]:
    """Список кодов фаз из журнала миссии."""
    return [entry["phase"] for entry in planner.phase_log]


# ---------------------------------------------------------------------------
# Маршрут и FSM
# ---------------------------------------------------------------------------


def test_patrol_route_geometry():
    """Маршрут на 2 галса: 5 участков, галс 30 м, смещение 5 м."""
    planner = FlightPlanner(lanes=2)
    route = planner.patrol_route

    assert len(route) == 5, f"ожидалось 5 участков, получено {len(route)}"

    lane_1, turn_1, shift, turn_2, lane_2 = route
    assert lane_1.title.startswith("Галс 1")
    assert lane_2.title.startswith("Галс 2")
    assert abs(lane_1.duration - 15.0) < EPS
    assert abs(lane_1.vx - 2.0) < EPS
    assert abs(lane_1.vx * lane_1.duration - 30.0) < EPS, "галс должен покрывать 30 м"

    expected_turn = (math.pi / 2.0) / 0.6
    assert abs(turn_1.duration - expected_turn) < 1e-3
    assert abs(abs(turn_1.wz) - 0.6) < EPS
    assert abs(turn_1.wz - turn_2.wz) < EPS, "два разворота в одну сторону = 180°"

    assert abs(shift.vx * shift.duration - 5.0) < EPS, "смещение должно быть ровно 5 м"


def test_takeoff_reaches_working_altitude():
    """TAKEOFF набирает 8 м и переводит автомат в PATROL_GRID."""
    planner = FlightPlanner(lanes=1)
    events = []
    for _ in range(200):  # 20 с — заведомо больше 8 с подъёма
        _, tick_events = planner.tick(DT)
        events.extend(tick_events)
        if planner.phase is not Phase.TAKEOFF:
            break

    assert "takeoff_complete" in events
    assert planner.phase is Phase.PATROL_GRID
    assert abs(planner.z - 8.0) < 0.15, planner.z


def test_full_mission_returns_to_launch_point():
    """Полная миссия: фазы по порядку, посадка в нуле и рядом с точкой старта."""
    planner = FlightPlanner(lanes=2)
    events = run_mission(planner)

    assert phases(planner) == [
        Phase.TAKEOFF.value,
        Phase.PATROL_GRID.value,
        Phase.RTL.value,
        Phase.LAND.value,
        Phase.FINISHED.value,
    ], phases(planner)

    for event in ("takeoff_complete", "patrol_complete", "rtl_complete", "landed"):
        assert event in events, f"не получено событие {event}"

    assert planner.is_finished
    assert planner.z <= 0.2, f"посадка не выполнена: Z={planner.z}"
    assert math.hypot(planner.x, planner.y) < 1.5, "дрон сел далеко от точки старта"
    assert planner.distance_travelled > 60.0


# ---------------------------------------------------------------------------
# Реакция на события ЧС
# ---------------------------------------------------------------------------


def test_critical_detection_triggers_long_inspection():
    """CRITICAL: зависание 10 с и ровно одна инспекция."""
    planner = FlightPlanner(lanes=2)
    detection = Detection(hazard_type="FIRE", threat_level="CRITICAL", area_sq_m=120.0)
    events = run_mission(planner, detection=detection, detect_at=8.0)

    assert "inspection_started" in events
    assert planner.inspections == 1
    entries = phases(planner)
    assert entries.index(Phase.HOVER_INSPECTION.value) < entries.index(Phase.RTL.value)

    hover_start = next(e["t"] for e in planner.phase_log if e["phase"] == Phase.HOVER_INSPECTION.value)
    hover_end = next(e["t"] for e in planner.phase_log if e["phase"] == Phase.PATROL_GRID.value and e["t"] > hover_start)
    assert 9.5 <= (hover_end - hover_start) <= 11.0, hover_end - hover_start
    assert planner.is_finished


def test_warning_detection_hovers_five_seconds():
    """WARNING: короткое зависание (5 с), как требует ТЗ."""
    planner = FlightPlanner(lanes=1)
    detection = Detection(hazard_type="SMOKE", threat_level="WARNING", area_sq_m=30.0)
    run_mission(planner, detection=detection, detect_at=9.0)

    hover_start = next(e["t"] for e in planner.phase_log if e["phase"] == Phase.HOVER_INSPECTION.value)
    hover_end = next(e["t"] for e in planner.phase_log if e["phase"] == Phase.PATROL_GRID.value and e["t"] > hover_start)
    assert 4.5 <= (hover_end - hover_start) <= 6.0, hover_end - hover_start


def test_advisory_detection_is_ignored():
    """ADVISORY не прерывает патруль: зависания нет, миссия идёт дальше."""
    planner = FlightPlanner(lanes=1)
    detection = Detection(hazard_type="SMOKE", threat_level="ADVISORY", area_sq_m=10.0)
    events = run_mission(planner, detection=detection, detect_at=9.0)

    assert planner.inspections == 0
    assert "inspection_started" not in events
    assert "detection_ignored" in events
    assert planner.is_finished


def test_repeated_detections_do_not_restart_inspection():
    """Поток одинаковых событий не должен превращаться в бесконечное зависание."""
    planner = FlightPlanner(lanes=2, max_inspections=1)
    detection = Detection(hazard_type="FIRE", threat_level="CRITICAL", area_sq_m=80.0)

    while not planner.is_finished and planner.elapsed < 600.0:
        planner.tick(DT, detection=detection)

    assert planner.is_finished
    assert planner.inspections == 1, f"инспекций {planner.inspections}, ожидалась 1"


def test_detection_during_takeoff_does_not_interrupt_climb():
    """Тревога на взлёте не прерывает набор высоты: сначала безопасность."""
    planner = FlightPlanner(lanes=1)
    detection = Detection(hazard_type="FIRE", threat_level="CRITICAL", area_sq_m=80.0)

    for _ in range(50):  # 5 с — ещё идёт взлёт (8 с)
        planner.tick(DT, detection=detection)

    assert planner.phase is Phase.TAKEOFF
    assert planner.inspections == 0

    run_mission(planner, detection=detection, detect_at=None)
    hover_entry = next(e for e in planner.phase_log if e["phase"] == Phase.HOVER_INSPECTION.value)
    assert hover_entry["t"] >= 7.5, "инспекция должна начаться уже в патруле"


def test_low_battery_forces_rtl():
    """Низкий заряд АКБ прерывает патруль и уводит дрон на базу."""
    planner = FlightPlanner(lanes=3, battery_drain_pct_per_s=2.0, battery_rtl_threshold_pct=90.0)
    events = run_mission(planner)

    assert "battery_low" in events
    assert Phase.RTL.value in phases(planner)
    assert planner.is_finished


# ---------------------------------------------------------------------------
# Разбор события
# ---------------------------------------------------------------------------


def test_detection_parsing_from_contract_json():
    """Событие разбирается по контракту зрения и считает угрозу по площади."""
    payload = {
        "hazard_type": "FIRE",
        "threat_level": "CRITICAL",
        "area_sq_m": 48.5,
        "target_geo": {"abs_x": 14.8, "abs_y": -5.2, "rel_x": 4.8, "rel_y": -1.2},
        "confidence": 0.94,
        "timestamp": "2026-11-11T12:00:00",
    }
    detection = Detection.from_payload(json.dumps(payload))

    assert detection is not None
    assert detection.hazard_type == "FIRE"
    assert detection.threat_level == "CRITICAL"
    assert detection.is_alarm
    assert detection.abs_position == (14.8, -5.2)
    assert abs(detection.confidence - 0.94) < EPS


def test_detection_parsing_derives_threat_level():
    """Если уровень не передан, он вычисляется по площади (15 и 50 м²)."""
    small = Detection.from_payload({"hazard_type": "SMOKE", "area_sq_m": 10})
    medium = Detection.from_payload({"hazard_type": "FIRE", "area_sq_m": 30})
    large = Detection.from_payload({"hazard_type": "FIRE", "area_sq_m": 60})

    assert small.threat_level == "ADVISORY"
    assert medium.threat_level == "WARNING"
    assert large.threat_level == "CRITICAL"


def test_detection_parsing_rejects_garbage():
    """Мусор в топике не должен ронять ноду."""
    assert Detection.from_payload("не JSON") is None
    assert Detection.from_payload("[1, 2, 3]") is None


def test_threat_classification_boundaries():
    """Границы уровней угрозы совпадают с контрактом зрения."""
    assert classify_threat(0.0) == "ADVISORY"
    assert classify_threat(14.9) == "ADVISORY"
    assert classify_threat(15.0) == "WARNING"
    assert classify_threat(49.9) == "WARNING"
    assert classify_threat(50.0) == "CRITICAL"


# ---------------------------------------------------------------------------
# Протокол инцидента
# ---------------------------------------------------------------------------


def test_protocol_files_are_created():
    """Создаются оба файла протокола, JSON валиден, бланк содержит наряд."""
    with tempfile.TemporaryDirectory() as tmp:
        protocol = generate_incident_protocol(
            hazard_code="FIRE",
            area_sq_m=64.2,
            target_geo={"abs_x": 16.6, "abs_y": 20.3, "rel_x": 4.8, "rel_y": -1.2},
            confidence=0.94,
            drone_telemetry={"altitude_m": 8.0, "battery_percent": 96.0},
            out_dir=tmp,
            verbose=False,
        )

        json_path = os.path.join(tmp, "incident_protocol.json")
        txt_path = os.path.join(tmp, "incident_protocol.txt")
        assert os.path.exists(json_path), "не создан incident_protocol.json"
        assert os.path.exists(txt_path), "не создан incident_protocol.txt"

        with open(json_path, encoding="utf-8") as handle:
            data = json.load(handle)
        assert data["incident_id"].startswith("INC-")
        assert data["threat_level"] == "CRITICAL"
        assert data["hazard_code"] == "FIRE"
        assert data["estimated_area_sq_m"] == 64.2
        assert "АЦ-40" in data["recommended_response"]
        assert protocol["archive"]["json"] == json_path

        with open(txt_path, encoding="utf-8") as handle:
            text = handle.read()
        assert "ПРОТОКОЛ ИНЦИДЕНТА" in text
        assert data["incident_id"] in text
        assert "АЦ-40" in text


def test_protocol_builder_fields_for_other_hazards():
    """Для разлива и затопления подставляются свои наряды и действия."""
    oil = build_incident_protocol(hazard_code="OIL_SPILL", area_sq_m=70.0)
    flood = build_incident_protocol(hazard_code="FLOOD", area_sq_m=40.0)

    assert oil["hazard_type"] == "РАЗЛИВ НЕФТЕПРОДУКТОВ"
    assert oil["threat_level"] == "CRITICAL"
    assert "скиммер" in oil["recommended_response"]
    assert any("Росприроднадзор" in action for action in oil["recommended_actions"])

    assert flood["hazard_type"] == "ЗАТОПЛЕНИЕ"
    assert flood["threat_level"] == "WARNING"
    assert "спасательный расчёт" in flood["recommended_response"]

    flood_critical = build_incident_protocol(hazard_code="FLOOD", area_sq_m=120.0)
    assert flood_critical["threat_level"] == "CRITICAL"
    assert "плавсредства" in flood_critical["recommended_response"]


def test_protocol_text_contains_signature_line():
    """Печатный бланк заканчивается строкой для подписи дежурного."""
    protocol = build_incident_protocol(hazard_code="FIRE", area_sq_m=20.0)
    text = render_protocol_text(protocol)

    assert "Оперативный дежурный ЦУКС" in text
    assert "Око МЧС" in text
    assert protocol["incident_id"] in text


if __name__ == "__main__":
    failures = 0
    tests = [(name, obj) for name, obj in sorted(globals().items()) if name.startswith("test_") and callable(obj)]
    for name, test in tests:
        try:
            test()
        except AssertionError as error:
            failures += 1
            print(f"[FAIL] {name}: {error}")
        else:
            print(f"[OK]   {name}")
    print(f"\nИтого: {len(tests) - failures} из {len(tests)} тестов прошли.")
    sys.exit(1 if failures else 0)
