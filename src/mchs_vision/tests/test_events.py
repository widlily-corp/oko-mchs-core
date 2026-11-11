"""Тесты реестра инцидентов и ограничителя публикаций."""

from datetime import datetime, timedelta, timezone

from mchs_vision.events import EventThrottle, IncidentRegistry, sort_records


def make_event(incident_id="INC-2026-001", area=10.0, threat="ADVISORY"):
    return {"incident_id": incident_id, "area_sq_m": area,
            "threat_level": threat}


def test_incident_registry_reuses_nearby_incident():
    registry = IncidentRegistry(radius_m=15.0)
    now = datetime.now(timezone.utc)
    first = registry.incident_for("FIRE", (10.0, 10.0), now)
    second = registry.incident_for("FIRE", (15.0, 12.0), now + timedelta(seconds=10))
    assert first == second


def test_incident_registry_creates_new_for_far_or_other_type():
    registry = IncidentRegistry(radius_m=15.0)
    now = datetime.now(timezone.utc)
    first = registry.incident_for("FIRE", (0.0, 0.0), now)
    far = registry.incident_for("FIRE", (100.0, 100.0), now)
    other = registry.incident_for("OIL_SPILL", (0.0, 0.0), now)
    assert len({first, far, other}) == 3


def test_throttle_min_interval():
    throttle = EventThrottle(cooldown_s=3.0, min_interval_s=1.0)
    start = datetime.now(timezone.utc)
    assert throttle.allow(make_event(), start) is True
    assert throttle.allow(make_event(), start + timedelta(seconds=0.5)) is False
    assert throttle.allow(make_event(), start + timedelta(seconds=1.2)) is False
    assert throttle.allow(make_event(area=14.0), start + timedelta(seconds=1.2)) is True


def test_throttle_always_allows_threat_change():
    throttle = EventThrottle(cooldown_s=10.0, min_interval_s=1.0)
    start = datetime.now(timezone.utc)
    assert throttle.allow(make_event(area=10.0, threat="ADVISORY"), start)
    assert throttle.allow(make_event(area=10.0, threat="WARNING"),
                          start + timedelta(seconds=1.5)) is True


def test_throttle_isolated_by_incident():
    throttle = EventThrottle(cooldown_s=10.0, min_interval_s=1.0)
    start = datetime.now(timezone.utc)
    assert throttle.allow(make_event("INC-1"), start)
    assert throttle.allow(make_event("INC-2"), start) is True


def test_sort_records_prefers_critical():
    class Record:
        def __init__(self, threat, area):
            self.threat_level = threat
            self.smoothed_area_m2 = area

    records = [Record("ADVISORY", 100.0), Record("CRITICAL", 10.0),
               Record("WARNING", 50.0)]
    ordered = sort_records(records)
    assert [record.threat_level for record in ordered] == [
        "CRITICAL", "WARNING", "ADVISORY"
    ]
