#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Тесты слоя данных АРМ — без графического интерфейса.

Проверяется то, что легко сломать и трудно заметить глазами:

* разбор протокола в обоих форматах (вложенный от автопилота и плоский от АРМ);
* классификация уровня угрозы по площади;
* печатный бланк дежурного;
* отслеживание появления нового файла протокола.

Запуск (нужен tkinter, окно не открывается)::

    python3 src/mchs_ui/tests/test_app_data.py
"""

import json
import os
import sys
import tempfile

UI_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if UI_DIR not in sys.path:
    sys.path.insert(0, UI_DIR)

try:
    import app as ui
except Exception as error:  # pragma: no cover - на машине без GUI-библиотек
    ui = None
    IMPORT_ERROR = error
else:
    IMPORT_ERROR = None


def _skip_if_no_gui() -> bool:
    """Возвращает True, если GUI-библиотеки недоступны."""
    if ui is None:
        print(f"    (пропуск: {type(IMPORT_ERROR).__name__}: {IMPORT_ERROR})")
        return True
    return False


def test_normalize_nested_protocol():
    """Формат автопилота: mchs_incident_report + вложенная телеметрия."""
    if _skip_if_no_gui():
        return
    raw = {
        "mchs_incident_report": {
            "incident_id": "INC-2026-007",
            "hazard_type": "ПОЖАР",
            "hazard_code": "FIRE",
            "threat_level": "CRITICAL",
            "estimated_area_sq_m": 64.2,
            "target_geo": {"abs_x": 42.0, "abs_y": 28.0, "latitude": 53.42, "longitude": 58.99},
            "drone_telemetry": {"altitude_m": 8.0, "battery_percent": 91.0},
            "recommended_response": "Рекомендован наряд: 2 АЦ-40",
        }
    }
    data = ui.normalize_protocol(raw)

    assert data["incident_id"] == "INC-2026-007"
    assert data["hazard_type"] == "ПОЖАР"
    assert data["threat_level"] == "CRITICAL"
    assert abs(data["area_sq_m"] - 64.2) < 1e-6
    assert data["x"] == 42.0 and data["y"] == 28.0
    assert data["altitude"] == 8.0
    assert "2 АЦ-40" in data["recommended_response"]


def test_normalize_flat_protocol():
    """Плоский формат (ручной экспорт АРМ) тоже должен читаться."""
    if _skip_if_no_gui():
        return
    raw = {
        "incident_id": "INC-2026-1408",
        "hazard_type": "ОЧАГ ЛЕСНОГО ПОЖАРА",
        "threat_level": "WARNING",
        "estimated_area_sq_m": 30.0,
        "target_geo": {"abs_x": 12.0, "abs_y": 7.0},
    }
    data = ui.normalize_protocol(raw)

    assert data["incident_id"] == "INC-2026-1408"
    assert data["threat_level"] == "WARNING"
    assert data["x"] == 12.0 and data["y"] == 7.0
    assert data["latitude"] is None


def test_classify_threat_boundaries():
    """Пороги совпадают с модулем зрения: 15 и 50 м²."""
    if _skip_if_no_gui():
        return
    assert ui.classify_threat(0.0) == "ADVISORY"
    assert ui.classify_threat(14.9) == "ADVISORY"
    assert ui.classify_threat(15.0) == "WARNING"
    assert ui.classify_threat(49.9) == "WARNING"
    assert ui.classify_threat(50.0) == "CRITICAL"


def test_render_blank_contains_required_fields():
    """Печатный бланк содержит номер, тип ЧС и подпись дежурного."""
    if _skip_if_no_gui():
        return
    data = {
        "incident_id": "INC-2026-1408",
        "station": "ЦУКС ГУ МЧС России",
        "timestamp_msk": "11.11.2026 14:08",
        "hazard_type": "ОЧАГ ЛЕСНОГО ПОЖАРА",
        "threat_level": "CRITICAL",
        "estimated_area_sq_m": 64.2,
        "target_geo": {"abs_x": 42.0, "abs_y": 28.0},
        "drone_telemetry": {"drone_id": "MCHS-UAV-04", "altitude_m": 8.0, "battery_percent": 91},
        "recommended_response": "Рекомендован наряд: 2 АЦ-40",
        "recommended_actions": ["Передать координаты в ЦУКС"],
    }
    text = ui.OkoMchsApp.render_blank(data)

    assert "INC-2026-1408" in text
    assert "ОЧАГ ЛЕСНОГО ПОЖАРА" in text
    assert "64.2" in text
    assert "Оперативный дежурный ЦУКС" in text


def test_watcher_detects_new_protocol_file():
    """Watcher замечает новый файл один раз и не повторяет тревогу."""
    if _skip_if_no_gui():
        return
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "incident_protocol.json")
        watcher = ui.ProtocolWatcher([path])

        assert watcher.poll() is None, "пустой watcher не должен ничего находить"

        with open(path, "w", encoding="utf-8") as handle:
            json.dump({"incident_id": "INC-2026-001", "threat_level": "CRITICAL"}, handle)

        first = watcher.poll()
        assert first is not None and first["incident_id"] == "INC-2026-001"
        assert watcher.poll() is None, "повторный опрос не должен поднимать тревогу снова"


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
