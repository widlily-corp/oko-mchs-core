"""Перемещение очага через arena_controller обновляет config/hazards.json.

Это единый источник истины: без записи назад перемещённый человек остаётся
«не там» для фото-улик и протокола ЦУКС.
"""

import importlib.util
import json
import os

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPT = os.path.join(REPO_ROOT, "src", "mchs_simulation", "scripts",
                      "arena_controller.py")


def _load_module():
    spec = importlib.util.spec_from_file_location("arena_controller", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_config(path):
    data = {
        "HUMAN": {"x": 15.0, "y": -22.0, "z": 0.0, "threat": "CRITICAL",
                  "model": "person_standing"},
        "FIRE": {"x": -62.2, "y": 68.5, "z": 0.2, "threat": "CRITICAL",
                 "model": "fire_hazard"},
    }
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)


def test_move_human_updates_config(tmp_path, monkeypatch):
    module = _load_module()
    config_path = tmp_path / "hazards.json"
    _write_config(str(config_path))
    monkeypatch.setattr(module, "HAZARDS_CONFIG_PATH", str(config_path))

    assert module.update_hazards_config("human", 20.0, 10.0, 0.0) is True

    with open(str(config_path), "r", encoding="utf-8") as handle:
        data = json.load(handle)
    assert data["HUMAN"]["x"] == 20.0
    assert data["HUMAN"]["y"] == 10.0
    # Остальные поля очага и соседние записи не теряются.
    assert data["HUMAN"]["threat"] == "CRITICAL"
    assert data["HUMAN"]["model"] == "person_standing"
    assert data["FIRE"]["x"] == -62.2


def test_victim_alias_and_unknown_key(tmp_path, monkeypatch):
    module = _load_module()
    config_path = tmp_path / "hazards.json"
    _write_config(str(config_path))
    monkeypatch.setattr(module, "HAZARDS_CONFIG_PATH", str(config_path))

    assert module.update_hazards_config("victim", -5.0, -7.5, 0.0) is True
    with open(str(config_path), "r", encoding="utf-8") as handle:
        assert json.load(handle)["HUMAN"]["x"] == -5.0

    assert module.update_hazards_config("nonsense", 0.0, 0.0, 0.0) is False
