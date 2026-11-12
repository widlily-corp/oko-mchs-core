"""Failure injection for configuration persistence; never touch the real arena."""
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest
import app as ui


def test_failed_replace_keeps_existing_hazards(tmp_path, monkeypatch):
    path = tmp_path / "hazards.json"
    original = '{"FIRE": {"x": 1, "y": 2}, "vendor": "keep"}'
    path.write_text(original)
    def locked(*args):
        raise PermissionError("Windows reader holds destination")
    monkeypatch.setattr(ui.os, "replace", locked)
    with pytest.raises(OSError):
        ui.save_hazards({"FIRE": {"x": 9, "y": 8}}, path)
    assert path.read_text() == original
    assert list(tmp_path.iterdir()) == [path]


def test_save_preserves_unknown_fields(tmp_path):
    path = tmp_path / "hazards.json"
    original = {"_comment": "user", "schema_extra": [1, 2],
                "FIRE": {"x": 1, "y": 2, "vendor": {"tag": "keep"}},
                "CUSTOM": {"x": 3, "y": 4}}
    path.write_text(json.dumps(original))
    ui.save_hazards({"FIRE": {"x": 9, "y": 8}}, path)
    result = json.loads(path.read_text())
    assert result["schema_extra"] == [1, 2]
    assert result["FIRE"]["vendor"] == {"tag": "keep"}
    assert result["CUSTOM"] == original["CUSTOM"]
    assert result["FIRE"]["x"] == 9


def test_concurrent_writers_publish_complete_json(tmp_path):
    path = tmp_path / "hazards.json"
    path.write_text('{"FIRE": {"x": 0, "y": 0}}')
    def write(i):
        ui.save_hazards({"FIRE": {"x": i, "y": i, "padding": "a" * 20000}}, path)
        data = json.loads(path.read_text())
        assert data["FIRE"]["x"] == data["FIRE"]["y"]
    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(write, range(80)))
    assert list(tmp_path.iterdir()) == [path]
