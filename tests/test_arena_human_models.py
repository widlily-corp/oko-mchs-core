"""Тесты интеграции 3D-моделей людей/пострадавших в мир Gazebo и arena_controller."""

import json
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
import pytest


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MODELS_DIR = os.path.join(REPO_ROOT, "src", "mchs_simulation", "models")
PERSON_MODEL_DIR = os.path.join(MODELS_DIR, "person_standing")
GENERATE_WORLD_SCRIPT = os.path.join(REPO_ROOT, "src", "mchs_simulation", "scripts", "generate_arena_world.py")
ARENA_CONTROLLER_SCRIPT = os.path.join(REPO_ROOT, "src", "mchs_simulation", "scripts", "arena_controller.py")
HAZARDS_CONFIG = os.path.join(REPO_ROOT, "config", "hazards.json")


def test_person_model_exists_and_valid_sdf():
    """Проверяет наличие модели person_standing и валидность её SDF/config."""
    assert os.path.isdir(PERSON_MODEL_DIR), f"Каталог модели не найден: {PERSON_MODEL_DIR}"
    config_path = os.path.join(PERSON_MODEL_DIR, "model.config")
    sdf_path = os.path.join(PERSON_MODEL_DIR, "model.sdf")

    assert os.path.isfile(config_path), "model.config отсутствует"
    assert os.path.isfile(sdf_path), "model.sdf отсутствует"

    # Валидация XML
    ET.parse(config_path)
    sdf_tree = ET.parse(sdf_path)
    root = sdf_tree.getroot()
    assert root.tag == "sdf"
    model = root.find("model")
    assert model is not None
    assert "person" in model.attrib.get("name", "").lower()

    # Gazebo 11/ODE не поддерживает capsule и падает с assertion.
    with open(sdf_path, "r", encoding="utf-8") as handle:
        sdf_text = handle.read()
    assert "<capsule" not in sdf_text
    assert "<cylinder" in sdf_text


def test_hazards_config_includes_human():
    """Проверяет, что config/hazards.json содержит цель HUMAN/VICTIM с координатами."""
    with open(HAZARDS_CONFIG, "r", encoding="utf-8") as f:
        data = json.load(f)

    assert "HUMAN" in data or "VICTIM" in data, "HUMAN/VICTIM отсутствует в config/hazards.json"
    key = "HUMAN" if "HUMAN" in data else "VICTIM"
    entry = data[key]
    assert "x" in entry and "y" in entry
    assert entry.get("threat") in ("CRITICAL", "WARNING")
    assert "person" in entry.get("model", "").lower()


def test_generate_world_spawns_human_model():
    """Проверяет, что generate_arena_world.py генерирует SDF мир со спавном модели человека."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        out_world = os.path.join(tmp_dir, "test_arena.world")
        env = os.environ.copy()
        cmd = [sys.executable, GENERATE_WORLD_SCRIPT, "--output", out_world]

        proc = subprocess.run(cmd, env=env, capture_output=True, text=True)
        assert proc.returncode == 0, f"Генератор мира завершился с ошибкой: {proc.stderr}"
        assert os.path.isfile(out_world)

        with open(out_world, "r", encoding="utf-8") as f:
            content = f.read()

        assert "person_standing" in content or "human" in content.lower(), "Модель человека отсутствует в сгенерированном мире Gazebo"


def test_arena_controller_supports_human_hazard():
    """Проверяет, что arena_controller.py поддерживает цель human/victim при вызове с --help."""
    cmd = [sys.executable, ARENA_CONTROLLER_SCRIPT, "--help"]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    assert proc.returncode == 0
    help_text = proc.stdout.lower()
    assert "human" in help_text or "victim" in help_text or "человек" in help_text
