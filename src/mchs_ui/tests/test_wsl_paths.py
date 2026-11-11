"""Тесты преобразования путей Windows ↔ WSL для запуска скриптов из АРМ."""

import os
import sys

UI_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if UI_DIR not in sys.path:
    sys.path.insert(0, UI_DIR)

from wsl_paths import repo_wsl_path, to_wsl_path  # noqa: E402


def test_unc_wsl_localhost():
    path = "\\\\wsl.localhost\\Ubuntu-22.04\\root\\Rep\\oko-mchs-core"
    assert to_wsl_path(path) == "/root/Rep/oko-mchs-core"


def test_unc_wsl_dollar_and_drive():
    assert to_wsl_path("\\\\wsl$\\Ubuntu-22.04\\root\\repo") == "/root/repo"
    assert to_wsl_path("C:\\Users\\USER\\Downloads") == "/mnt/c/Users/USER/Downloads"


def test_posix_and_short_unc():
    assert to_wsl_path("/root/Rep/oko-mchs-core") == "/root/Rep/oko-mchs-core"
    assert to_wsl_path("\\\\wsl.localhost\\Ubuntu-22.04") == "/"


def test_repo_wsl_path_is_absolute_wsl():
    """Корень репозитория из этого файла — всегда абсолютный путь WSL."""
    path = repo_wsl_path()
    assert path.startswith("/")
    assert "oko-mchs-core" in path
    assert "\\" not in path
