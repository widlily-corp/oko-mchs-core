"""Преобразование путей между Windows и WSL (без GUI-зависимостей).

АРМ запускается на Windows, а скрипты симуляции — внутри WSL. Путь к
репозиторию нельзя хардкодить: он зависит от того, где лежит чекаут.
"""

from __future__ import annotations

import os
import re

#: Префиксы UNC-путей WSL (оба варианта, поддерживаемые Windows).
_WSL_UNC_PREFIXES = ("\\\\wsl.localhost\\", "\\\\wsl$\\")


def to_wsl_path(path) -> str:
    """Преобразует путь Windows/WSL в путь внутри WSL.

    >>> to_wsl_path(r"\\\\wsl.localhost\\Ubuntu-22.04\\root\\repo")
    '/root/repo'
    """
    text = str(path).replace("/", "\\")
    lowered = text.lower()
    for prefix in _WSL_UNC_PREFIXES:
        if lowered.startswith(prefix):
            rest = text[len(prefix):]
            parts = rest.split("\\", 1)
            if len(parts) == 2:
                return "/" + parts[1].replace("\\", "/")
            return "/"
    match = re.match(r"^([A-Za-z]):\\", text)
    if match:
        return "/mnt/%s/%s" % (match.group(1).lower(),
                               text[3:].replace("\\", "/"))
    return text.replace("\\", "/")


def repo_wsl_path(start: str | None = None) -> str:
    """Путь корня репозитория внутри WSL (для wsl.exe из АРМ)."""
    here = os.path.dirname(os.path.abspath(start or __file__))
    # .../src/mchs_ui/<этот файл> → корень репозитория на два уровня выше src
    repo_root = os.path.dirname(os.path.dirname(here))
    return to_wsl_path(repo_root)
