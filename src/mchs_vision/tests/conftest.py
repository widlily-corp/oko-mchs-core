"""Общие настройки pytest: добавить корень пакета в sys.path."""

import pathlib
import sys

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
