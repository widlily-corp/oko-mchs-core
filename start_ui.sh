#!/usr/bin/env bash
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
cd "$SCRIPT_DIR"
python.exe "$SCRIPT_DIR/src/mchs_ui/app.py" 2>/dev/null || python3 "$SCRIPT_DIR/src/mchs_ui/app.py"
