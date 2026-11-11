@echo off
pushd "%~dp0"
echo Starting MCHS Operator UI on Windows...
python "%~dp0src\mchs_ui\app.py"
popd
pause
