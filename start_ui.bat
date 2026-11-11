@echo off
pushd "%~dp0"
echo Starting MCHS Operator UI on Windows...
python "\\wsl.localhost\Ubuntu-22.04\root\oko-mchs-core\src\mchs_ui\app.py"
popd
pause
