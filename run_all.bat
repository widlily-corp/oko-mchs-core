@echo off
setlocal
pushd "%~dp0"

set "REPO_UNC=%~dp0"
set "REPO_UNC=%REPO_UNC:\=/%"
cd /d "%TEMP%"
for /f "usebackq delims=" %%I in (`wsl.exe -d Ubuntu-22.04 wslpath -u "%REPO_UNC%"`) do set "REPO_WSL=%%I"
if not defined REPO_WSL (
    echo [ERROR] Failed to resolve repository path inside WSL.
    echo         Check that WSL2 + Ubuntu-22.04 is installed and run this script from the repo.
    popd
    exit /b 1
)

echo ========================================================
echo       KOMPLEKS OKO MCHS: UNIFIED SYSTEM LAUNCH
echo ========================================================
echo [1/4] Stopping stale simulation processes...
wsl.exe -d Ubuntu-22.04 bash -c "pkill -9 -x gzserver 2>/dev/null || true; pkill -9 -f '[m]chs_vision_node.py' 2>/dev/null || true; pkill -9 -f '[m]chs_pilot_route.py' 2>/dev/null || true"

echo [2/4] Starting Gazebo, Vision and Pilot in WSL...
start "OKO MCHS - SIMULATION & AUTOPILOT (WSL)" wsl.exe -d Ubuntu-22.04 bash -c "MCHS_NO_UI=1 bash '%REPO_WSL%/scripts/run_all.sh'"

echo [3/4] Waiting 8 seconds for nodes initialization...
timeout /t 8 /nobreak >nul

echo [4/4] Starting Operator Station on Windows...
start "OKO MCHS - OPERATOR UI" python "%~dp0src\mchs_ui\app.py"

echo ========================================================
echo ALL SYSTEMS ACTIVE!
echo - Gazebo 3D Arena: running in WSLg
echo - Operator UI: running on Windows
echo - Drone Live Camera Stream: http://localhost:8080
echo ========================================================
popd
