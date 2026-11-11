@echo off
pushd "%~dp0"
echo ========================================================
echo       KOMPLEKS OKO MCHS: UNIFIED SYSTEM LAUNCH
echo ========================================================
echo [1/4] Stopping stale processes...
wsl.exe -d Ubuntu-22.04 bash -c "killall -9 gazebo gzserver gzclient python3 2>/dev/null || true"

echo [2/4] Starting Gazebo, Vision and Pilot in WSL...
start "OKO MCHS - SIMULATION & AUTOPILOT (WSL)" wsl.exe -d Ubuntu-22.04 bash -c "bash /root/oko-mchs-core/scripts/run_all.sh"

echo [3/4] Waiting 5 seconds for nodes initialization...
timeout /t 5 /nobreak >nul

echo [4/4] Starting Operator Station on Windows...
start "OKO MCHS - OPERATOR UI" python "%~dp0src\mchs_ui\app.py"

echo ========================================================
echo ALL SYSTEMS ACTIVE!
echo - Gazebo 3D Arena: running in WSLg
echo - Operator UI: running on Windows
echo - Drone Live Camera Stream: http://localhost:8080
echo ========================================================
popd
