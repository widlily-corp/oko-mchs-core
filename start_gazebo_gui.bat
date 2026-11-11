@echo off
pushd "%~dp0"
echo Starting Gazebo 11 3D Arena...
wsl.exe -d Ubuntu-22.04 bash -c "killall -9 gazebo gzserver gzclient 2>/dev/null || true"
wsl.exe -d Ubuntu-22.04 /usr/local/bin/gazebo-mchs
popd
pause
