#!/usr/bin/env bash
source /opt/ros/humble/setup.bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
REPO_DIR=$(dirname "$SCRIPT_DIR")
export GAZEBO_MODEL_PATH="$REPO_DIR/src/mchs_simulation/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
gzserver --verbose "$REPO_DIR/src/mchs_simulation/worlds/mchs_emergency_arena.world" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &
GZ_PID=$!
sleep 5
ros2 topic info -v /mchs/camera/image_raw
kill -9 $GZ_PID 2>/dev/null || true
