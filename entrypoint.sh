#!/usr/bin/env bash
# Source ROS and the workspace holding xgo_localization (plus, in the robot
# image, the XGO and LiDAR drivers), then hand off.
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

exec "$@"
