#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source install/setup.bash
exec ros2 launch rj_base_station "$@"