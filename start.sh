#!/bin/bash

if ! command -v git >/dev/null; then
    echo "Git is not installed!!!" >&2
    exit 1
fi

if [ "$(git rev-parse --abbrev-ref HEAD)" = "HEAD" ]; then
    echo "HEAD IS DETACHED AT COMMIT: $(git rev-parse HEAD)"
else
    echo "Branch is: $(git rev-parse --abbrev-ref HEAD)"
fi

source /opt/ros/foxy/setup.bash
make
source install/setup.bash
ros2 launch rj_robocup soccer.launch.py
