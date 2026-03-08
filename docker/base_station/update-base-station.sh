#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Build a new image
${SCRIPT_DIR}/docker-build.sh

# Copy the new image to the raspberry pi
scp ./base_station.tar robojackets@10.42.0.248:/home/pi/base_station.tar

# Stop the current base station code and run the new one
sshpass -p robojackets ssh \
    -o UserKnownHostsfile=/dev/null \
    -o StrictHostKeyChecking=no \
    robojackets@10.42.0.248 \
<<EOF
    docker stop base-station
    docker rm base-station
    docker image load -i base_station.tar
    docker run -d --privileged --net=host --name base-station rj_base_station base_station.launch.py
EOF