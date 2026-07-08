#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

xhost +
docker run -d -it \
    --name robocup-dev \
    -v ${SCRIPT_DIR}/..:/ws \
    -v /dev/shm:/dev/shm \
    -v /tmp:/tmp \
    --net=host \
    -e "DISPLAY=$DISPLAY" \
    robocup-dev \
    /bin/bash
