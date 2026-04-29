#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

docker buildx build \
    --memory 8g \
    --platform linux/aarch64 \
    -t localhost:5000/rj_base_station:latest \
    --push \
    -f ${SCRIPT_DIR}/Dockerfile.base_station \
    ${SCRIPT_DIR}/../..
