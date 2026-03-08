#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

docker buildx build \
    --memory 8g \
    --platform linux/aarch64 \
    --build-arg TARGETPLATFORM=linux/aarch64 \
    -t rj_base_station \-
    -f ${SCRIPT_DIR}/Dockerfile.base_station \
    --output type=tar,dest=${SCRIPT_DIR}/base_station.tar \
    ${SCRIPT_DIR}/../..
