#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

docker build -t robocup-dev -f ${SCRIPT_DIR}/Dockerfile.dev ${SCRIPT_DIR}/../
