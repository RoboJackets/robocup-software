#!/bin/bash
# Builds our circle image which runs tests
set -e

DIR=$(cd $(dirname $0) ; pwd -P)
ROBOCUP_ROOT="${DIR}/../../../"

# Get sha sum
SHA_SUM_SETUP="$(${ROBOCUP_ROOT}/util/docker/getsetupsha.sh)"
SHA_SUM="$(git rev-parse HEAD)"
IMAGE_NAME="robojackets/robocup-ci"

docker build -t ${IMAGE_NAME}:${SHA_SUM}  -f ${ROBOCUP_ROOT}/util/docker/circle/Dockerfile ${ROBOCUP_ROOT}
