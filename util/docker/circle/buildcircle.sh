#!/bin/bash
# Builds our circle image which runs tests


DIR=$(dirname "${BASH_SOURCE}")
ROBOCUP_ROOT="${DIR}/../../../"

# Get sha sum
SHA_SUM_SETUP="$(${ROBOCUP_ROOT}/util/docker/getsetupsha.sh)"
SHA_SUM="$(git rev-parse HEAD)"
IMAGE_NAME="jgkamat/robocup-software"

sudo docker build -t ${IMAGE_NAME}:${SHA_SUM}  -f util/docker/circle/Dockerfile ${ROBOCUP_ROOT}
docker push ${IMAGE_NAME}:${SHA_SUM}
