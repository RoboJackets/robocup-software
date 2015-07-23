#!/bin/bash
# Builds our baseimage if needed


DIR=$(dirname "${BASH_SOURCE}")
ROBOCUP_ROOT="${DIR}/../../../"

# Get sha sum
SHA_SUM="$(${ROBOCUP_ROOT}/util/docker/getsetupsha.sh)"
IMAGE_NAME="jgkamat/robocup-baseimage"

if docker pull ${IMAGE_NAME}:${SHA_SUM}; then
    docker tag ${IMAGE_NAME}:${SHA_SUM} ${IMAGE_NAME}:latest
else
    sudo docker build -t ${IMAGE_NAME}:${SHA_SUM}  -f util/docker/baseimage/Dockerfile ${ROBOCUP_ROOT}
    docker push ${IMAGE_NAME}:${SHA_SUM}
    docker tag ${IMAGE_NAME}:${SHA_SUM} ${IMAGE_NAME}:latest
fi

