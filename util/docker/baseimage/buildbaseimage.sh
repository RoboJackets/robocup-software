#!/bin/bash
# Builds our baseimage if needed
set -e

DIR=$(cd $(dirname $0) ; pwd -P)
ROBOCUP_ROOT="${DIR}/../../../"

# Get sha sum
SHA_SUM="$(${ROBOCUP_ROOT}/util/docker/getsetupsha.sh)"
IMAGE_NAME="jgkamat/robocup-baseimage"

if curl -s https://registry.hub.docker.com/v1/repositories/${IMAGE_NAME}/tags |  fgrep -q "\"name\": \"${SHA_SUM}\""; then
    # The tag currently exists, and can be pulled
    docker pull ${IMAGE_NAME}:${SHA_SUM}
    docker tag ${IMAGE_NAME}:${SHA_SUM} ${IMAGE_NAME}:latest
else
    # The tag does not exist, let's build it!
    docker build -t ${IMAGE_NAME}:${SHA_SUM}  -f ${ROBOCUP_ROOT}/util/docker/baseimage/Dockerfile ${ROBOCUP_ROOT}
    docker push ${IMAGE_NAME}:${SHA_SUM}
    docker tag ${IMAGE_NAME}:${SHA_SUM} ${IMAGE_NAME}:latest
fi

