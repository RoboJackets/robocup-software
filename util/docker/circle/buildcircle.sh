#!/bin/bash
# Builds our circle image which runs tests
set -e

if [ "$GH_TOKEN" = "" ]; then
    echo "Github token not set!"
    exit 1
fi

DIR=$(cd $(dirname $0) ; pwd -P)
ROBOCUP_ROOT="${DIR}/../../../"

# Get sha sum
SHA_SUM_SETUP="$(${ROBOCUP_ROOT}/util/docker/getsetupsha.sh)"
SHA_SUM="$(git rev-parse HEAD)"
IMAGE_NAME="robojackets/robocup-ci"

docker build -t tmp_ci_image -f ${ROBOCUP_ROOT}/util/docker/circle/Dockerfile ${ROBOCUP_ROOT}
BUILT_IMAGE="tmp_ci_image"
DOCKER_IMAGE="$(docker run $BUILT_IMAGE ./util/docker/maketest.sh ${GH_TOKEN})"
docker commit $DOCKER_IMAGE ${IMAGE_NAME}:${SHA_SUM}
