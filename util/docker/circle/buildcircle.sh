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
INPUT_IMAGE_NAME="robojackets/robocup-baseimage"
IMAGE_NAME="robojackets/robocup-ci"

# Lets trick the dockerfile into using the image we want.
docker tag -f ${INPUT_IMAGE_NAME}:${SHA_SUM_SETUP} ${INPUT_IMAGE_NAME}:latest_build
docker push ${INPUT_IMAGE_NAME}:latest_build

docker build -t tmp_ci_image -f ${ROBOCUP_ROOT}/util/docker/circle/Dockerfile ${ROBOCUP_ROOT}
BUILT_IMAGE="tmp_ci_image"
docker run ${BUILT_IMAGE} ./util/docker/maketest.sh ${GH_TOKEN}
# Commit the latest image
docker commit "$(docker ps -aq | head -n1)" ${IMAGE_NAME}:${SHA_SUM}
