#!/bin/bash
# Builds our circle image which runs tests
set -e

DIR=$(cd $(dirname $0) ; pwd -P)
ROBOCUP_ROOT="${DIR}/../../../"

# Get sha sum
SHA_SUM_SETUP="$(${ROBOCUP_ROOT}/util/docker/getsetupsha.sh)"
SHA_SUM="$(git rev-parse HEAD)"
IMAGE_NAME="jgkamat/robocup-ci"

docker push ${IMAGE_NAME}:${SHA_SUM}

if [ "$(git rev-parse --abbrev-ref HEAD)" != "HEAD" ]; then
    docker tag "${IMAGE_NAME}:${SHA_SUM}" "${IMAGE_NAME}:$(git rev-parse --abbrev-ref HEAD)"
    docker push "${IMAGE_NAME}:$(git rev-parse --abbrev-ref HEAD)"
fi

