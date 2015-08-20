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

# Entrypiont is needed to preserve exit code
docker run \
    -v ${CCACHE_DIR:-${HOME}/.ccache}:/home/developer/.ccache \
    -v ${ROBOCUP_ROOT}:/home/developer/robocup-software \
    -v ${CIRCLE_ARTIFACTS:-/tmp/}:/tmp/build_artifacts \
    -e CIRCLE_BUILD_NUM=${CIRCLE_BUILD_NUM} \
    -e CIRCLE_ARTIFACTS=${CIRCLE_ARTIFACTS} \
    --entrypoint /bin/bash \
    ${INPUT_IMAGE_NAME}:${SHA_SUM_SETUP} /home/developer/robocup-software/util/docker/maketest.sh ${GH_TOKEN}

EXIT=$?
if [ $EXIT -ne 0 ]; then
    exit $EXIT
fi

# Commit the latest image
docker commit "$(docker ps -aq | head -n1)" ${IMAGE_NAME}:${SHA_SUM}
