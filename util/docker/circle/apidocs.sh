#!/bin/bash
# Runs the autoupdate script for api documentation
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

# Entrypiont is needed to preserve exit code
docker run \
    -v ${CCACHE_DIR:-${HOME}/.ccache}:/home/developer/.ccache \
    -v ${ROBOCUP_ROOT}:/home/developer/robocup-software \
    -v ${CIRCLE_ARTIFACTS:-/tmp/}:/tmp/build_artifacts \
    -e CIRCLE_BUILD_NUM=${CIRCLE_BUILD_NUM} \
    -e CIRCLE_ARTIFACTS=${CIRCLE_ARTIFACTS} \
    -e GIT_USERNAME=${GIT_USERNAME} \
    -e GIT_EMAIL=${GIT_EMAIL} \
    -e GH_TOKEN=${GH_TOKEN} \
    --entrypoint /bin/bash \
    ${INPUT_IMAGE_NAME}:${SHA_SUM_SETUP} /home/developer/robocup-software/autoupdate-docs-travis.sh
