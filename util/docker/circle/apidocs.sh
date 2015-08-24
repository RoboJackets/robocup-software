#!/bin/bash
# Runs the autoupdate script for api documentation
set -e

DIR=$(cd $(dirname $0) ; pwd -P)
source ${DIR}/../docker_common.sh

if [ "$GH_TOKEN" = "" ]; then
    echo "Github token not set!"
    exit 1
fi

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
    ${IMAGE_NAME_BASE}:${SHA_SUM_SETUP} /home/developer/robocup-software/autoupdate-docs.sh
