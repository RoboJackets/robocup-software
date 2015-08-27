#!/bin/bash
# Builds our circle image which runs tests
set -e

DIR=$(cd $(dirname $0) ; pwd -P)
source ${DIR}/../docker_common.sh

if [ "$GH_STATUS_TOKEN" = "" ]; then
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
    -e GH_USER=${GH_USER} \
    --entrypoint /bin/bash \
    ${IMAGE_NAME_BASE}:${SHA_SUM_SETUP} /home/developer/robocup-software/util/docker/maketest.sh ${GH_STATUS_TOKEN}

EXIT=$?
if [ $EXIT -ne 0 ]; then
    exit $EXIT
fi

# Commit the latest image
docker commit "$(docker ps -aq | head -n1)" ${IMAGE_NAME_CI}:${SHA_SUM}
