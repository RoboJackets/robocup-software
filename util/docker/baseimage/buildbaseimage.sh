#!/bin/bash
# Builds our baseimage if needed
set -e

DIR=$(cd $(dirname $0) ; pwd -P)
source ${DIR}/../docker_common.sh

if curl -s https://registry.hub.docker.com/v1/repositories/${IMAGE_NAME_BASE}/tags |  fgrep -q "\"name\": \"${SHA_SUM_SETUP}\""; then
    # The tag currently exists, and can be pulled
    docker pull ${IMAGE_NAME_BASE}:${SHA_SUM_SETUP}
else
    # The tag does not exist, let's build it!
    docker build -t ${IMAGE_NAME_BASE}:in_progress  -f ${ROBOCUP_ROOT}/util/docker/baseimage/Dockerfile .
    docker run \
        -v ${ROBOCUP_ROOT}:/home/developer/robocup-software ${IMAGE_NAME_BASE}:in_progress \
        sh -c './util/ubuntu-setup --yes --firmware && sudo apt-get clean'

    docker commit "$(docker ps -aq | head -n1)" ${IMAGE_NAME_BASE}:${SHA_SUM_SETUP}
    docker push ${IMAGE_NAME_BASE}:${SHA_SUM_SETUP}
fi

if [ "$(git rev-parse --abbrev-ref HEAD)" = "master" ]; then
    git tag ${IMAGE_NAME_BASE}:${SHA_SUM_SETUP} ${IMAGE_NAME_BASE}:master
    git push ${IMAGE_NAME_BASE}:master
fi
