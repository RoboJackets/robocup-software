#!/bin/bash
# Builds our baseimage if needed
set -e

DIR=$(cd $(dirname $0) ; pwd -P)
ROBOCUP_ROOT="${DIR}/../../../"

# Get sha sum
SHA_SUM="$(${ROBOCUP_ROOT}/util/docker/getsetupsha.sh)"
IMAGE_NAME="robojackets/robocup-baseimage"

if curl -s https://registry.hub.docker.com/v1/repositories/${IMAGE_NAME}/tags |  fgrep -q "\"name\": \"${SHA_SUM}\""; then
    # The tag currently exists, and can be pulled
    docker pull ${IMAGE_NAME}:${SHA_SUM}
else
    # The tag does not exist, let's build it!
    docker build -t ${IMAGE_NAME}:in_progress  -f ${ROBOCUP_ROOT}/util/docker/baseimage/Dockerfile .
    docker run \
        -v ${ROBOCUP_ROOT}:/home/developer/robocup-software ${IMAGE_NAME}:in_progress \
        sh -c './util/ubuntu-setup --yes --firmware && sudo apt-get clean'

    docker commit "$(docker ps -aq | head -n1)" ${IMAGE_NAME}:${SHA_SUM}
    docker push ${IMAGE_NAME}:${SHA_SUM}
fi

