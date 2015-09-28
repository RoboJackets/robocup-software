#!/bin/bash
# Pushes the baseimage to the master tag
set -e

DIR=$(cd $(dirname $0) ; pwd -P)
source ${DIR}/../docker_common.sh

docker tag -f ${IMAGE_NAME_BASE}:current ${IMAGE_NAME_BASE}:master

if [ -n "$DOCKER_PASS" ]; then
    docker login -e $DOCKER_EMAIL -u $DOCKER_USER -p $DOCKER_PASS
    docker push ${IMAGE_NAME_BASE}:master
fi
