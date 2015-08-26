#!/bin/bash

COMMON_DIR=$(cd `dirname "${BASH_SOURCE[0]}"` && pwd)/

ROBOCUP_ROOT="${COMMON_DIR}/../../"

# Get sha sum
SHA_SUM_SETUP="$(${ROBOCUP_ROOT}/util/docker/getsetupsha.sh)"
SHA_SUM="$(git rev-parse HEAD)"
IMAGE_NAME_CI="robojackets/robocup-ci"
IMAGE_NAME_BASE="robojackets/robocup-baseimage"
