#!/bin/bash
# Builds our circle image which runs tests

DIR=$(cd $(dirname $0) ; pwd -P)
ROBOCUP_ROOT="${DIR}/../../"

# Get sha sum
SHA_SUM="$(git rev-parse HEAD)"
USER="jgkamat"
SUCCESS=true
LINK_PREFIX="https://circle-artifacts.com/gh/RoboJackets/robocup-software/$CIRCLE_BUILD_NUM/artifacts/0$CIRCLE_ARTIFACTS/"
ARTIFACT_DIR="/tmp/build_artifacts"

if [ "$1" == "" ]; then
    echo "Token needed to update statuses"
    exit 1
else
    TOKEN="$1"
fi


# Go to root so we can make.
cd ${ROBOCUP_ROOT}


# Clean
make clean
git submodule update --init
sudo chown -R `whoami`:`whoami` ${HOME}/.ccache

# TODO Auth
curl -u $USER:$TOKEN -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"pending", "description": "A check for compiling", "context": "circle/compile", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
curl -u $USER:$TOKEN -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"pending", "description": "A check for cpp tests", "context": "circle/test-cpp", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
curl -u $USER:$TOKEN -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"pending", "description": "A check for style", "context": "circle/style", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
curl -u $USER:$TOKEN -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"pending", "description": "A check for firmware", "context": "circle/firmware", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'

make | tee "${ARTIFACT_DIR}/makeoutput.txt"
if [ "$?" = "0" ]; then
    curl -u $USER:$TOKEN -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"success", "description": "A check for compiling", "context": "circle/compile", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
else
    curl -u $USER:$TOKEN -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"failure", "description": "A check for compiling", "context": "circle/compile", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
    SUCCESS=false
fi

make test-cpp | tee "${ARTIFACT_DIR}/testcppoutput.txt"
if [ "$?" = "0" ]; then
    curl -u $USER:$TOKEN -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"success", "description": "A check for cpp tests", "context": "circle/test-cpp", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
else
    curl -u $USER:$TOKEN -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"failure", "description": "A check for cpp tests", "context": "circle/test-cpp", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
    SUCCESS=false
fi

make robot2015 | tee "${ARTIFACT_DIR}/firmwareoutput.txt"
if [ "$?" = "0" ]; then
    curl -u $USER:$TOKEN -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"success", "description": "A check for firmware", "context": "circle/firmware", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
else
    curl -u $USER:$TOKEN -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"failure", "description": "A check for firmware", "context": "circle/firmware", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
    SUCCESS=false
fi

# TODO style check
curl -u $USER:$TOKEN -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"success", "description": "A check for style", "context": "circle/style", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'

exit 0
# exit $($SUCCESS)

