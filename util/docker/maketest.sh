#!/bin/bash
# Builds our circle image which runs tests
set -e

DIR=$(cd $(dirname $0) ; pwd -P)
ROBOCUP_ROOT="${DIR}/../../"

# Get sha sum
SHA_SUM="$(git rev-parse HEAD)"
SUCCESS=true

if [ "$1" == "" ]; then
    echo "Token needed to update statuses"
else
    curl -u "jgkamat:$1" https://api.github.com/jgkamat
fi


# Go to root so we can make.
cd ../../

# TODO Auth
curl -u jgkamat -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"pending", "description": "A check for compiling", "context": "circle/compile", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
curl -u jgkamat -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"pending", "description": "A check for cpp tests", "context": "circle/test-cpp", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
curl -u jgkamat -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"pending", "description": "A check for style", "context": "circle/style", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
curl -u jgkamat -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"pending", "description": "A check for firmware", "context": "circle/firmware", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'


if make; then
    curl -u jgkamat -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"success", "description": "A check for compiling", "context": "circle/compile", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
else
    curl -u jgkamat -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"failure", "description": "A check for compiling", "context": "circle/compile", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
    SUCCESS=false
fi

if make test-cpp; then
    curl -u jgkamat -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"success", "description": "A check for cpp tests", "context": "circle/test-cpp", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
else
    curl -u jgkamat -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"failure", "description": "A check for cpp tests", "context": "circle/test-cpp", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
    SUCCESS=false
fi

if make robot2015; then
    curl -u jgkamat -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"success", "description": "A check for firmware", "context": "circle/firmware", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
else
    curl -u jgkamat -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"failure", "description": "A check for firmware", "context": "circle/firmware", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'
    SUCCESS=false
fi

# TODO style check
curl -u jgkamat -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} -H "Content-Type: application/json" -d '{"state":"success", "description": "A check for style", "context": "circle/style", "target_url": "https://circleci.com/gh/RoboJackets/robocup-software"}'

# Fail the build if anything fails
return $($SUCCESS)
