#!/bin/bash
# Runs tests and reports on them to github

DIR=$(cd $(dirname $0) ; pwd -P)
source ${DIR}/docker_common.sh

# Get sha sum
USER="${GH_USER:-georgeburdell}"
SUCCESS=true
LINK_PREFIX="https://circle-artifacts.com/gh/RoboJackets/robocup-software/$CIRCLE_BUILD_NUM/artifacts/0$CIRCLE_ARTIFACTS/"
ARTIFACT_DIR="/tmp/build_artifacts"
PENDING=false
SHORTNAMES=( )

start_pending() {
    SHORTNAME="$1"
    >/dev/null curl -s -u $USER:$TOKEN \
        -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} \
        -H "Content-Type: application/json" \
        -d '{"state":"pending", "description": "This check is pending. Please wait.", "context": '"\"circle/$SHORTNAME\""', "target_url": "http://www.robojackets.org/"}'
}

# A function to run a command. Takes in a command name, shortname and description.
ci_task() {
    CMD="$1"
    SHORTNAME="$2"
    DESCRIPTION="$3"

    SHORTNAMES+=("${SHORTNAME}")

    if [ "$PENDING" = "true" ]; then
        return 0
    fi

    ${CMD} 2>&1 | tee "${ARTIFACT_DIR}/${SHORTNAME}.txt"
    if [ "${PIPESTATUS[0]}" = "0" ]; then
        >/dev/null curl -s -u $USER:$TOKEN \
            -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} \
            -H "Content-Type: application/json" \
            -d '{"state":"success", "description": '"\"${DESCRIPTION}\""', "context": '"\"circle/${SHORTNAME}\""', "target_url": '""\"${LINK_PREFIX}${SHORTNAME}.txt\""}"
    else
        >/dev/null curl -s -u $USER:$TOKEN \
            -X POST https://api.github.com/repos/robojackets/robocup-software/statuses/${SHA_SUM} \
            -H "Content-Type: application/json" \
            -d '{"state":"failure", "description": '"\"${DESCRIPTION}\""', "context": '"\"circle/${SHORTNAME}\""', "target_url": '""\"${LINK_PREFIX}${SHORTNAME}.txt\""}"
        SUCCESS=false >/dev/null
    fi
}

if [ "$1" = "" ]; then
    echo "Token needed to update statuses"
    exit 1
elif [ "$1" = "--pending" -a "$2" = "" ]; then
    echo "Token needed to update statuses"
    exit 1
elif [ "$1" = "--pending" -a "$2" != "" ]; then
    TOKEN="$2"
    PENDING=true
else
    TOKEN="$1"
fi

# Go to root so we can make.
cd ${ROBOCUP_ROOT}

# Clean
make clean
git submodule update --init
sudo chown -R `whoami`:`whoami` ${HOME}/.ccache

ci_task 'make' 'compile' 'A check to see if the code compiles'
ci_task 'make test-soccer' 'test-soccer' 'A check to see if soccer tests pass'
ci_task 'make robot2015' 'firmware' 'A check to see if firmware works'
ci_task 'make test-firmware' 'test-firmware' 'A check to see if firmware tests pass'
ci_task 'make checkstyle' 'style' 'A check to see if style passes'

# This script needs to be run prior with the --pending flag if you want to see pending flags
if [ "$PENDING" = "true" ]; then
    for i in ${SHORTNAMES[*]}; do
        start_pending ${i}
    done
fi

$SUCCESS
exit $?

