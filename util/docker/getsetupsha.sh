#!/bin/bash
DIR=$(dirname "${BASH_SOURCE}")
ROBOCUP_ROOT="${DIR}/../../"

cd ${ROBOCUP_ROOT}/util
cat ubuntu-setup ubuntu-packages.txt requirements2.txt requirements3.txt | sha256sum | awk '{print $1}'
