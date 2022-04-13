#!/bin/bash
DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
DIR=${DIR}/../build-release-debug$
echo $DIR$
cd $DIR$
ninja install