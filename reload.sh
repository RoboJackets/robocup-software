#!/bin/bash

set -e

BASE_INDEX=0
ROBOT_INDEX=1

export CC1201_CONFIG_NAME=cc1101-compatible

MBED_INDEX=$BASE_INDEX make base2015-prog
MBED_INDEX=$ROBOT_INDEX make robot-prog
