#!/bin/bash

# Script that loads a progam directly into the mbed microcontroller's flash memory and launches its execution

UTIL_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Become root
if [ $UID -ne 0 ]; then
    echo "--  Becoming root"
    exec sudo $0 $@
fi

MBED_DEV=$(sudo blkid -L MBED)
MBED_DEVICES_PATH="$(ls /dev/disk/by-id/ | grep -i mbed | sed 's\.*\/dev/disk/by-id/&\g')"

if [ -z "$MBED_DEV" ]; then
    echo "--  No mbed device(s) found"
    exit 0
else
    echo "--  device found at $MBED_DEV"

    IFS=' ' read -a dev_fields <<< $(mount | grep $MBED_DEV)
    MNT_DIR=$(echo "${dev_fields[2]}")

    if [ -z "$MBED_DEV" ];then
        if [ -z "${MNT_DIR#/}" ]; then
            echo "--  mbed is not mounted"
            exit 0
        fi
    else
        PRGM_SCRIPT="$UTIL_DIR/mbed-prgm.py"
        sudo python $PRGM_SCRIPT --file=$1 --destination=$MNT_DIR
    fi

fi
