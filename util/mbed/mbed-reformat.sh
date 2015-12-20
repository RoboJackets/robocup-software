#!/bin/bash

# Script that will reformat an mbed's USB storage sector.

SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source $SCRIPT_DIR/colors

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
        if [ -z "$MNT_DIR" ]; then
            echo "--  undefined mount path"
            exit 0
        fi

        echo "--  unmounting from $MNT_DIR"
        umount $MBED_DEV
    fi
    
    echo "--  formatting mbed storage at $MBED_DEV"
    echo "--  $(sudo mkfs.vfat -F 12 -I -nMBED $MBED_DEV)"
    
    echo "--  mounting $MBED_DEV to $MNT_DIR"
    mount -t auto "$MBED_DEV" "$MNT_DIR"
fi

exit 1
