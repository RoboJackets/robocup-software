#!/bin/bash

# Script that will reformat an mbed's USB storage sector.

# Become root
if [ $UID -ne 0 ]; then
    echo "-- Becoming root"
    exec sudo $0 $@
fi

MBED_DEV=$(sudo blkid -L MBED)

if [ -z "$MBED_DEV" ]; then
    echo "-- No mbed devices found"
    exit 0
else
    echo "-- mbed found at $MBED_DEV"

    MNT_RES=$(mount | grep $MBED_DEV)

    if [ -z "$MNT_RES" ]; then
        echo "-- mbed is not mounted"
    else
        echo "-- mbed is currently mounted...unmounting"
        umount $MBED_DEV
        echo "-- mbed unmounted"
    fi
    
    echo "-- formatting mbed's USB storage"
    sudo mkfs.vfat -F 12 -I -nMBED $MBED_DEV

    echo "-- done. You must remount the mbed at its next use."
    
fi
