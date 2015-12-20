#!/bin/bash

# This script takes the argument as a filename and copies it to all mbeds attached!

# this causes the script to fail if any command below fails
set -e

# checks for file existence
if [ ! -e "$1" ]; then
    echo "File $1 not found..."
    exit 1
fi

# checks to see if the device files used are present
if [ ! -e "/dev/disk/by-id" ]; then
    echo "Your linux distro is not supported =("
    exit 2
fi

SHA2="$(sha256sum $1 | awk '{print $1}')"

MBED_DEVICES="$(ls /mnt/robot2015/ | grep -i mbed)"

# errors out if no mbed devices were found
if [ -z $MBED_DEVICES ]; then
    echo "No mbed device detected!"
    exit 3
fi

# loop through all mbed devices, mount them, copy over the bin file, and unmount them
pushd /mnt/robot2015 &> /dev/null
for path in ./mbed*; do
    [ -d "${path}" ] || continue # if not a directory, skip
    i="$(basename "${path}")"
    echo Installing on $i
    cp -f $1 $i
done; popd &> /dev/null

MEM_SPACE=$(($(stat --printf="%s" $1)/512))
echo $(($MEM_SPACE/10))"."$(($MEM_SPACE%10))"% of available memory filled."

# here's hoping the copy takes less than 5 seconds
sleep 4

# use newlines as delimiters in the forloop, rather than all whitespace
IFS=$'\n'

# restart mbed code
MBED_SERIAL_PATH="$(ls /dev/ | grep ttyACM | sed 's\.*\/dev/&\g')"

# send break signal to all mbeds
for i in $MBED_SERIAL_PATH; do
python3 - <<PY_END

import serial;
try:
    print("Attempting reboot on $i")
    serial.Serial("$i").sendBreak()
    print("Reboot started")
except:
    print("Failed to reboot $i")

PY_END
done
