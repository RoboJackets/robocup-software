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

MBED_DEVICES="$(ls /dev/disk/by-id/ | grep -i mbed)"

# This prepends the directory structure to all found mbed devices
MBED_DEVICES_PATH="$(ls /dev/disk/by-id/ | grep -i mbed | sed 's\.*\/dev/disk/by-id/&\g')"

SHA2="$(sha256sum $1 | awk '{print $1}')"

# errors out if no mbed devices were found
if [ -z $MBED_DEVICES ]; then
    echo "No mbed device detected!"
    exit 3
fi

echo -e Devices path is $MBED_DEVICES_PATH

# this causes the script to fail if any command below fails
set -e

# use newlines as delimiters in the forloop, rather than all whitespace
IFS=$'\n'

# loop through all mbed devices, mount them, copy over the bin file, and unmount them
for i in $MBED_DEVICES_PATH; do
    echo Installing on $i
    sudo mkdir -p /mnt/script/MBED
    sudo mount $i /mnt/script/MBED
    sudo cp -f -b $1 /mnt/script/MBED/

    if [ "$SHA2" != "$(sha256sum /mnt/script/MBED/$(echo "$1" | awk 'BEGIN {FS = "/"}; {print $NF}') | awk '{print $1}')" ]; then
        echo ERROR: Sha Hashes do not match, copy failed. Exiting...
        exit 1
    else
        echo Sha256sums of binary files match, copy successful!
    fi
done

MEM_SPACE=$(($(stat --printf="%s" $1)/512))
echo $(($MEM_SPACE/10))"."$(($MEM_SPACE%10))"% of available memory filled."

# here's hoping the copy takes less than 5 seconds
# TODO find a better solution...
sleep 4

# restart mbed code
MBED_SERIAL_PATH="$(ls /dev/ | grep ttyACM | sed 's\.*\/dev/&\g')"

for i in $MBED_SERIAL_PATH; do
    echo Attempting reboot on $i ...
    # send break signal to all mbeds
    sudo python3 -c "import serial; serial.Serial(\"$i\").sendBreak()"
done

