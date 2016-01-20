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
    mnt_point="/mnt/script/$(basename "$i")"

    sudo mkdir -p $mnt_point
    sudo mount $i $mnt_point
    sudo cp -f -b $1 $mnt_point

    # Call the golden command
    sync
    
    # compute the sha256sum of the moved file before we can't access it anymore
    SHA2_CP="$(sha256sum $mnt_point/$(echo "$1" | awk 'BEGIN {FS = "/"}; {print $NF}') | awk '{print $1}')"

    # lazy unmount
    sudo umount -l $mnt_point

    # see if it is still mounted, if successful unmount, remove directory that was created earlier
    if ! mount | grep "$mnt_point" &> /dev/null; then
        if [ "$(ls -A $mnt_point)" ]; then
            # dir not empty
            :
        else
            # dir is empty
            sudo rm -rf "$mnt_point"
        fi
    fi

    if [ "$SHA2" != "$SHA2_CP" ]; then
        echo "Error: Mismatched sha256sums, copy failed. Exiting..."
        echo "    src:  $SHA2"
        echo "    dst:  $SHA2_CP"
        exit 1
    else
        echo "sha256sum matched, copy successful!"
        echo "    sha256sum: $SHA2"
        # unset this before the next iteration if there is one
        SHA2_CP=""
    fi
done

MEM_SPACE=$(($(stat --printf="%s" $1)/512))
echo $(($MEM_SPACE/10))"."$(($MEM_SPACE%10))"% of available memory filled."

# restart mbed code
MBED_SERIAL_PATH="$(ls /dev/ | grep ttyACM | sed 's\.*\/dev/&\g')"

for i in $MBED_SERIAL_PATH; do
    echo Attempting reboot on $i ...
    # send break signal to all mbeds
    sudo python3 -c "import serial; serial.Serial(\"$i\").sendBreak()"
done

