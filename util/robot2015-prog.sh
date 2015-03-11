#!/bin/bash

# This script takes the argument as a filename and copies it to all mbeds attached!

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
    sudo cp $1 /mnt/script/MBED/
    sudo umount -l /mnt/script/MBED/
    if [ $? -eq 0 ]; then
        echo Unmount successful! Do not remove mbed until write light stops blinking!
    else
        echo UNMOUNT FAILED! PLEASE UNMOUNT /mnt/script/MBED MANUALLY!!!!
    fi
    sudo rmdir /mnt/script/MBED

    # This part sends 'reboot\r' over the serial connection. -e enables escape codes, and -n omits the endline at the end of echo, which prints by default.
    # \r is the character that represents the end of the command.
done

# here's hoping the copy takes less than 5 seconds
# TODO find a better solution...
sleep 1

# restart mbed code
MBED_SERIAL_PATH="$(ls /dev/ | grep ttyACM | sed 's\.*\/dev/&\g')"

for i in $MBED_SERIAL_PATH; do
    echo Attempting reboot on $i ...
    # clear buffer, send reboot, enter

    # send break signal to all mbeds
    sudo python3 -c "import serial; serial.Serial(\"$i\").sendBreak()"
    # sudo bash -c "echo -ne '\rreboot\r' > $i"
done


