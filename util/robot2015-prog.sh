#!/bin/bash

# This script takes the argument as a filename and copies it to all mbeds attached!

if [ ! -e "$1" ]; then
	echo "File $1 not found..."
	exit 1
fi

if [ ! -e "/dev/disk/by-id" ]; then
	echo "Your linux distro is not supported =("
	exit 2
fi

MBED_DEVICES="$(ls /dev/disk/by-id/ | grep mbed)"

# This prepends the directory structure to all found mbed devices
MBED_DEVICES_PATH="$(ls /dev/disk/by-id/ | grep mbed | sed 's\.*\/dev/disk/by-id/&\g')"

if [ -z $MBED_DEVICES ]; then
	echo "No mbed device detected!"
	exit 3
fi

echo -e Attempting to mount $MBED_DEVICES
echo -e Devices path is $MBED_DEVICES_PATH


sudo mkdir -p /mnt/script/MBED
sudo mount $MBED_DEVICES_PATH /mnt/script/MBED
sudo cp $1 /mnt/script/MBED/
sudo umount /mnt/script/MBED/
sudo rmdir /mnt/script/MBED

# echo  $(ls /dev/disk/by-id/ | grep mbed)
