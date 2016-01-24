#!/bin/bash

# This script takes the argument as a filename and copies it to all mbeds attached!

# this causes the script to fail if any command below fails
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SCRIPT_NAME="$(basename "$0")"
MNT_ROOT="$HOME/mbed-mount-"$(date +"%m-%d-%y-%T")""

function usage {
    echo "Usage: ./$SCRIPT_NAME <file-to-copy>"
}

function echo_line {
    cc=$1
    nn="$(tput cols)"
    zz="$(printf "%-${nn}s" "$cc")"
    echo "${zz// /$cc}"
}

function finalize {
    if ! mount | grep "$MNT_ROOT" &> /dev/null; then
        if [ -e "$MNT_ROOT" ]; then
            rm -r "$MNT_ROOT"
        fi
    else
        echo
        echo_line "*"
        echo_line "*"
        echo -e "\n$SCRIPT_NAME: MANUAL CLEANUP MAY BE REQUIRED! Failed to unmount mbed device(s)! Details given below.\n"
        echo "Try running the following command to see details about mount points:"
        echo "    'mount | grep "$MNT_ROOT"'"
        echo
        echo "Directories that may need unmounting:"
        for d in "$MNT_ROOT"/*/; do
            echo "    $(readlink -f $d)"
        done
        echo
        echo "Once all paths under $MNT_ROOT are unmounted, the is may be removed with the following command:"
        echo "    'rm -r $MNT_ROOT'"
        echo
        echo_line "*"
        echo_line "*"
        echo
    fi

    exit $1
}

function err_exit {
    echo "$SCRIPT_NAME: ${1:-"unknown error"}" 1>&2
    finalize 1
}

function err_exit_on_copy {
    # unmount the mbed
    sudo umount "$mnt_point"
    err_exit "failed to move files to external device(s)"
}

# set the callback function on any exit condition
trap finalize SIGINT SIGTERM EXIT

if [ "$#" -ne 1 ]; then
    usage
    err_exit "one file to send to connected mbeds must be specified"
fi

if echo "$OSTYPE" | grep -i "darwin" &> /dev/null; then
    err_exit "osx is not supported"
fi

# get absolute path to source file and the file's name itself
COPY_FILE="$(readlink -m "$1")"
COPY_FN="$(basename "$COPY_FILE")"

# checks for file existence
if [ ! -e "$COPY_FILE" ]; then
    err_exit "file does not exist -- '$COPY_FN'"
fi

# checks to see if the device files used are present
if [ ! -e "/dev/disk/by-id" ]; then
    err_exit "unsupported os distro =("
fi

# turn off exiting on failure fo these commands
set +e
MBED_DEVICES="$(ls /dev/disk/by-id/ | grep -i "mbed")"
# This prepends the directory structure to all found mbed devices
MBED_DEVICES_PATH="$(ls /dev/disk/by-id/ | grep -i mbed | sed 's\.*\/dev/disk/by-id/&\g')"
SHA2="$(sha256sum "$COPY_FILE" | awk '{print $1}')"
set -e

# errors out if no mbed devices were found
if [ -z "$MBED_DEVICES" ]; then
    err_exit "no mbed(s) found"
fi

echo -e "device path is $MBED_DEVICES_PATH"

# use newlines as delimiters in the forloop, rather than all whitespace
IFS=$'\n'

# loop through all mbed devices, mount them, copy over the bin file, and unmount them
for i in $MBED_DEVICES_PATH; do
    mnt_point="$MNT_ROOT"/"$(basename "$i")"
    echo "installing to $mnt_point"

    mkdir -p "$mnt_point"
    sudo mount "$i" "$mnt_point"

    # redirect this critical section to a different callback on errors
    trap err_exit_on_copy SIGINT SIGTERM EXIT

    sudo cp -f "$COPY_FILE" "$mnt_point"

    # Call the golden command
    sync

    # compute the sha256sum of the moved file before we can't access it anymore
    SHA2_CP="$(sha256sum "$mnt_point"/"$COPY_FN" | awk '{print $1}')"

    # reset the trap callback
    trap finalize SIGINT SIGTERM EXIT

    sudo umount "$mnt_point"

    # compare the checksums
    if [ "$SHA2" != "$SHA2_CP" ]; then
        echo "src sha256sum: $SHA2"
        echo "dst sha256sum: $SHA2_CP"
        err_exit "mismatched sha256sums, copy failed"
    else
        echo "matched sha256sum, copy successful"
        echo "    sha256sum: $SHA2"
        
    fi

    # unset this before the next iteration
    SHA2_CP=
done

if ! which python3 &> /dev/null; then
    echo "no python3 installation detected, skipping device reset"
    exit 0
fi

# send break signal to all mbeds
set +e
MBED_SERIAL_PATH="$(ls /dev/ | grep ttyACM | sed 's\.*\/dev/&\g')"
for i in $MBED_SERIAL_PATH; do
    echo "attempting reboot on $i"
    # if ! python3 -c "import serial; serial.Serial(\"$i\").sendBreak()" &> /dev/null; then
    #     sudo python3 -c "import serial; serial.Serial(\"$i\").sendBreak()"
    # fi
    sudo python3 "$SCRIPT_DIR/mbed-reset.py" "$i" 
    sleep 2
done

MEM_SPACE=$(($(stat --printf="%s" "$COPY_FILE")/512))
echo "mbed binary memory utilization: "$(($MEM_SPACE/10))"."$(($MEM_SPACE%10))"%"

exit 0
