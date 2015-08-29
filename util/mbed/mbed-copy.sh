#!/bin/bash

# This script takes the argument as a filename and copies it to all mbeds attached!

# tput smcup

SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source $SCRIPT_DIR/colors

# checks for file existence
if [ ! -e "$1" ]; then
    echo "${RED}${WHITEBG}${INVT}File $1 not found...${R}"
    exit 1
fi

# checks to see if the device files used are present
if [ ! -e "/dev/disk/by-id" ]; then
    echo "${RED}${WHITEBG}${INVT}Your linux distro is not supported =(${R}"
    exit 2
fi

MBED_DEVICES="$(ls /dev/disk/by-id/ | grep -i mbed)"

# This prepends the directory structure to all found mbed devices
MBED_DEVICES_PATH="$(ls /dev/disk/by-id/ | grep -i mbed | sed 's\.*\/dev/disk/by-id/&\g')"

SHA2="$(sha256sum $1 | awk '{print $1}')"

# errors out if no mbed devices were found
if [ -z $MBED_DEVICES ]; then    
    msg="No mbed device detected!"
    printf "$WHITE $REDBG $PUTLN \n"
    printf "%*s\n" $(((${#msg}+$(tput cols))/2)) "$msg"     # center align the message
    printf "$PUTLN$R\n"
    exit 3
fi

echo -e "Devices path is $MBED_DEVICES_PATH"

# restart mbed code
MBED_SERIAL_PATH="$(ls /dev/ | grep ttyACM | sed 's\.*\/dev/&\g')"

# Create a path where we can write to arbitruary mbed(s)
MNT_PATH=/mnt/script/MBED

# this causes the script to fail if any command below fails
set -e

# use newlines as delimiters in the forloop, rather than all whitespace
IFS=$'\n'

sudo mkdir -p $MNT_PATH

# loop through all mbed devices, mount them, copy over the bin file, and unmount them
for i in $MBED_DEVICES_PATH; do
    echo "Installing on $i"

    sudo mount $i $MNT_PATH
    sudo cp $1 $MNT_PATH/

    if [ "$SHA2" != "$(sha256sum /mnt/script/MBED/$(echo "$1" | awk 'BEGIN {FS = "/"}; {print $NF}') | awk '{print $1}')" ]; then
        STYLE=1
        echo "${WHITE}${REDBG}ERROR: Sha Hashes do not match, copy failed. Exiting.${FINLN}${R}"
        exit 1
    else
        echo "${WHITE}${BLUEBG}SHA256SUMS of binary files match, copy ${BOLD}successful${AOFF}${WHITE}${BLUEBG}!${FINLN}${R}"
    fi

    sudo umount -l $MNT_PATH

    if [ $? -eq 0 ]; then
        # Yay, we made it!
        msg1="${BOLD}SUCCESS!${AOFF}${WHITE}${GREENBG} "
        msg2="Safe to disconnect ${BLUEBG}BLUE${GREENBG} light stops blinking."

        printf "$WHITE $GREENBG $PUTLN \n $PUTLN \n"
        printf "%*s\n" $(((${#msg1}+$(tput cols))/2)) "$msg1"
        printf "%*s\n" $(((${#msg2}+$(tput cols))/2)) "$msg2"     # center align the message
        printf "$PUTLN \n $PUTLN$R\n"
     else
        # It won't get more obvious than this...
        msg="UNMOUNT FAILED! PLEASE UNMOUNT '$MNT_PATH' MANUALLY!$FINLN"

        printf "$REDBG $PUTLN \n $PUTLN \n"
        printf "%*s\n" $(((${#msg}+$(tput cols))/2)) "$msg"     # center align the message
        printf "$PUTLN \n $PUTLN$R\n"
    fi

    
    # echo "$(fuser -m $MNT_PATH)"
done &&

sudo rm -rf $MNT_PATH

sleep 5

# Reset the mbed(s)
for i in $MBED_SERIAL_PATH; do
    echo Attempting reboot on $i ...
    # clear buffer, send reboot, enter

    # Wait until no PIDs are using the path
    while [ -n "$(fuser -m $MNT_PATH)" ]; do
        echo "$(fuser -m $i)"
        sleep 0.2
    done

    # reboots the mbed through pyOCD (using the interface mcu)
    pyocd-tool reset

    # send break signal to all mbeds
    # sudo python3 -c "import serial; serial.Serial(\"$i\").sendBreak()"
done &&


# Always remove the created path - even on a failure
if [ -e "$MNT_PATH" ]; then
    echo "directory still exists"

    # Wait until the mbed is not being used
    while [ -n "$(fuser -m $MNT_PATH)" ]; do
        echo "ping"
        sleep 1
    done

    if sudo rmdir $MNT_PATH; then
        echo "Directory force removed"
    else
        echo "Unable to remove directory"
    fi
fi

# tput rmcup
