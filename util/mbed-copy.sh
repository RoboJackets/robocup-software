#!/bin/bash

# This script takes the argument as a filename and copies it to all mbeds attached!

# tput smcup

# Define some colors because...why not?
R1="$(tput setab 9)"
R2="$(tput setaf 9)"
AOFF="$(tput sgr0)"     # Attributes off
R="$AOFF$R1$R2"
# bold/normal font types   
INVT="$(tput smso)"
BOLD="$(tput bold)"
NORM="$(tput rmso)"
# Foreground Colors
BLACK="$(tput setaf 0)"
RED="$(tput setaf 1)"
GREEN="$(tput setaf 2)"
YELLOW="$(tput setaf 3)"
BLUE="$(tput setaf 4)"
PURP="$(tput setaf 5)"
CYAN="$(tput setaf 6)"
WHITE="$(tput setaf 7)"
# Background colors
BLACKBG="$(tput setab 0)"
REDBG="$(tput setab 1)"
GREENBG="$(tput setab 2)"
YELLOWBG="$(tput setab 3)"
BLUEBG="$(tput setab 4)"
PURPBG="$(tput setab 5)"
CYANBG="$(tput setab 6)"
WHITEBG="$(tput setab 7)"
# Insert a blank line
CLRLN="$(tput el1)"
PUTLN="$CLRLN$(tput ich $(tput cols))"
FINLN="$(tput el)"


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
MNT_PATH="/mnt/script/MBED"

# This will trigger anytime a file is closed at the mouting point
# The process exits at the first event trigger. Times out after 10s
sudo mkdir -p $MNT_PATH

for ((i=0;i<${#MBED_DEVICES_PATH[@]};++i)); do

    sudo inotifywait -q -r -t 6 -e close_write "$MNT_PATH" 2>&1 | while read f; do
        echo "${WHITE}${GREENBG}File write success for file ${f}!${FINLN}${R}"
        echo "${WHITE}${YELLOWBG}Unmount succes!${FINLN}${R}"

        # send break signal to all mbeds
        sudo python3 -c "import serial; serial.Serial(\"${MBED_SERIAL_PATH[i]}\").sendBreak()"
        #sudo rmdir $MNT_PATH

        #echo "${WHITE}${YELLOWBG}Starting screen session.${R}"
        # screen -d -m -S mysession
        # screen -L -S mysession split -v -p 0 -X stuff mbed
    done &

done

# this causes the script to fail if any command below fails
set -e

# use newlines as delimiters in the forloop, rather than all whitespace
IFS=$'\n'

# loop through all mbed devices, mount them, copy over the bin file, and unmount them
for i in $MBED_DEVICES_PATH; do
    echo "Installing on $i"
    sudo mount $i $MNT_PATH
    sudo rmdir $MNT_PATH
    sudo cp $1 $MNT_PATH

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
    
done &

wait

# tput rmcup
