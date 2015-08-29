#!/bin/bash

# Script that loads a progam directly into the mbed microcontroller's flash memory and launches its execution

SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source $SCRIPT_DIR/colors

# Become root
if [ $UID -ne 0 ]; then
    echo "--  Becoming root"
    exec sudo $0 $@
fi

# checks for passed file existence
if [ ! -e "$1" ]; then
    echo "${RED}${WHITEBG}${INVT}File $1 not found...${R}"
    exit 1
fi

MBED_DEV=$(sudo blkid -L MBED)
MBED_DEVICES_PATH="$(ls /dev/disk/by-id/ | grep -i mbed | sed 's\.*\/dev/disk/by-id/&\g')"

if [ -z "$MBED_DEV" ]; then
    msg="No mbed device detected!"
    printf "$WHITE $REDBG $PUTLN \n"
    printf "%*s\n" $(((${#msg}+$(tput cols))/2)) "$msg"     # center align the message
    printf "$PUTLN$R\n"
    exit 0
else
    echo "--  device found at $MBED_DEV"

    IFS=' ' read -a dev_fields <<< $(mount | grep $MBED_DEV)
    MNT_DIR=$(echo "${dev_fields[2]}")

    if [ -z "$MBED_DEV" ];then
        if [ -z "${MNT_DIR#/}" ]; then
            msg="mbed found but has no mount point!"
            printf "$WHITE $REDBG $PUTLN \n"
            printf "%*s\n" $(((${#msg}+$(tput cols))/2)) "$msg"     # center align the message
            printf "$PUTLN$R\n"
            exit 0
        fi
    else
        PRGM_SCRIPT="$SCRIPT_DIR/mbed-prgm.py"
        python $PRGM_SCRIPT --file=$1 --destination=$MNT_DIR

        if [ $? -eq 0 ]; then
        # Yay, we made it!
        msg1="${BOLD}File transfer SUCCESS!${AOFF}${WHITE}${GREENBG}"
        msg2="Sent ${BLUEBG}$1${GREENBG} to the mbed${AOFF}${WHITE}${GREENBG}"

        printf "$WHITE$GREENBG$PUTLN\n"
        printf "%*s\n" $(((${#msg1}+$(tput cols))/2)) "$msg1"   # center align the message
        printf "%*s\n" $(((${#msg2}+$(tput cols))/2)) "$msg2"     
        printf "$PUTLN$R\n"
     else
        # something went wrong
        msg="${BOLD}File transfer FAILED!${AOFF}${WHITE}${GREENBG}"
        msg2="Could not send ${BLUEBG}$1${GREENBG} to the mbed${AOFF}${WHITE}${GREENBG}"

        printf "$REDBG$PUTLN\n"
        printf "%*s\n" $(((${#msg1}+$(tput cols))/2)) "$msg1"   # center align the message
        printf "%*s\n" $(((${#msg2}+$(tput cols))/2)) "$msg2"     
        printf "$PUTLN$R\n"
    fi
    fi

fi
