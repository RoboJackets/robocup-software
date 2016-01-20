#!/bin/bash

SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source $SCRIPT_DIR/colors

cd $SCRIPT_DIR
cd ../..
set -e

make robot2015-i2c
make robot2015-io-expander
make robot2015-piezo
make robot2015-fpga

msg="${BOLD}All 2015 firmware hw-test targets successfully built!${AOFF}${WHITE}${GREENBG} "
printf "$WHITE $GREENBG $PUTLN \n $PUTLN \n"
printf "%*s\n" $(((${#msg}+$(tput cols))/2)) "$msg"
printf "$PUTLN \n $PUTLN$R\n"
