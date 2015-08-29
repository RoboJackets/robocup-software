#!/bin/bash

SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

cd $SCRIPT_DIR
cd ..
set -e

make robot2015-i2c-prog
make robot2015-io-expander-prog
make robot2015-piezo-prog
# synthesize the fpga's bitfile and move it to the mbed for the fpga test
make fpga2015-prog
make robot2015-fpga-prog
