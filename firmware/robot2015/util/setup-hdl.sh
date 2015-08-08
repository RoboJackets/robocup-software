#!/bin/bash

# Script that will setup a machine for HDL simulation & synthesis

# Get the directory we're in 
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Become root
if [ $UID -ne 0 ]; then
    echo "-- Becoming root"
    exec sudo $0 $@
fi

# Detect the python version if any
TMPVAR=$(python -c 'import sys; print(sys.version_info[:])')
if [ -z "$TMPVAR" ]; then
    echo "-- Python not found"
    exit 0
else
    PYTHON_VERSION=${TMPVAR#"("}
    PYTHON_VERSION=${PYTHON_VERSION%")"}
    IFS=, read -a v <<<"$PYTHON_VERSION"
    PY_MAJOR=$(echo -e "${v[0]}" | tr -d '[[:space:]]')
    PY_MINOR=$(echo -e "${v[1]}" | tr -d '[[:space:]]')
    PYTHON_VERSION=$(printf "%d.%d" $PY_MAJOR $PY_MINOR) 

    echo "-- Detected python version $PYTHON_VERSION"
fi

# Add the PPA for install ghdl
PPA_CHECK=$(grep ^ /etc/apt/sources.list /etc/apt/sources.list.d/* | grep ghdl)
if [ -z "$PPA_CHECK" ]; then
    echo "-- Adding PPA for GHDL"
    add-apt-repository -y ppa:pgavin/ghdl
else
    echo "-- PPA for GHDL found"
fi

# Make sure pip can be used along with running test - also install Icarus iverilog
apt-get update
apt-get -y install python-pip python-test

# These must be run with sudo and be placed in /bin/sh?
sudo apt-get -y install ghdl gtkwave verilog

# Install MyHDL
pip install MyHDL       # Everything from here until the end is optional

# Clone the official MyHDL repo for running the python tests
echo "-- Verifying the MyHDL install"
git clone https://github.com/jandecaluwe/myhdl $DIR/tmp-myhdl

# Run the test script
cd $DIR/tmp-myhdl/myhdl/test/core
py.test

# Compile the 'myhdl.vpi' PLI module
cd $DIR/tmp-myhdl/cosimulation/icarus
make
cp myhdl.vpi /usr/bin/myhdl.vpi

# Test co-simulation
cd $DIR/tmp-myhdl/cosimulation/icarus/test
python test_all.py

echo "-- Cleaning up"
cd $DIR
rm -rf tmp-myhdl

echo "Setup Complete."
