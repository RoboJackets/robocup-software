#!/bin/bash
# this script launches the ER-force framework and our simulator UI simultaneously and allows you to kill both at once using ctrl-c
# you may need to press ctrl-c twice
echo "launching ER-Force framework and our UI"
# in a subshell: 
# first trap SIGINT to kill the entire subshell
# then find the location of simulator-cli in the home directory and run it with args -g 2020B (in background)
# then run sim through makefile shortcut (in background)
# finally wait, so that the simulator-cli process isn't lost/disowned when run-sim-only dies before it does.
(trap 'kill 0' SIGINT; find ~ -name 'simulator-cli' -type f -exec '{}' -g 2020B ';' & make run-our-stack & wait)

