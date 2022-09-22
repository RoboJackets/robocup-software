#!/bin/bash

# This script runs two instances of soccer at once, with slightly different
# network configs, to allow for simulating two teams at once on the field.
#
# It is equivalent to calling the following in separate terminals:
#
# ROS_DOMAIN_ID=0 make run-real
# ROS_DOMAIN_ID=1 make run-alt-real
#
# One CTRL-C will terminate both processes.
# See: https://stackoverflow.com/questions/3004811/how-do-you-run-multiple-programs-in-parallel-from-a-bash-script

(trap 'kill 0' SIGINT; (ROS_DOMAIN_ID=0 make run-real) & (ROS_DOMAIN_ID=1 make run-alt-real))
