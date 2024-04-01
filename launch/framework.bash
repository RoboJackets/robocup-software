#!/bin/bash

flag_t = false

#check for test flag
while getopts ":t" opt; do
  case ${opt} in
    t )
      flag_t = true
      ;;
    \? )
      echo "Invalid option: $OPTARG" 1>&2
      exit 1
      ;;
    : )
      echo "Invalid option: $OPTARG requires and argument" 1>&2
      exit 1
      ;;
  esac
done
shift $((OPTIND -1))
# this script launches the ER-force framework and our simulator UI simultaneously and allows you to kill both at once using ctrl-c
# you may need to press ctrl-c twice
echo "launching ER-Force framework and our UI"
# kill existing ER-Force simulator instances
pkill .*simulator-cli
# in a subshell: 
# first trap SIGINT to kill the entire subshell
# then find the location of simulator-cli in the home directory and run it with args -g 2020B (in background)
# then run sim through makefile shortcut (in background)
# finally wait, so that the simulator-cli process isn't lost/disowned when run-our-stack dies before it does.
if ["$flag_t" = false]; then
  (trap 'exit' SIGINT SIGTERM; trap 'kill 0' EXIT; find ~ -name 'simulator-cli' -type f -exec '{}' -g 2020B ';' & make run-our-stack & wait)
else
    (trap 'exit' SIGINT SIGTERM; trap 'kill 0' EXIT; find ~ -name 'simulator-cli' -type f -exec '{}' -g 2020B ';' & make run-with-path-tests & wait)
fi
pkill .*simulator-cli