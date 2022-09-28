#!/bin/bash

# This script is extremely similar to real2play.bash. See that script for details.
# this exhibits weird behavior on CTRL-C / close because the method in real2play.bash didn't work quite as intended

# TODO: why does this only work when you launch blue first?
# launching yellow first causes blue's to throw an error:
# [sim_radio_node-8] terminate called after throwing an instance of 'boost::wrapexcept<boost::system::system_error>'
# [sim_radio_node-8]   what():  bind: Address already in use

trap "exit" INT
ROS_DOMAIN_ID=0 ros2 launch rj_robocup soccer.launch.py team_flag:=-b direction_flag:=minus &
sleep 1
ROS_DOMAIN_ID=1 ros2 launch rj_robocup soccer.launch.py &
wait
