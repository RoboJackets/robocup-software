#!/bin/bash

# This script is extremely similar to real2play.bash. See that script for details.
# TODO: merge the two

# TODO: why does this only work when you launch blue first?
# launching yellow first causes blue's to throw an error:
# [sim_radio_node-8] terminate called after throwing an instance of 'boost::wrapexcept<boost::system::system_error>'
# [sim_radio_node-8]   what():  bind: Address already in use

(trap 'kill 0' SIGINT; (ROS_DOMAIN_ID=0 ros2 launch rj_robocup soccer.launch.py team_flag:=-b) & (ROS_DOMAIN_ID=1 ros2 launch rj_robocup soccer.launch.py team_flag:=-y))
