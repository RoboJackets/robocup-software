#!/bin/bash 
ros2 param set planner collect.touch_delta_speed .03
ros2 param set planner pivot.radius_multiplier 1.1
ros2 param set planner collect.approach_dist_target 0.01
ros2 param set planner collect.control_accel_scale 0.1
ros2 param set planner collect.dist_cutoff_to_control 0.05
ros2 param set planner collect.stop_dist_scale 0.5
