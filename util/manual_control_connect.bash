#!/bin/bash 
 
display_help()
{
    echo "Usage: $0 [-h] [-r 0] [-c keyboard-controller] [-C True]"
    echo 
    echo "Options:"
    echo "  -h   Show this help message."
    echo "  -r   Control robot_id manually (default: 0)."
    echo "  -c   Connect controller (default: keyboard-controller)."
    echo "  -C   Connect flag, set to False to disconnect (default: True)."
    # never do anything after displaying help msg
    exit
}

# if -h is first flag, display help msg above and exit
# otherwise, take in args per the help msg above, then call the ros2 service
robot_id=0
controller_uuid="keyboard-controller"
connect=True

while getopts h:r:c:C: flag
do
    case "${flag}" in
        h) display_help;;
        r) robot_id=${OPTARG};;
        c) controller_uuid=${OPTARG};;
        C) connect=${OPTARG};;
    esac
done

cmd="ros2 service call /select_manual rj_msgs/srv/SetManual \"{robot_id: $robot_id, controller_uuid: $controller_uuid, connect: $connect}\""

eval " $cmd"

