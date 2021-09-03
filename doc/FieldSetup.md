# Field Setup

## Manual Control

In a new terminal:

1. cd to robocup-software
2. launch manual.launch.py with ros2
```
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch rj_robocup manual.launch.py
```

In a 2nd new terminal:
cd to robocup-software
1. open rqt: `rqt`
2. Go to Plugins > Service Caller
3. Select `/list_joystick`, note string for keyboard or controller 
4. Then select `/select_manual`, set `robot_id`, set `controller_uuid` to the string found in the last step (`'keyboard-controller'` for kb), set connect to True
5. Hit call.

Keep focus on the black window and WASD should move the robot, QE to pivot.

