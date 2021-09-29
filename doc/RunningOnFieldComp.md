# How to Run Things on the Field Computer

This guide assumes you know what a terminal is and how to cd (change
directories). Otherwise, everything is explicitly laid out. Note: when the
instructions say "new terminal", a new tab will also do.

## Branch

In a new terminal, run the following line:
```git checkout ros2
```
## Launching Soccer

In a new terminal, run the following lines one-by-one: ``` source
/opt/ros/foxy/setup.bash 
make perf
source install/setup.bash 
ros2 launch rj_robocup soccer.launch.py
```

This should open soccer, good for testing radio connection. If you want to run
gameplay, you have to start vision (keep scrolling).

## Open Ref

In a new terminal:
1. `cd ~/ssl-game-controller`
2. `./ssl-game-controller_v2.12.7_linux_amd64` (Just hit tab after typing ssl)
3. Click open link on the line that states UI is available at ...
4. Make sure at least one team is namd RoboJackets, if not click the top left gear, click pen next to one of the names and find RoboJackets in the list.

## Manual Control

In a new terminal:

1. cd to robocup-software
2. launch manual.launch.py with the following commands: ``` source
   /opt/ros/foxy/setup.bash 
   make perf 
   source install/setup.bash 
   ros2 launch rj_robocup manual.launch.py
   ```

In a new terminal:

1. cd to robocup-software
2. Input `source /opt/ros/foxy/setup.bash`
3.INput `source install/setup.bash`
4. launch rqt: `rqt`

Then, within rqt:
1. Go to Plugins > Services > Service Caller
`How do i get to list joystick
2. From the Service dropdown menu, `/list_joystick`, then click the call button on the top right. Take note the string given for keyboard at uuids or
   controller input (e.g. `'keyboard-controller'` for kb)
3. From the Service dropdown menu, select `/select_manual`, then in this menu:
   1. set the desired `robot_id` by clicking on the value under "Expression" and typing the desired number (e.g. 1 for robot 1)
   2. set `controller_uuid` to the string found in the list_joystick step, surrounded with single quotes ('')
   3. set `connect` to `True`, same as previous two steps
4. Hit call.

Keep focus on the black window (as in, you should click on that window if you
can't see it). 

WASD to move the robot, QE to pivot, K to kick and C to chip.

## Starting Vision

Assuming you have soccer running (see top section), you need the
league-provided vision software to give our gameplay system what it needs to
work. 

In a new terminal: 
`cd ~/ssl-vision` 
`./bin/vision `

This should pull up a GUI of the ssl-vision system. In said GUI, start capture by going to the left sidebar and clicking the arrows to dropdown:
(If you can't see these buttons, try scrolling to the right with the bar on
bottom.)

`Thread 0 -> Image Capture -> Capture Control -> Start Capture`

Then push `start`. 

If our system isn't picking up on vision's input, try:
1. restarting soccer
2. launching the graphical client (`cd ~/ssl-vision && ./bin/graphicalClient`)

If the graphical client doesn't show any robots, vision is broken and something
is terribly wrong. Give an experienced software member a shout.

