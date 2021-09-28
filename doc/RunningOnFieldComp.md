# How to Run Things on the Field Computer

This guide assumes you know what a terminal is and how to cd (change
directories). Otherwise, everything is explicitly laid out. Note: when the
instructions say "new terminal", a new tab will also do.

## Launching Soccer

In a new terminal, run the following lines one-by-one: ``` source
/opt/ros/foxy/setup.bash make perf ros2 launch rj_robocup soccer.launch.py
source install/setup.bash ```

This should open soccer, good for testing radio connection. If you want to run
gameplay, you have to start vision (keep scrolling).

## Manual Control

In a new terminal:

1. cd to robocup-software
2. launch manual.launch.py with the following commands: ``` source
   /opt/ros/foxy/setup.bash make perf ros2 launch rj_robocup manual.launch.py
   source install/setup.bash ```

In a new terminal:

1. cd to robocup-software
2. launch rqt: `rqt`

Then, within rqt:
1. Go to Plugins > Service Caller
2. Select `/list_joystick`, and note the string given for keyboard or
   controller input (e.g. `'keyboard-controller'` for kb)
3. Select `/select_manual`, set the desired `robot_id`, set `controller_uuid`
   to the string found in the last step, set `connect` to `True`
4. Hit call.

Keep focus on the black window (as in, you should click on that window if you
can't see it). 

WASD to move the robot, QE to pivot, K to kick and C to chip.

## Starting Vision

Assuming you have soccer running (see top section), you need the
league-provided vision software to give our gameplay system what it needs to
work. 

In a new terminal: ``` cd ~/ssl-vision ./bin/vision ```

This should pull up a GUI of the ssl-vision system. In said GUI, start capture
with:

`Thread 0 -> Image Capture -> Capture Control -> Start Capture`

Then push `publish`. 

(If you can't see these buttons, try scrolling to the right with the bar on
bottom.)

If our system isn't picking up on vision's input, try:
1. restarting soccer
2. launching the graphical client (`cd ~/ssl-vision && ./bin/graphicalClient`)

If the graphical client doesn't show any robots, vision is broken and something
is terribly wrong. Give an experienced software member a shout.

