Field Computer Operation
========================
This guide assumes you know basic terminal usage (how to cd, run commands,
etc).  Otherwise, everything is explicitly laid out. Note: when the
instructions say "new terminal", a new tab will also do.

In a new terminal, run ``cd ~/robocup-software``, then run ``git checkout ros2``.

Manual Control
--------------

In a new terminal:

1. ``cd ~/robocup-software``
2. Source ROS2 and our build files:

.. code-block::

    . ./source.bash

3. Launch soccer in manual control mode

.. code-block::

    make run-manual

In a second new terminal:

1. ``cd ~/robocup-software``
2. Source everything (see above)
3. Run the utility script to connect a controller to a robot:

.. code-block::

    ./util/manual_control_connect.bash

This will output a help message. In nearly every case you will only have to
change the robot ID, like so:

.. code-block::

    ./util/manual_control_connect.bash -r 1

Other options for args can be found in the source code: ``soccer/src/soccer/joystick/``.

After you run this utility script, you will see printed confirmation in the
first terminal where you launched soccer.

For keyboard control, keep focus on the small black window (as in, you should
click on that window if you can't see it).
 - WASD to move
 - QE to pivot
 - K to kick
 - J to chip
 - Spacebar to dribble

Launching Soccer
----------------

In a new terminal, run the following lines one-by-one:

.. code-block::

    source /opt/ros/foxy/setup.bash
    make perf
    source install/setup.bash
    ros2 launch rj_robocup soccer.launch.py

This should open soccer, good for testing radio connection. If you want to run
gameplay, you have to start autoref and vision (keep scrolling).

Open Ref
--------

In a new terminal:
1. ``cd ~/ssl-game-controller``
2. ``./ssl-game-controller_v2.12.7_linux_amd64`` (Just hit tab after typing ssl)
3. Click open link on the line that states ``UI is available at http://...``
4. Make sure at least one team is named RoboJackets. If not, click the top left
   gear, click pen next to one of the names and find RoboJackets in the
   dropdown.

Starting Vision
---------------

Assuming you have soccer running (see top section), you need the
league-provided vision software to give our gameplay system what it needs to
work.

In a new terminal: ``cd ~/ssl-vision`` ``./bin/vision``

This should pull up a GUI of the ssl-vision system. In said GUI, start capture
by going to the left sidebar and clicking the arrows to dropdown:

**Thread 0 -> Image Capture -> Capture Control -> Start Capture**

Then push the ``start`` button. (If you can't see this button, try scrolling to the right with the bar on bottom.)
Do the same for Thread 1 for full-field coverage.

If our system isn't picking up on vision's input, try:
1. restarting soccer
2. launching the graphical client (``cd ~/ssl-vision && ./bin/graphicalClient``)

If the graphical client doesn't show any robots, vision is broken and something
is terribly wrong. Give an experienced software member a shout.

If you are the experienced member or there is not an experienced member to help, you can try using [this guide](https://github.com/RoboJackets/robocup-computer-config/blob/main/ssl-vision/config-howto.md) in our computer-config repo to fix vision.

