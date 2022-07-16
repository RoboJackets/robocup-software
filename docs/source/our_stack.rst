Our Stack
=========

This is a brief overview of our codebase. Since the codebase is constantly
changing, expect that some or many parts of these docs will be out-of-date.
The most up-to-date reference on our codebase is our code itself.

.. note::

    To see when any documentation page was last updated, click "Edit on GitHub" in
    the top right, then see the timestamp on the last commit.

Gameplay
--------
Gameplay is where our strategy is produced. It takes in the current state and
assigns all robots a task based on some coordination. Currently, we have a
system with a play selector that chooses a play based on world state. Then
that play assigns roles, roles select a tactic based on their progress, and
tactic select an atomic skill to perform. The gameplay loop is set to tick at
60Hz. The below code block is how currently that is done. Each time the timer
ends, the callback method is entered:

.. code-block:: python

        timer_period = 1 / 60  # seconds
        self.timer = self.create_timer(timer_period, self.gameplay_tick)

Motion Planning
---------------
Motion planning is about generating a path from a robots current position and
orientation (pose) to a goal supplied by gameplay. Starting at the lowest
level, we use RRT for path generation. This occurs after trying to apply a
straight line path to the goal and that is unsuccessful (if there are no
obstacles in that straight line why bother with RRT?). We have our own RRT
implementation in `a separate repository <https://github
.com/RoboJackets/rrt>`_ which we use as a submodule. This
way others can easily use our RRT implementation if they want.

But we can't send a path to a robot and expect it to figure it out. We need
to give our firmware a single velocity command for that instant of planning.
The file ``soccer/src/soccer/planning/primitives/velocity_profiling.cpp``
finds that velocity trajectory which we can then return as the current
trajectory as the robot.

At the highest level of planning, we have several different planners which the
planning_node selects based on what kind of skill gameplay requested. The
have different behavior and based on the their names you can kind of get an
idea of what each would be used for. They are all located
in ``soccer/src/soccer/planning/planner``.

This is not an extensive explanation of everything planning does, looking
through the ``planning`` folder will show you everything else, but it's a
good start.

Motion Control
--------------
A lot of the motion control is done on robot by the firmware, but part of the
motion control is handled by software. See the ``control`` directory,
``motion_control.cpp`` uses PID to correct robot position. The PID
controller implementation is not in software, but it is in the
`fshare repository <https://github.com/RoboJackets/robocup-fshare>`_.
Correction of robot velocity is done by robot firmware (the code that runs on
each robot's microcontroller) in the separate firmware repository.

Processor
---------
It is defined in ``soccer/src/soccer/processor.cpp``, and it basically just
starts a bunch of temp nodes and then has an infinite loop to update the
context. Context is used to update the ui. It is good that most other nodes
are not dependent on this loop.

Radio
-----
See the ``/radio`` directory. We have a sim radio for when playing virtual
matches or testing locally and a network radio for when we run on real
hardware. It sends commands to our robots using the league designated ports
for yellow and blue when in sim mode. The simulation packets are standardized
by the league and in the ``rj_protos`` directory (the ones with the ssl
prefix).

The network radio expects each robot to know the ip address of where to send
to. When soccer eventually receives a robot status message from a robot, it
adds the robot id to an ip map. Now our software knows how to reach that
robot id. If that robot doesn't send any robot status in a while, it will
disconnect and disappear from the UI (when running with real robots).

Referee
-------
Referee controls the match state. We have an internal referee which can send
quick commands like stop and start from our UI, so we don't always have to
launch the game controller. When we do want to use the game controller
though, the external referee handles that by listening for game events and
updates on specific ports designated by the league.

UI
--
We use Qt for UI development. Code for it is located in ``soccer/src/soccer/ui``.
The ``qt`` directory contains the view as .ui files and images that are
placed on the view.
You could directly edit these .ui files based on their
proper syntax to change our view, but I wouldn't recommend it.
Instead install Qt designer and open .ui files using it to save yourself a
lot of time.

After the view, all the C++ files act as the view-model for each view.
Some of them are quite messy and do not follow our naming conventions,
but if you follow the pattern of other methods, then you should be able to
add new functionality to any part of our ui.

Vision
------
There are two parts to vision, both equally important: receiver and filter.

First, receiver gets frames from the simulator or camera. It does some
updating, then sends that to the filter. Filter uses a kalman filter to
estimate the current world state and then publishes that as a built world
state message which is then published for the rest of codebase to use:

.. code-block:: c++

        VisionFilter::WorldStateMsg VisionFilter::build_world_state_msg(bool
us_blue) const {
        return rj_msgs::build<WorldStateMsg>()
        .last_update_time(rj_convert::convert_to_ros(\world_
        .last_update_time()))
        .their_robots(build_robot_state_msgs(!us_blue))
        .our_robots(build_robot_state_msgs(us_blue))
        .ball(build_ball_state_msg());
        }
