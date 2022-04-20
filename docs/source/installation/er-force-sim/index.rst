ER-Force Simulation Installation
============

Download ER-Force Sim
----------------------

First git clone the ER-Force Simulator from the following repo:

.. code-block:: sh

   git clone https://github.com/robotics-erlangen/framework.git

Change directory into the recently cloned repo

Building the framework
----------------------

Run the following code line by line
.. code-block:: sh
    mkdir build && cd build
    cmake ..
    make simulator-cli
This builds an executable in framework/build/bin

Setting Internal Referee Executable to External Referee
--------------------------------------------------------

Go to robocup-software/launch/soccer.launch.py

Go to the section that states
.. code-block:: sh
    ref_receiver = Node(
        package="rj_robocup",
        executable="internal_referee_node",
        output="screen",
        parameters=[config],
        on_exit=Shutdown(),
    )
Change the "exceutable" line to 

.. code-block:: sh
    executable="external_referee_node",

Commenting Out GrSim
--------------------
Got to robocup-software/launch/sim.launch.py

Go to the section that states
.. code-block:: sh
        return LaunchDescription(
        [
            stdout_linebuf_envvar,
            DeclareLaunchArgument("team_flag", default_value="-y"),
            DeclareLaunchArgument("ref_flag", default_value="-noref"),
            DeclareLaunchArgument("headless_flag", default_value=""),
            DeclareLaunchArgument("direction_flag", default_value="plus"),
            stdout_linebuf_envvar,
            grsim,
            soccer,
        ]
    )
Comment out the line that says "grsim"



