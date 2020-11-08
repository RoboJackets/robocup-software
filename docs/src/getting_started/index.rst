Getting Started
===============

As a first step, let's run the stack on the GrSim_ simulation.

.. _GrSim: https://github.com/RoboCup-SSL/grSim

Make sure you have sourced the ROS stack if you haven't already:

.. code-block:: sh

    source /opt/ros/foxy/setup.sh

Afterwards, we need to source our loca setup. Run the following in the **robocup-software** directory:

.. code-block:: sh

    source install/setup.sh

If you're on zsh, source the ``.zsh`` version instead:

.. code-block:: sh

    source install/setup.zsh

Now we are good to go. As a sanity check, the following command should print out ``rj_robocup``:

.. code-block:: sh

    ros2 pkg list | grep rj_robocup

To run sim, run the following:

.. code-block:: sh

    ros2 launch rj_robocup sim.launch.py

If everything is working properly, you should see the following window show up.

.. image:: soccer.png
