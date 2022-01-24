Getting Started
===============

If you have a clean Ubuntu VM, run the following to install all required dependencies first:

.. code-block:: bash

    ./util/ubuntu-setup

Then, let's run the stack on the GrSim_ simulation.

.. _GrSim: https://github.com/RoboCup-SSL/grSim

Make sure you have sourced the ROS stack if you haven't already:

.. code-block:: bash

    source /opt/ros/foxy/setup.bash

If you're on zsh, source the ``.zsh`` version instead:

.. code-block:: sh

    source /opt/ros/foxy/setup.zsh

Then build the codebase (this compiles all of our code) with:

.. code-block:: bash

   make perf

(This step will take upwards of 15 minutes on a VM.)

Afterwards, we need to source our local setup. Run the following in the **robocup-software** directory:

.. code-block:: bash

    source install/setup.bash

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
