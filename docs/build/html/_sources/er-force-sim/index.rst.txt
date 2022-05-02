ER-Force Simulation Installation
================================

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

Running the Simulation
----------------------

Open a new terminal window and change directory into framework/build/bin

In this folder, the file named "simulator-cli"

Run this file by doing the following

.. code-block:: sh

    ./simulator-cli

This will run the ER-Force Simulator to test your code

Then, source the ROS setup file and source your local setup.