Getting Started
===============

Installation
------------

We only provide official support for Ubuntu 20.04 due to ROS2. Make sure you
are on an Ubuntu 20.04 machine before continuing.

First, clone the repository from GitHub:

.. code-block:: bash

   git clone git://github.com/RoboJackets/robocup-software

Then change directories (``cd``) to the repository you just cloned, and run the
setup script to install all required dependencies:

.. code-block:: bash

    cd robocup-software
    ./util/ubuntu-setup
    
And setup git submodules:

.. code-block:: bash

    git submodule init
    git submodule update
    

Running The First Time
----------------------

Let's run the stack! We'll use the ER-Force simulator to simulate the vision
data we'd get from a real field camera setup. (Read the "Welcome" page if that
sentence made no sense to you.)

TODO(Prahbanjan): add instructions for ER-force sim launch here

First, make sure you're on the most updated version of ``ros2`` branch. This is
where the latest working version of our codebase exists. (See "Github" doc.
--TODO(Kevin): transfer this to docs--)

.. code-block:: bash

    git pull
    git checkout ros2

Then, source the ROS setup file. This ensures your shell can use ROS commands.

.. code-block:: bash

    source /opt/ros/foxy/setup.bash

If you're on zsh, source the ``.zsh`` version instead. (If you don't know what
zsh is, you're not on zsh.)

.. code-block:: bash

    source /opt/ros/foxy/setup.zsh

Then build the codebase. This compiles all of our code.

.. code-block:: bash

   make perf

(This step will take upwards of 15 minutes on a VM.)

After building, we need to source our local setup. Run the following in the
``robocup-software`` directory:

.. code-block:: bash

    source install/setup.bash

Again, if you're on zsh, source the ``.zsh`` version instead:

.. code-block:: bash

    source install/setup.zsh

Now we are good to go. As a sanity check, the following command should print
out ``rj_robocup``:

.. code-block:: bash

    ros2 pkg list | grep rj_robocup

To run sim, run the following:

.. code-block:: bash

    ros2 launch rj_robocup sim.launch.py

If everything is working properly, you should see the following window show up.

.. image:: soccer.png


Running Again
-------------

Now that you know how to source dependencies, build, and run our code, you can
take advantage of some neat shortcuts. These shortcuts all depend on the
following knowledge:

Sourcing only needs to happen every time a new terminal is opened, and
building only needs to happen when C++ or launch.py files are changed. 

(Python is not a compiled language; thus, those files do not need to be built
when changed.)

So, after you've built once, the ``install/setup.bash`` script will exist in
your version of the repo, and you won't have to build again until you make
changes to C++ or launch files. That means the next time you open up a new
terminal, you can launch sim with:

.. code-block:: bash

   . ./source.bash
   make run-sim

``source.bash`` is an alias for the two source commands in the above section,
and ``make run-sim`` is an alias for ``ros2 launch rj_robocup sim.launch.py``. 

Since Python is not compiled, if you're exclusively working on Python files,
and staying in one terminal, it's likely that you'll mostly only need:

.. code-block:: bash

   make run-sim

However, if you're working on C++ or launch files (or if someone else changes
them on a different branch and you want their changes), you'll need to build
again. If you've already built once, though, you can build again more quickly
with:

.. code-block:: bash

   make again
   . ./source.bash

The ``source.bash`` line is necessary to source the file in ``install/``, which
is refreshed on each build. (Note: this does not build any CMake-related files,
so if you're editing those, use ``make perf`` as usual.)

There are a few different ways to build our code. See the makefile for more
details, but in short:

.. code-block:: bash

   make all         # builds with full debugging symbols
   make debug       # alias for make all
   make all-release # builds with 0 debugging symbols
   make perf        # builds with some debugging symbols; preferred method

TODO(Kevin): add description of running on field comp (move that md file over too)


Troubleshooting
---------------

The most common issue when running our code is forgetting to source after
opening a new terminal. For that, run the source commands in "Running the First
Time" above, or use the shortcut

.. code-block:: bash

   . ./source.bash

The next most common issue is forgetting to build after changing a C++ file or
launch script. Resolve that by building to see your changes. Again, the default
is

.. code-block:: bash

   make perf

If ``make perf`` fails, your build system may be corrupted. Clean out your
build files with

.. code-block:: bash

   make clean

then build again. (Note: you will have to manually re-source the install/ file
after building: see above.)

If ``make clean`` fails, it may be something wrong with your branch or machine.
Checkout ros2 (the default branch) and try building there. If that succeeds,
your machine is fine, so the problem must be with your branch's code.

If ros2 fails to build and run, there is an issue with your machine, since
ros2's build-ability is checked by GitHub Actions on every commit. Try re-doing
the steps in Installation (above) to make sure your machine has all the
required dependencies. If it outputs any errors when you do so, Google them!

As a final sanity check, if you've tried all of the above, delete your copy of
the robocup-software/ repo, and start over completely.

If you've gotten to this point and you're still unable to build and run the
stack, ask an older member.
