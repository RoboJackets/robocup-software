FAQ
===

General Troubleshooting
-----------------------

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

ER-Force Troubleshooting
------------------------

**What are the general steps to launching the simulation with an external game controller?**

1. Run ER-force simulator
2. Run game controller, open
3. Set team color to RoboJackets
4. Run soccer (our UI)

**Why is there no RoboJackets option in the team selection?**

You need to change the available ``teams`` in the ``config`` folder for the game controller. After running the game controller for the first time, it generates multiple files and a config folder. Once navigating to the folder that has the ssl-gamecontroller, select the config folder. Select the ``engine.yaml`` file, and add RoboJackets to the list of names.

.. note::

    The controller is case-sensitive while syncing with the soccer UI so the name MUST be spelled the same as shown.

**Why is the soccer UI glitching out?**

It could be possible that grSim is still running instead of the ER-Force simulator. Check to see if the grSim is running. If it is, then build the whole code base again by typing ``make perf``.

