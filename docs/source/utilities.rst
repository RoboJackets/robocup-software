Utilities
===================================================
This page will contain high-level documentation on the various utilities for RJ
RoboCup Software. Utilities are everything not directly necessary to launch
our AI program, soccer (what a confusing name).

External Referee
--------------------------------------------------
First, read the Referee section of the Our Stack page and `this section
<https://robocup-ssl.github.io/ssl-rules/sslrules.html#_game_controller>`_ of
the rulebook. This will give you some background on what the SSL Game
Controller does. This program is given by the league and helps simulate what it
will be like at competition, where the (human) referee sits at a different
computer to the one that runs our software and gives game commands from there.

Installation is simple. First, create an empty directory named
``ssl-game-controller`` at the same level as your clone of
``robocup-software``:

~/coding/robocup/
├── robocup-software/
├── ssl-game-controller/

Then, download the latest release binary in the `SSL GC repo
<https://github.com/RoboCup-SSL/ssl-game-controller>`_ and put it into that
folder. Finally, make the release binary executable by ``cd``ing to the
``ssl-game-controller`` repo and running ``chmod +x <name of release binary>``.

When you want to launch the game controller, ``cd`` to your
``ssl-game-controller`` directory and run the release binary with ``./<name of
release binary>``. (You can tab-complete this by typing ``./`` and then hitting
tab.) The binary will output a message saying it has launched the UI at a
specific URL--click that link to open the UI. 

.. image:: ./_static/ssl-gc-ui.png

Operation instructions can be found in the FAQ of the SSL GC repo.

Launch
--------------------------------------------------
Our launch files are pretty similar to the ones introduced in ROS wiki
tutorials, but we also use flags on launch to easily switch between certain
nodes (for example we can switch between sim radio and network radio by
setting a particular to the proper boolean value).
Alias for common launch configurations can be found in the makefile.

Build System
--------------------------------------------------
We use cmake and ninja. We prefer using clang as the C++ compiler compared to
gcc, but either will work.
Most of the details on the high-level construction of our build system can
been ascertained by reading the root ``CMakeLists.txt``.
If you are adding a new C++ file, it is best to just follow the existing
format by reading through ``CMakeLists.txt`` in the relevant directories.

Debugging C++ Code
--------------------------------------------------
Setting up a debugger for our C++ side code is actually quite simple!
In particular, we have had success using `LLDB <https://lldb.llvm.org/>`_,
made by the same group that develops the clang compiler.
As of winter 2022, the ubuntu-setup script installs clang-10 using a script
which also includes lldb-10
(so you should not need to install anything new for this).
All you need to do is start soccer or sim in one terminal tab, and in another
tab, ``ros2 run --prefix 'lldb-10 run' rj_robocup executable_name``.

If that doesn't work for some reason, 
another way is to run the executable without the prefix 
(``ros2 run rj_robocup executable_name``). 
Then attach to this executable by running
``sudo lldb-10 -n name_of_particular_proc``.
This method is not as good as the previously described one, 
but I included it for completeness.

How do you find this particular name?
Well it depends on what file/node you wish to debug.
As of the time of writing this article, we do not use any ros2 node
composition, so each node is its own process.
Looking in the soccer launch file a node's "executable_name" corresponds to
the process name to place after ``-n``.
Another method to find the process names of nodes is to run ``top`` in a new
terminal tab and look in there.

**How do you use lldb?**
Google what you want to do and follow the top result or follow the lldb
tutorial on their website.
CS2110 and CS2200 will introduce you to gdb which is another debugger for
C/C++; if you already know that,
the commands are basically the same (the syntax is different in many places
though).

Continuous Integration
--------------------------------------------------
We use Github Actions. The configuration for that can be found in ``.github/workflows``.
