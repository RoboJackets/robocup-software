Utilities
===================================================
This page will contain high-level documentation on the various utilities in
RJ RoboCup Software.
Utilities will be classified as anything that is not directly run after you
launch soccer.

Sourcing ros2
---------------------------------------------------
Run ``./source.bash`` to automatically detect whether your shell is using zsh
or bash and source that respective setup file (they are located in ``/install``).
Additionally, we would recommend adding ``source /opt/ros/foxy/setup.zsh`` to
your ``.zshrc`` if using zsh or to your ``.bashrc`` so you don't have to
type that out each time you open a new terminal. When in the future we
transition to a different ROS version, you can update that line to the path
of the new version.

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
As of summer 2022, the ubuntu-setup script installs clang-10 using a script
which also includes lldb-10
(so you should not need to install anything new for this).
All you need to do is start soccer or sim in one terminal tab, and in another
tab, run ``sudo lldb-10 -n name_of_particular_proc``.

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

There is also another method which involves commenting out the node you want
to run in the launch files and running it separately in a new terminal tab
with a debugging prefix. This method is slower and not as consistent, so I
won't explain it here. Check out ``util/debug-cpp.sh`` if you
are curious, or if the first method I explained stops working.

Continuous Integration
--------------------------------------------------
We use github actions. The configuration for that can be found in ``.github/workflows``.
