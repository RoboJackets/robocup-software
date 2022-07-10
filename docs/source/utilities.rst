Utilities
===================================================
This page will contain high-level documentation on the various utilities in RJ RoboCup Software.
Utilities will be classified as anything that is not directly run after you launch soccer.

Sourcing ros2
---------------------------------------------------
Run ``./source.bash`` to automatically detect whether your shell is using zsh or bash and source that respective setup file (they are located in ``/install``).
Additionally, we would reccomend adding ``source /opt/ros/foxy/setup.zsh`` to your ``.zshrc`` if using zsh or to your ``.bashrc``. So you don't have to type that out each time you open a new terminal. When in the future we transition to a different ROS version, you can update that line to the path of the new version.

Launch
--------------------------------------------------
Our launch files are pretty similar to the ones introduced in ROS wiki tutorials, 
but we also use flags on launch to easily switch between certain nodes 
(for example we can switch between sim radio and network radio by setting a particular to the proper boolean value).
Alias for common launch configurations can be found in the makefile.

Build System
--------------------------------------------------
We use cmake and ninja. We prefer using clang as the C++ compiler compared to gcc, but either will work. 
Most of the details on the high-level construction of out build system can been ascertained by reading the root ``CMakeLists.txt``. 
If you are adding a new C++ file, it is best to just follow the existing format by reading through ``CMakeLists.txt`` in the relevant directories.

Debugging C++ Code
--------------------------------------------------


Continous Integration 
--------------------------------------------------
