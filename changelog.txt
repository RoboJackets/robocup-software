Important changes were made when upgrading the stack to ROS2 Humble. A list
of changes and the reasoning behind them is listed below:

As of Nov 19, 2023:
1. The makefile was changed to work with the newer version of CMake
   that installs by default on Ubuntu 22.04. In particular, the 
   cmake commands no longer need the --target flag. The CMAKE_PREFIX_PATH
   variable at the beginning of the file has also been changed.

############## This change is important ###############
2. install/setup.bash - This has not been changed, but it is important
   to note that distutils is deprecated and slated for removal in 
   Python 3.12. There are no direct replacements for distutils, so
   a ticket should be opened ASAP to fix this. However, there is no
   immediate issue with leaving it as is because Ubuntu 22.04 comes with
   Python 3.10 by default. Do note that this is also the case in 
   install/setup.zsh.

3. source.bash - Source commands now reference humble and Ubuntu 22.04
   instead of foxy and Ubuntu 20.04.

4. rj_common/testing/rj_common_convert_test.cpp - the rclcpp::Duration
   class no longer accepts a single integer argument for milliseconds.
   Updated a line referencing this outdated constructor to use 
   a std::chrono::milliseconds instead.

5. rj_common/include/rj_common/time.hpp - Changed for similar reasons to (4)

6. rj_utils/src/logging.cpp - RCLCPP_DEBUG and similar macros accept 
   C strings now; updated calls to these macros

7. rj_config/CMakeLists.txt - added find_package(fmt), this change complements
   change number 6.

8. soccer/src/soccer/strategy/agent/agent_action_client.cpp and hpp - 
   Updated lines 211 to 219 to use lambda expressions instead of std::bind,
   Changed a method to take in a different parameter type. 
   Previously took a future template holding a GoalHandleRobotMove::SharedPtr,
   Now just takes in the GoalHandleRobotMove::SharedPtr.

9. soccer/src/soccer/ui/field_view.cpp - Added a preprocessor directive
   to include QPainterPath, which has been separated into its own namespace.
   Qt5 likely has other changes as well

10. soccer/src/soccer/ui/robot_status_widget.hpp - Added a preprocessor
    directive to include the std::optional namespace.

This changelog should be updated to reflect any further changes completed
before this upgrade is fully adopted.
