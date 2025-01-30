Installation
============

.. note::

   It's been mentioned (as of 5/17/2022) that this section is no longer
   up-to-date. Use these instructions as a rough guide.

Codebase
--------

CLion Setup
-----------
We strongly recommend that users use CLion_, which Jetbrains offers for free through their university programs.
Apply for it here__.

.. __: https://www.jetbrains.com/shop/eform/students
.. _CLion: https://www.jetbrains.com/clion/

Getting CLion
^^^^^^^^^^^^^
Download the `Jetbrains toolbox`_. From there, install the latest version of CLion.

.. _Jetbrains toolbox: https://www.jetbrains.com/toolbox-app/

Opening the project in CLion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Open up CLion. Click **Open or Import**, and then select the **robocup-software** folder.
An editor should pop up, with the **cmake** tab showing a bunch of errors. Don’t worry, this to be expected.

To fix this, we need to tell CLion about where ROS is.

Go to **File -> Settings -> Build, Execution, Deployment -> CMake -> Environment** and paste the below in that textbox::

    AMENT_PREFIX_PATH=/opt/ros/foxy;AMENT_CURRENT_PATH=/opt/ros/foxy;ROS_DISTRO=foxy;ROS_VERSION=2;ROS_PYTHON_VERSION=3;PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages;CMAKE_PREFIX_PATH=/opt/ros/foxy;PATH=/opt/ros/foxy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

Next, on the same page, paste the below in the **CMake Options** textbox::

    -DCMAKE_INSTALL_PREFIX=../install

Close the settings menu, and click the **CMake** tab on the bottom.
On the left, there should be a “refresh” symbol. There should now be no errors.
