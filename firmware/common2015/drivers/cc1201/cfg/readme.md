
# CC1201 Configs

This directory contains config files for the cc1201 radio.  The easiest way to configure the cc1201 is to use TI's SmartRF Studio.  It allows you to use a gui to set register values and easily see their effects and test them on an actual radio with the cc debugger.


## Config format

Save SmartRF xml config files in this directory and select which one to use by changing the CC1201_CONFIG variable in common2015/CMakeLists.txt.  At compile time, a python script converts this xml file into a C header file containing the register values, which is used to configure the cc1201 in firmware.
