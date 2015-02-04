
# %Robot Firmware 2015

%Robot firmware is anything that runs on the robot itself, rather than on the field computer.  It is composed of two main parts: VeriLog HDL code for the FPGA and C code that runs on the microprocessor.  The microprocessor handles most of the logic that drives the robots and the FPGA is the bridge that the microprocessor uses to communicate with the hardware.

The 2015 robot firmware runs different firmware than the 2008 and 2011 version. A DIP switch on the control board is used to set various settings in the firmware. These settings are still being decided.


## Startup

TBD


## Hardware

The firmware's job is to accept instructions over the radio and carry them out by controlling the hardware.  Here's a list of the hardware onboard:

* TBD


## Console

A great way to debug a robot or just to query it for information is to use the console.  This is a command-prompt that you can use when the robot is connected via USB.  We have [udev](http://en.wikipedia.org/wiki/Udev) rules setup so that when the robot is connected via USB, it will show up in the filesystem as `/dev/robot`.  You can use the [screen](http://en.wikipedia.org/wiki/GNU_Screen) program in the terminal to connect to it by typing:

~~~~{.c}
sudo screen /dev/mbed
~~~~

From there you can type any of the many available commands to make the robot do things or get info.  Type `help` to see a list of available commands.

~~~
help
echo
alias
ping
reset
reboot
~~~


## Compiler Toolchain

The MBED requires a version of the arm compiler that is newer than the one provided by Ubuntu 14.04. We have not added this to the ubuntu-setup script, as we dont want to add more ppa's than we need to. If you need to compiler firmware, you can run

~~~~~{.c}
sudo apt-get remove binutils-arm-none-eabi gcc-arm-none-eabi
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi
~~~~~


## Sounds

The firmware plays certain sounds at startup or during operation to indicate status:

* [0-startup](0-startup.m4a)
* [1-failure](1-failure.m4a)
* [2-overvoltage](2-overvoltage.m4a)
* [3-undervoltage](3-undervoltage.m4a)
* [4-fuse-blown](4-fuse-blown.m4a)
* [5-fight-song](5-victory.m4a)
* [6-still-alive](6-still-alive.ogg)
