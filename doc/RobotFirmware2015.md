
# Robot Firmware 2015

%Robot firmware is anything that runs on the robot itself, rather than on the field computer.  It is composed of two main parts: VeriLog HDL code for the FPGA and C++ code that runs on the microprocessor (MBED LPC1768).  The microprocessor handles most of the logic that drives the robots and the FPGA is the bridge that the microprocessor uses to communicate with some of the hardware components.


## Startup

TBD


## Hardware

The firmware's job is to accept instructions over the radio and carry them out by controlling the hardware.  Here's a list of the hardware onboard:

* TBD


## Console

A great way to debug a robot or just to query it for information is to use the console.  This is a command-prompt that you can use when the robot is connected via USB.  We have [udev](http://en.wikipedia.org/wiki/Udev) rules setup so that when the robot is connected via USB, it will show up in the filesystem as `/dev/mbed0`.  You can use the [screen](http://en.wikipedia.org/wiki/GNU_Screen) program in the terminal to connect to it by typing:

~~~~{.sh}
sudo screen /dev/mbed0
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
