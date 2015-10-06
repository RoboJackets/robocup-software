
# Robot Firmware 2008, 2011

%Robot firmware is anything that runs on the robot itself, rather than on the field computer.  It is composed of two main parts: VeriLog HDL code for the FPGA and C code that runs on the microprocessor.  The microprocessor handles most of the logic that drives the robots and the FPGA is the bridge that the microprocessor uses to communicate with the hardware.

Both the 2008 and 2011 robot revisions run the same firmware.  A DIP switch on the control board is set to tell the firmware what revision it is running on so that it can handle the hardware differences at runtime.

The firmware for the new 2015 revision will run a new codebase that is documented separately.



## Startup

The firmware starts out by first initializing certain hardware components, such as setting up I/O pins, starting the system clock, etc.

It then plays the startup sounds to let us know that the board initialized without error.  There are also special sounds for dead battery and startup failure.

In many places you'll notice 'variables' being used that begin with 'AT91C_BASE_PIOA', which gives access to the I/O pins.


## Hardware

The firmware's job is to accept instructions over the radio and carry them out by controlling the hardware.  Here's a list of the hardware onboard:

* IMU
* Motor drivers - one for each wheel and one for the dribbler
* encoders
* [hall sensors](http://en.wikipedia.org/wiki/Hall_effect_sensor) (one per motor)
* kicker
* battery voltage meter (reading from an [adc](http://en.wikipedia.org/wiki/Analog-to-digital_converter))
* radio (see protocol information [here](https://github.com/RoboJackets/robocup-software/blob/master/doc/radio-protocol-2011.txt))
* USB
* LEDs
* Sounds / songs
* ball sensor
* Robot ID selector


## Console

A great way to debug a robot or just to query it for information is to use the console.  This is a command-prompt that you can use when the robot is connected via USB.  We have [udev](http://en.wikipedia.org/wiki/Udev) rules setup so that when the robot is connected via USB, it will show up in the filesystem as `/dev/robot`.  You can use the [screen](http://en.wikipedia.org/wiki/GNU_Screen) program in the terminal to connect to it by typing:

~~~~
sudo screen /dev/robot
~~~~

From there you can type any of the many available commands to make the robot do things or get info.  Type `help` to see a list of available commands.  As of writing this document, the list is:

~~~
help
status
reflash
reset
stfu
run
inputs
timers
fpga_reset
fpga_off
fpga_on
fpga_test
spi_test
spi_erase
spi_write
spi_read
radio_configure
radio_start
music
tone
fail
adc
i2c_read
i2c_write
monitor_faults
monitor_halls
read
rx_test
kicker_test
drive_mode
monitor_charge
imu_test
monitor_ball
~~~


## Sounds

The firmware plays certain sounds at startup or during operation to indicate status:

* [0-startup](0-startup.m4a)
* [1-failure](1-failure.m4a)
* [2-overvoltage](2-overvoltage.m4a)
* [3-undervoltage](3-undervoltage.m4a)
* [4-fuse-blown](4-fuse-blown.m4a)
* [5-fight-song](5-victory.m4a)
* [6-still-alive](6-still-alive.ogg)
