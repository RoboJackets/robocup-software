# robot firmware

This folder contains the firmware that runs on the robots themselves.  The cpu/ folder contains the c code that runs on the ARM microprocessor and the fpga/ folder contains the verilog code for the on-board fpga.


## Building and Installing

There are two main ways to load the firmware onto the robot: over USB and over the radio.  If the robot already has working firmware and you'd like to load a new version, you can use the regular USB install.  Otherwise, you'll have to program it using SAMBA


### Over-The-Air Install

Turn the robot on so that its radio can communicate with the radio base station connected to your computer.  Then in the software directory, use scons to begin the update:

```
$ scons robot-ota
```

The computer will then rebuild the firmware if necessary, then load it onto all robots that are turned on and within radio range.


### Regular USB Install

1. Turn on the robot and plug it into your computer with a micro USB cord
2. Run `$ sudo scons robot-prog`


### USB SAMBA Install

1. Turn on the robot and plug it into your computer with a micro USB cord
1. Set DIP switch 3 on the robot to ON
1. Wait 10 seconds
1. Turn DIP switch 3 off
1. Turn the robot off and then back on again
1. Run `$ sudo scons robot-prog-samba` to load the firmware
