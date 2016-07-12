# Robot Firmware

This folder contains the firmware that runs on the robots themselves.  The [`./cpu`](./cpu) folder contains the C code that runs on the [ARM7TDMI](http://en.wikipedia.org/wiki/ARM7) microcontroller ([Atmel AT91SAM7S256](http://www.atmel.com/devices/sam7s256.aspx)) and the [`./fpga`](./fpga) folder contains the [Verilog](http://en.wikipedia.org/wiki/Verilog) code for the Xilinx [Spartan-3E Series](http://www.xilinx.com/support/index.html/content/xilinx/en/supportNav/silicon_devices/fpga/spartan-3e.html) field-programmable gate array ([FPGA](http://en.wikipedia.org/wiki/Field-programmable_gate_array)).


## Building and Installing

There are two main ways to load the firmware onto the robot: over USB and over the radio.  If the robot already has working firmware and you'd like to load a new version, you can use the regular USB install.  Otherwise, you'll have to program it using SAMBA.


### Over-the-Air Install

Turn the robot on so that its radio can communicate with the radio base station connected to your computer.  Then in the software directory, use `make` to begin the update:

```
$ make robot-ota
```

The computer will then rebuild the firmware if necessary, then load it onto all robots that are turned on and within radio range.


### Regular USB Install

1. Turn on the robot and plug it into your computer with a mini USB cord
2. Run `$ make robot-prog`


### USB SAMBA Install

1. Turn on the robot and plug it into your computer with a mini USB cord
1. Set DIP switch 3 on the robot to ON
1. Wait 10 seconds
1. Turn DIP switch 3 off
1. Turn the robot off and then back on again (make sure all LEDs have time to turn off)
1. Run `$ make robot-prog-samba` to load the firmware
