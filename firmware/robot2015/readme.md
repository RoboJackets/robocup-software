
# Robot Firmware 2015

The files in this folder provide all of the code that is run on the robot itself, including the MBED microcontroller C++ code and the Verilog code for the FPGA.


## FPGA

Spartan 3E XC3S100E 4C

In order to synthesize/compile the vhdl code into a a binary for fpga configuration, you must download the xilinx tools.  Currently Xilinx is moving away from its old sdk called "ISE Design Tools" to the new one called "Vivado Design Tools".  We're currently using an older fpga (Spartan 3E series), so we have to stick with the old dev tools.  To download, you'll have to sign up for a free account on xilinx.com and download the Web Edition of ISE Design Tools, which is free.  Go to Downloads -> ISE Design Tools -> Full Installer For Linux.  This is a large (~6GB) file and will take a while to download.  When the installer prompts you, choose "ISE WebPack", which is the free version.  Afterwards, you'll have to setup licensing.  There's a signup/request for the license on xilinx.com, then they'll email you a .lic file that you copy into ~/.Xilinx.


### Configuring via MBED

The FPGA can be configured at "bootup" in a variety of ways.  Our previous generation of robots contained a small flash memory chip connected to the FPGA via SPI, which held the configuration data.  When the control board turned on, it would tell the FPGA to configure itself from flash.  Our new design eliminates the need for the flash memory chip and lets the MBED configure the FPGA via "Slave Serial Mode".  See the FPGA Configuration Guide (~page 30) for more info.


#### Slave Serial Mode

External clock signal applied on CCLK pin

As the fpga is configured, it calculates a CRC.  If the CRC doesn't match the one supplied at the end of the bitstream, it pulls INIT_B low and aborts configuration. (see page 41)


Most of the configuration info is starts on page 49.  Below is a summary of the important points:

* The mode select pins M[2:0] define the configuration mode to be used.
* The DONE pin, when high, indicates successful configuration completion.
* The program pin PROG_B initiates the configuration process.  FPGA automatically initiates configuration on power-up??
* The config clock pin CCLK is used during configuration.  If configuring in slave mode, you have to provide the clock.  If in master mode, it generates its own clock.
* The INIT_B pins do several things.  At the start of configuration, INIT_B goes Low indicating that it is "housecleaning", meaning it's clearing its internal configuration memory.  Later on, if INIT_B goes low, it indicates a CRC check failure.
* Three different slave modes for configuration: serial, parallel, and JTAG.  Parallel would probably be fastest since it sends 8 bits at a a time, but we don't have that many spare pins.  Serial is the simplest and will work fine, so we're going with that.

* for Slave Serial Mode, set M[2:0] to 111.

* If M[2:0] pins are unconnected and HSWAP is pulled low, then it defaults to Slave-Serial configuration mode

* The DONE pin is actively driven Low during configuration.  There's a bitstream generation option to define the behavior of this pin after configuration is done.  NOTE: may want to look into this - the default DOESNT actively drive the pin high after configuration.



* Some of the pins used for configuration become general-purpose I/O pins after configuration is done and we use this for SPI.  SPI is implemented in verilog.


* IMPORTANT: see page 58 for info on the CCLK requirements/suggestions


