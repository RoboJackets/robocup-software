# 2015 Firmware: The FPGA

The [`src`](./src) directory contains the Verilog source files. When synthesized, these files generate a bitfile that will tell the FPGA how to configure transistors in the circuit after a robot is turned on.

The [`sim`](./sim) directory contains simulation files. Simulations help us determine if real-world results will perform how we expect them to without loading anything into hardware (or if there's no FPGA nearby...).


# Synthesizing the Verilog

Using a machine configured with the correct Xilinx tools for systhesis, running the `make fpga2015` command will start the synthesis process. The `make fpga2015-prog` command will synthesize everything before uploading it onto a connected mbed.
