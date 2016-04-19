/*
 *  This Verilog header file is where you can define or comment out areas
 *  during FPGA synthesis.
 */

`ifndef _ROBOCUP_HEADER_
`define _ROBOCUP_HEADER_

/**
 * Enable/Disable the module for the dribbler motor.
 */
`define DRIBBLER_MOTOR_DISABLE

`ifdef __ICARUS__
`undef DRIBBLER_MOTOR_DISABLE
`endif

`endif  // _ROBOCUP_HEADER_
