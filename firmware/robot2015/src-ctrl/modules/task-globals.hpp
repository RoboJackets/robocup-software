#pragma once

// Global variables shared between task threads for error flagging.
// The LSB of each halfword indicates where the error code is valid or not.

extern uint16_t comm_err;
extern uint16_t fpga_err;
extern uint16_t imu_err;
