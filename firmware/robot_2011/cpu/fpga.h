#pragma once

// Initializes and tests the FPGA.
// This does not force the FPGA to reconfigure, but waits for it to finish.
//
// Returns 0 on failure, 1 on successful configuration, or 2 if the FPGA was already configured.
int fpga_init(void);

void fpga_update(void);
