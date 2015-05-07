`ifndef _phase_driver_vh_
`define _phase_driver_vh_

`define DEAD_TIME 2 			//dead time in units of clock ticks

`define COUNTER_WIDTH 10 		//bits available to counter
`define MAX_COUNTER 10'h3FF 		//PWM period = MAX_COUNTER * clockPeriod 

`define DUTY_CYCLE_WIDTH 10 		//bits available to duty_cycle
`define MAX_DUTY_CYCLE 10'h3FF	//Value represeting a duty cycle of 100%
`define DUTY_CYCLE_STEP_RES 1 	//ceil(MAX_COUNTER/MAX_DUTY_CYCLE) 

`endif