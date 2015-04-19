`ifndef _phase_driver_vh_
`define _phase_driver_vh_

`define DEAD_TIME 1 			//dead time in units of clock ticks

`define COUNTER_WIDTH 8 		//bits available to counter
`define MAX_COUNTER 8'hFF 		//PWM period = MAX_COUNTER * clockPeriod 

`define DUTY_CYCLE_WIDTH 8 		//bits available to duty_cycle
`define MAX_DUTY_CYCLE 8'hFF	//Vaule represeting a duty cycle of 100%
`define DUTY_CYCLE_STEP_RES 1 	//ceil(MAX_COUNTER/(2^(duty cycle width)-1))

`endif //_phase_driver_vh_