`include "Phase_Driver.vh"

module PWM(clock, duty_cycle, pwm);

	input clock;
	input [`DUTY_CYCLE_WIDTH - 1:0] duty_cycle;
	output pwm;
	reg pwm = 0;

reg [`COUNTER_WIDTH - 1:0] counter = 0;

always @ (posedge clock)
begin
	pwm = (counter < duty_cycle*`DUTY_CYCLE_STEP_RES) ? 1 : 0;
	counter = counter + 1; 
	if (counter >= `MAX_COUNTER) counter = 0;
end
endmodule
