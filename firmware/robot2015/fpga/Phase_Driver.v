`include "phase_driver.vh"
//`include "PWM.v"  Used to be seperate files, now accomplished in here
//`include "PWM_LOW.v" Used to be seperate files, now accomplised in here

module Phase_Driver(clock, duty_cycle, high_z, pwm_high, pwm_low);

input clock;
input [`DUTY_CYCLE_WIDTH - 1 : 0] duty_cycle;
input high_z;

output pwm_high, pwm_low;
wire pwm_high, pwm_low;
wire h, l;

reg [`COUNTER_WIDTH - 1:0] counter = 0;

assign h = (counter + `DEAD_TIME < duty_cycle*`DUTY_CYCLE_STEP_RES) ? 1 : 0;
assign l = (counter >= duty_cycle*`DUTY_CYCLE_STEP_RES && counter + `DEAD_TIME < `MAX_COUNTER) ? 1 : 0;


assign  pwm_high = (high_z == 1) ? 0 :
		(duty_cycle == `MAX_DUTY_CYCLE) ? 1 :
		(duty_cycle == 0) ? 0 : h;
assign	pwm_low = (high_z == 1) ? 0 :
		(duty_cycle ==`MAX_DUTY_CYCLE) ? 0 :
		(duty_cycle == 0) ? 1 : l;
//PWM pwm_high_driver (clock, duty_cycle, h);
//PWM_LOW pwm_low_driver (clock, duty_cycle, l);

always @(posedge clock)
begin
	counter = counter + 1; 
	if (counter >= `MAX_COUNTER) counter = 0;
end


endmodule