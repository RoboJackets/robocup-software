`include "phase_driver.vh"

module Phase_Driver(clock, duty_cycle, high_z, pwm_high, pwm_low);

input clock;
input [`DUTY_CYCLE_WIDTH - 1 : 0] duty_cycle;
input high_z;

output pwm_high, pwm_low;
reg pwm_high = 0, pwm_low = 0;
wire h, l;

PWM pwm_high_driver (clock, duty_cycle, h);
PWM_LOW pwm_low_driver (clock, duty_cycle, l);

always @ (clock)
begin
	pwm_high = (high_z == 1) ? 0 : h;
	pwm_low = (high_z == 1) ? 0 : l;
end






endmodule