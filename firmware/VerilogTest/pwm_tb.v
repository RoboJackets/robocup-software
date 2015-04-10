//`timescale 1ns/100ps //timescale / operation precision /**Borked

`include "phase_driver.vh"

module pwm_tb;

/*Output values to 'test.vcd' file */
initial
 begin
    $dumpfile("test.vcd");
    $dumpvars(0,pwm_tb);
 end

/*inputs to DUT*/
 reg [`DUTY_CYCLE_WIDTH - 1:0] duty_cycle;
 reg clock;
 reg high_z = 0;

/*outputs from DUT*/
 wire pwmH, pwmL, low_mod;



initial begin
	
	duty_cycle = 8'h10; //dutycycle
	clock = 0;

end





 always begin
 	 #1 clock = !clock;
 end
 always begin
 	 #50000 high_z = 1;
 	 #100000 $finish;
 end
/* instantiate PWM high and low */


Phase_Driver phasedriver(clock, duty_cycle, high_z, pmwH, pwmL);

 endmodule