//`include "phase_driver.vh"

/**
This module is responsible for driving one phase of the BLDC. It generates both
the low and high side PWM (Pulse Width Modulation) signals necessary for the
switching regulator.

The switching regulator, in this case, is two mosfets. One mosfet connects the
phase to the supply voltage, the other connects the phase to ground. The 'high
side PWM' drives the high side mosfet and the low side PWM signal drives the
low side mosfet. High side means the mosfet connects the load (the phase coil)
to the supply voltage, low side means the mosfet connects the load to ground. A
logical high on the PWM signal will turn on its respective mosfet. An 'on'
mosfet can be simplified as a conductor (closed circuit) while an 'off' mosfet
can be simplifed as an infinite resistance (an open circuit).

//image of single phase high and low mosfets (half H bridge)

The amount of time a PWM signal stays logically high relative to the pwm period
is called the duty cycle (high time / period). In this case, the duty cycle for
the high side PWM represents the percentage of time per duty cycle that the
phase is connected to the supply voltage, and the duty cycle for the low side
PWM represents the percentage of time per duty cycle that the coil is connected
to ground.

Any voltage in the range of ground to the supply can be acheived can by
rapidly changing whether the phase coil is connected to the supply voltage or
ground. If the PWM frequency is fast enough, the voltage supplied to the coil is
equal to the average voltage supplied to the phase over the period of the highside
duty cycle (HS duty cycle * supply voltage). To accomplish this in code, a
counter register is set that increments with every clock tick. This value is
then constantly compared to duty cycle. If the counter is less than the duty cycle
the high side PWM is driven high and the low side PWM is driven low. If the 
counter is greater than the duty cycle, the low side PWM is driven high and the
high side PWM is driven low. 

//pretty matlab picture

Unfortunately, it is not as simple as inverting the high side PWM signal to
create the required low side PWM signal. This has the potential to cause 'shoot 
through' where both the high and low side fets are on simultaniously and the
supply voltage is shorted to ground. This is caused by fet turn off delay time, 
which is the amount of time between the PWM signal going low and the fet turning 
off. This value can be found on the mosfet datasheet. In this PWM module, 
the turn off delay is corrected by dead time, where both the high side and low
side fets are turned off to ensure no shoot through occurs. 

//pretty picture of dead time

The only instances where dead time is not neccessary is when the PWM signal is
either unity/zero or the phase is set to high impedance. In these cases the fets
are do not alternate going on and off so there is no danger of shoot through.

--Doho*/

`ifndef _PHASE_DRIVER_
`define _PHASE_DRIVER_


module Phase_Driver ( clk, duty_cycle, high_z, pwm_high, pwm_low );

parameter DEAD_TIME = 2;            // dead time in units of clock ticks
parameter COUNTER_WIDTH = 10;        // bits available to counter
parameter MAX_COUNTER = 10'h3ff;          // PWM period = MAX_COUNTER * clockPeriod
parameter DUTY_CYCLE_WIDTH = 10;     // bits available to duty_cycle
parameter MAX_DUTY_CYCLE = 10'h3ff;       // Value represeting a duty cycle of 100%
parameter DUTY_CYCLE_STEP_RES = 1;  // ceil( MAX_COUNTER / MAX_DUTY_CYCLE ) 

input   clk;
input   [DUTY_CYCLE_WIDTH-1:0] duty_cycle;
input   high_z;
output  pwm_high, pwm_low;
// ===============================================  

// wire pwm_high, pwm_low;
wire h, l;
//wire deadtime;

reg [COUNTER_WIDTH-1:0] counter = 0;

//assign deadtime  = (duty_cycle == MAX_DUTY_CYCLE || duty_cycle == 0) ? 0 : DEAD_TIME;

assign h = ( ( counter + DEAD_TIME ) < ( duty_cycle * DUTY_CYCLE_STEP_RES ) ) ? 1 : 0;
assign l = ( ( counter >= ( duty_cycle * DUTY_CYCLE_STEP_RES ) ) && ( ( counter + DEAD_TIME ) < MAX_COUNTER ) ) ? 1 : 0;


assign  pwm_high = 	(high_z == 1) ? 0 : h;

assign	pwm_low = 	(high_z == 1) ? 0 :
					(duty_cycle == 0) ? 1 :
                    l;

always @(posedge clk) begin : PHASE_DRIVER
	counter = counter + 1;
	if (counter >= MAX_COUNTER) counter = 0;
end

endmodule

`endif
