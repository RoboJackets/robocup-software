`include "bldc.v"

// Motor driver and encoder

module half_bridge(
	input pwm_high, pwm_low,
	input [1:0] bridge_in,
	output [1:0] bridge_out
);

parameter NOFF = 2'b11;
parameter HIGH = 2'b10;
parameter LOW  = 2'b01;
parameter OFF  = 2'b00;

// Convert NOFF to OFF
wire [1:0] clean_in = (bridge_in == NOFF) ? OFF : bridge_in;

// LOW for driven outputs and OFF for open outputs
wire [1:0] brake = (clean_in == LOW || clean_in == HIGH) ? LOW : OFF;

reg [1:0] slow_decay;
always @(pwm_high or pwm_low or clean_in) begin
	if (pwm_high)
		slow_decay <= clean_in;
	else if (pwm_low)
		slow_decay <= (clean_in == LOW || clean_in == HIGH) ? LOW : OFF;
	else
		slow_decay <= OFF;
end

// Fast decay driving: work normally, and change NOFF to OFF.
wire [1:0] fast_drive = pwm_high ? clean_in : OFF;

// Fast decay braking: set all driving outputs low and leave non-driving outputs open.
wire [1:0] fast_brake = pwm_high ? brake : OFF;

//FIXME - Select drive mode
assign bridge_out = slow_decay;

endmodule

module motor(
	input clk,
	
	input [6:0] pwm_phase,
	
	input write,
	input [7:0] data_in,
	
	input [2:0] hall,
	output [5:0] motor_out,
	output fault
);

//FIXME - Check this experimentally.  This is multiplied by the pwm clock divisor.
// 3 seems to be OK.  2 causes increased current draw, but I don't know if that is from shoot-through
// (200ns dead time should be enough, so 2 should work).
parameter dead_time = 3;

// Synchronize the hall inputs to the clock
reg [2:0] hall_sync;
always @(posedge clk) begin
	hall_sync <= hall;
end

// Motor driver
wire [5:0] bridge_drive;
reg direction = 0;
reg [6:0] pwm_level = 0;
always @(posedge clk) begin
	if (write) begin
		direction <= data_in[7];
		pwm_level <= data_in[6:0];
	end
end

// Apply dead time to the low side so if we are only using the high side
// (fast decay driving or braking) it has no effect.
// We need an extra bit in case of overflow.
wire [7:0] pwm_low_threshold = {0, pwm_level} + dead_time;

wire pwm_high = (pwm_phase < pwm_level);
wire pwm_low  = (pwm_phase >= pwm_low_threshold);

bldc bldc(direction, hall_sync, bridge_drive);
half_bridge half_bridge[1:3](pwm_high, pwm_low, bridge_drive, motor_out);

assign fault = (hall == 3'b000) || (hall == 3'b111);

endmodule
