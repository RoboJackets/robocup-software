// Motor driver and encoder

module half_bridge(
	input pwm_active, pwm_inverted,
	input [1:0] drive_mode,
	input [1:0] bridge_in,
	output reg [1:0] bridge_out
);

localparam NOFF = 2'b11;
localparam HIGH = 2'b10;
localparam LOW  = 2'b01;
localparam OFF  = 2'b00;

// Convert NOFF to OFF
wire [1:0] clean_in = (bridge_in == NOFF) ? OFF : bridge_in;

// LOW for driven outputs and OFF for open outputs
wire [1:0] brake = (clean_in == LOW || clean_in == HIGH) ? LOW : OFF;

// Slow decay: active outputs are normal and inactive output are low
reg [1:0] slow_decay;
always @(pwm_active or pwm_inverted or clean_in) begin
	if (pwm_active)
		slow_decay <= clean_in;
	else if (pwm_inverted)
		slow_decay <= (clean_in == LOW || clean_in == HIGH) ? LOW : OFF;
	else
		slow_decay <= OFF;
end

// Fast decay driving: active outputs are normal and inactive output are off
wire [1:0] fast_drive = pwm_active ? clean_in : OFF;

// Fast decay braking: set all driving outputs low and leave non-driving outputs open.
wire [1:0] fast_brake = pwm_active ? brake : OFF;

always @(drive_mode or slow_decay or fast_drive or fast_brake) begin
	case (drive_mode)
		2'b00: bridge_out <= OFF;
		2'b01: bridge_out <= slow_decay;
		2'b10: bridge_out <= fast_brake;
		2'b11: bridge_out <= fast_drive;
	endcase
end

endmodule

module motor(
	input clk,
	
	input [8:0] pwm_phase,
	
	input new_direction,
	input [8:0] new_level,
	input [1:0] drive_mode,
	
	input [2:0] hall,
	output reg [5:0] motor_out_s = 0,
	output fault
);

localparam NOFF = 2'b11;
localparam HIGH = 2'b10;
localparam LOW  = 2'b01;
localparam OFF  = 2'b00;

//FIXME - Check this experimentally.  This is multiplied by the pwm clock divisor.
// 3 seems to be OK.  2 causes increased current draw, but I don't know if that is from shoot-through
// (200ns dead time should be enough, so 2 should work).
localparam dead_time = 10;

reg direction = 0;
reg [8:0] pwm_level = 0;
always @(posedge clk) begin
	if (pwm_phase == 9'h1fe) begin
		direction <= new_direction;
		pwm_level <= new_level;
	end
end

// Apply dead time to the inverted side so if we are only using the active side
// (fast decay driving or braking) it has no effect.
// We need an extra bit in case of overflow.
wire [9:0] pwm_inverted_threshold = {1'b0, pwm_level} + dead_time;

wire pwm_active = (pwm_phase < pwm_level);
wire pwm_inverted = (pwm_phase >= pwm_inverted_threshold);

// Commutation
wire [5:0] com_out =
	(hall == 3'b101) ? {HIGH, LOW,  OFF} :
	(hall == 3'b100) ? {HIGH, OFF,  LOW} :
	(hall == 3'b110) ? {OFF,  HIGH, LOW} :
	(hall == 3'b010) ? {LOW,  HIGH, OFF} :
	(hall == 3'b011) ? {LOW,  OFF,  HIGH} :
	(hall == 3'b001) ? {OFF,  LOW,  HIGH} :
					   {OFF,  OFF,  OFF};

// Direction
// For CCW: high->low, low->high, 00->11 (off->off)
wire [5:0] bridge_drive = direction ? ~com_out : com_out;

wire [5:0] motor_out;
half_bridge half_bridge[1:3](pwm_active, pwm_inverted, drive_mode, bridge_drive, motor_out);

// Synchronize outputs to eliminate glitches
always @(posedge clk) begin
	motor_out_s <= motor_out;
end

// Detect bad hall-effect inputs.
// It's still possible for one or two inputs to fail and not be detected here.
// The CPU looks for stalled motors to detect that case.
assign fault = (hall == 3'b000) || (hall == 3'b111);

endmodule
