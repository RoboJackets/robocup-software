// Signal naming conventions:
// signal_s is a copy of the signal synchronized to the master clock.
// 	(only applies to signals sampled from inputs)
// signal_d is a copy of the signal delayed from signal_s by one clock.
// signal_d2 is a copy of the signal delayed from signal_s by two clocks.
//
// To prevent glitches:
// All inputs MUST be synchronized before use.
// All outputs MUST be driven directly from registers, not from combinational logic.
// "always" blocks should only be clocked directly by sysclk (18.432MHz).
// 	The exception is when using an always block to write combinational logic.
// 	ALL registers must be synchronized to sysclk.  Do not use gated or divided clocks.
//
// "begin" and "end" should always be used where applicable to prevent stupid bugs.
// I know it's ugly.
//
// Registers should always be initialized so that the design behaves the same in
// simulation as on the real hardware.
// Use includes rather than a project file to make integration into a simulation easier.

`include "motor.v"
`include "kicker.v"
`include "encoder.v"
`include "kicker_i2c.v"
`include "hall_counter.v"

// Conventions:
//  Hall effect sensor vectors are [2:0] which maps to {a, b, c}.
//  Half-bridge drive vectors are [1:0] which maps to {high, low}.
//  Motor output vectors are [5:0] which maps to
//      {high_a, low_a, high_b, low_b, high_c, low_c}
//       MxA_H   MxA_L  MxB_H   MxB_L  MxC_H   MxC_L

module robocup (
	// Clock
	input sysclk,
	
	// Hall effect sensors on motors
	input m1hall_a, m1hall_b, m1hall_c,
	input m2hall_a, m2hall_b, m2hall_c,
	input m3hall_a, m3hall_b, m3hall_c,
	input m4hall_a, m4hall_b, m4hall_c,
	input m5hall_a, m5hall_b, m5hall_c,
	
	// Half-bridge outputs
	output m1a_h, m1a_l, m1b_h, m1b_l, m1c_h, m1c_l,
	output m2a_h, m2a_l, m2b_h, m2b_l, m2c_h, m2c_l,
	output m3a_h, m3a_l, m3b_h, m3b_l, m3c_h, m3c_l,
	output m4a_h, m4a_l, m4b_h, m4b_l, m4c_h, m4c_l,
	output m5a_h, m5a_l, m5b_h, m5b_l, m5c_h, m5c_l,
	
	// Encoders
	input m1enc_a, m1enc_b,
	input m2enc_a, m2enc_b,
	input m3enc_a, m3enc_b,
	input m4enc_a, m4enc_b,

	// Discharge button
	input discharge,
	
	// Kicker
	input kdone,
	output kcharge, kkick, kchip,
	inout ksda, kscl,
	
	// Microcontroller interface
	input flash_ncs,
	input fpga_ncs,
	input mosi, sck,
	inout miso
);

// This is sent as the first byte of every SPI transfer
localparam LOGIC_VERSION = 8'h04;

// Status that can be read over SPI
wire [5:1] motor_fault;
reg [15:0] encoder_capture_1 = 0;
reg [15:0] encoder_capture_2 = 0;
reg [15:0] encoder_capture_3 = 0;
reg [15:0] encoder_capture_4 = 0;

reg [7:0] hall_count_capture_1 = 0;
reg [7:0] hall_count_capture_2 = 0;
reg [7:0] hall_count_capture_3 = 0;
reg [7:0] hall_count_capture_4 = 0;
reg [7:0] hall_count_capture_5 = 0;

// Motor commands
reg [5:1] motor_dir = 0;
reg [8:0] motor_speed_1 = 0;
reg [8:0] motor_speed_2 = 0;
reg [8:0] motor_speed_3 = 0;
reg [8:0] motor_speed_4 = 0;
reg [8:0] motor_speed_5 = 0;

reg [1:0] drive_mode_1 = 0;
reg [1:0] drive_mode_2 = 0;
reg [1:0] drive_mode_3 = 0;
reg [1:0] drive_mode_4 = 0;
reg [1:0] drive_mode_5 = 0;

wire [7:0] kicker_status;
wire [7:0] kicker_voltage;
wire kicker_voltage_ok;

// SPI interface

// Data register
reg [7:0] spi_dr = 0;

// Synchronized and delayed interface signals
reg sck_s = 0, sck_d = 0;
reg mosi_s = 0;
reg ncs_s = 0;
reg ncs_d = 0;

// Value of MOSI at the last SCK rising edge
reg mosi_sampled = 0;

// Index of the byte being received
reg [4:0] spi_byte_count = 0;

// Which bit in the current byte is being received
reg [2:0] spi_bit_count = 0;

// High for one cycle after a transfer is started
wire spi_start_strobe = (ncs_d == 1 && ncs_s == 0);

// High for one cycle at the end of a transfer.
// spi_byte_count is valid for this cycle.
wire spi_end_strobe = (ncs_d == 0 && ncs_s == 1);

// Received data
reg [7:0] spi_rx[0:14];

wire sck_rising = (sck_d == 0 && sck_s == 1);
wire sck_falling = (sck_d == 1 && sck_s == 0);

// High for one cycle when spi_dr needs to be loaded with the next byte to transmit.
// This does not apply to byte 0: that byte must be loaded whenever ncs_s == 1.
wire spi_tx_setup = (sck_falling && spi_bit_count == 0);

// The command byte is the first byte received
wire [7:0] spi_command = spi_rx[0];

reg [7:0] watchdog_overflow_count = 0;

// SPI serdes logic
//
// This is designed for SPI mode 0 (clock idles low, data valid on rising edge).
//
// SCK must be at most sysclk/8.  This could be improved with an asynchronous shift
// register, but I don't want to deal with synchronizing that to the other logic.
always @(posedge sysclk) begin
	// Synchronize SPI signals
	sck_d <= sck_s;
	sck_s <= sck;
	mosi_s <= mosi;
	ncs_s <= fpga_ncs;
	ncs_d <= ncs_s;
	
	if (ncs_s == 1) begin
		// Reset when not selected
		spi_bit_count <= 0;
		spi_byte_count <= 0;
		spi_rx[0] <= 0;
	end else begin
		if (sck_rising) begin
			// SCK rising edge: middle of a bit.  Sample MOSI.
			mosi_sampled <= mosi_s;

			// Increment bit and byte counts
			spi_bit_count <= spi_bit_count + 1;
			if (spi_bit_count == 7) begin
				spi_byte_count <= spi_byte_count + 1;
			end
			
			if (spi_bit_count == 7) begin
				// Sampling the last bit of a byte.
				if (spi_byte_count <= 11) begin
					spi_rx[spi_byte_count] <= {spi_dr[6:0], mosi_s};
				end
			end
		end else if (sck_falling) begin
			// SCK falling edge: end of a bit.  Change MISO.
			if (spi_bit_count != 0) begin
				// End of a bit: shift on SCK falling edge if not loading new data
				spi_dr <= {spi_dr[6:0], mosi_sampled};
			end
		end
	end

	// SPI transmit logic
	if (ncs_s == 1) begin
		// TX byte 0: FPGA SPI interface version
		spi_dr <= LOGIC_VERSION;
	end else if (spi_tx_setup) begin
		// Load transmit bytes after the first one
		if (spi_command == 8'h00) begin
				case (spi_byte_count)
				1: spi_dr <= encoder_capture_1[7:0];
				2: spi_dr <= encoder_capture_1[15:8];
				3: spi_dr <= encoder_capture_2[7:0];
				4: spi_dr <= encoder_capture_2[15:8];
				5: spi_dr <= encoder_capture_3[7:0];
				6: spi_dr <= encoder_capture_3[15:8];
				7: spi_dr <= encoder_capture_4[7:0];
				8: spi_dr <= encoder_capture_4[15:8];
				9: spi_dr <= {3'b000, motor_fault};
				10: spi_dr <= kicker_status;
				11: spi_dr <= kicker_voltage;
				12: spi_dr <= hall_count_capture_1;
				13: spi_dr <= hall_count_capture_2;
				14: spi_dr <= hall_count_capture_3;
				15: spi_dr <= hall_count_capture_4;
				16: spi_dr <= hall_count_capture_5;
				default: spi_dr <= 8'h00;
				endcase
		end else if (spi_command == 8'h01) begin
				case (spi_byte_count)
				1: spi_dr <= watchdog_overflow_count;
				default: spi_dr <= 8'h00;
				endcase
		end else begin
				// Invalid command
				spi_dr <= 8'hee;
		end
	end
end

// Drive MISO only if we are selected
assign miso = fpga_ncs ? 1'bz : spi_dr[7];

reg kick_strobe = 0;
reg [7:0] kick_strength = 0;
reg kick_select = 0;
reg charge = 0;

// Watchdog timer for motor updates.
// When this overflows, all motor outputs are reset.
reg [21:0] watchdog = 0;

// Capture all encoder counts together at the beginning of a transfer
wire [15:0] encoder_1;
wire [15:0] encoder_2;
wire [15:0] encoder_3;
wire [15:0] encoder_4;

wire [7:0] hall_count_1;
wire [7:0] hall_count_2;
wire [7:0] hall_count_3;
wire [7:0] hall_count_4;
wire [7:0] hall_count_5;

always @(posedge sysclk) begin
	if (spi_start_strobe) begin
		encoder_capture_1 <= encoder_1;
		encoder_capture_2 <= encoder_2;
		encoder_capture_3 <= encoder_3;
		encoder_capture_4 <= encoder_4;
		
		hall_count_capture_1 <= hall_count_1;
		hall_count_capture_2 <= hall_count_2;
		hall_count_capture_3 <= hall_count_3;
		hall_count_capture_4 <= hall_count_4;
		hall_count_capture_5 <= hall_count_5;
	end
end

// SPI receive logic
always @(posedge sysclk) begin
	if (spi_end_strobe && spi_command == 8'h01 && spi_byte_count == 12) begin
		// Finished receiving SPI command
		watchdog <= 0;
		
		motor_speed_1 <= {spi_rx[2][0], spi_rx[1]};
		motor_dir[1] <= spi_rx[2][1];
		drive_mode_1 <= spi_rx[2][3:2];
		
		motor_speed_2 <= {spi_rx[4][0], spi_rx[3]};
		motor_dir[2] <= spi_rx[4][1];
		drive_mode_2 <= spi_rx[4][3:2];
		
		motor_speed_3 <= {spi_rx[6][0], spi_rx[5]};
		motor_dir[3] <= spi_rx[6][1];
		drive_mode_3 <= spi_rx[6][3:2];
		
		motor_speed_4 <= {spi_rx[8][0], spi_rx[7]};
		motor_dir[4] <= spi_rx[8][1];
		drive_mode_4 <= spi_rx[8][3:2];
		
		motor_speed_5 <= {spi_rx[10][0], spi_rx[9]};
		motor_dir[5] <= spi_rx[10][1];
		drive_mode_5 <= spi_rx[10][3:2];
		
		charge <= spi_rx[10][7];
		kick_select <= spi_rx[10][6];
		kick_strength <= spi_rx[11];
		if (spi_rx[11] != 0) begin
			kick_strobe <= 1;
		end
	end else begin
		if (watchdog == 22'h3ffffe) begin
			watchdog_overflow_count <= watchdog_overflow_count + 1;
		end
		if (watchdog == 22'h3fffff) begin
			// Watchdog timeout
			motor_speed_1 <= 0;
			motor_speed_2 <= 0;
			motor_speed_3 <= 0;
			motor_speed_4 <= 0;
			motor_speed_5 <= 0;
			drive_mode_1 <= 0;
			drive_mode_2 <= 0;
			drive_mode_3 <= 0;
			drive_mode_4 <= 0;
			drive_mode_5 <= 0;
			charge <= 0;
			motor_dir <= 0;
		end else begin
			// NOTE - This counter used to overflow, repeatedly resetting motor commands
			// about every 227ms.  The counter did not get reset on Xilinx Webpack 12.4 (!!!)
			// but it does work if the counter saturates.  Both cases work on version 13.1
			watchdog <= watchdog + 1;
		end
		
		kick_strobe <= 0;
	end
end

// PWM clock divider and counter
reg [8:0] pwm_phase = 0;
reg pwm_direction = 0;
always @(posedge sysclk) begin
	if (pwm_direction == 0) begin
		if (pwm_phase == 9'h1fd) begin
			pwm_direction <= 1;
		end
		pwm_phase <= pwm_phase + 1;
	end else begin
		if (pwm_phase == 9'h001) begin
			pwm_direction <= 0;
		end
		pwm_phase <= pwm_phase - 1;
	end
end

wire [2:0] hall_1 = {m1hall_a, m1hall_b, m1hall_c};
wire [2:0] hall_2 = {m2hall_a, m2hall_b, m2hall_c};
wire [2:0] hall_3 = {m3hall_a, m3hall_b, m3hall_c};
wire [2:0] hall_4 = {m4hall_a, m4hall_b, m4hall_c};
wire [2:0] hall_5 = {m5hall_a, m5hall_b, m5hall_c};

// Synchronize the hall inputs to the clock
reg [2:0] hall_sync_1, hall_sync_2, hall_sync_3, hall_sync_4, hall_sync_5;
always @(posedge sysclk) begin
	hall_sync_1 <= hall_1;
	hall_sync_2 <= hall_2;
	hall_sync_3 <= hall_3;
	hall_sync_4 <= hall_4;
	hall_sync_5 <= hall_5;
end

hall_counter hall_counter_1(sysclk, hall_sync_1, hall_count_1);
hall_counter hall_counter_2(sysclk, hall_sync_2, hall_count_2);
hall_counter hall_counter_3(sysclk, hall_sync_3, hall_count_3);
hall_counter hall_counter_4(sysclk, hall_sync_4, hall_count_4);
hall_counter hall_counter_5(sysclk, hall_sync_5, hall_count_5);

// Motor drivers
motor md_1(sysclk, pwm_phase, motor_dir[1], motor_speed_1, drive_mode_1, hall_sync_1, {m1a_h, m1a_l, m1b_h, m1b_l, m1c_h, m1c_l}, motor_fault[1]);
motor md_2(sysclk, pwm_phase, motor_dir[2], motor_speed_2, drive_mode_2, hall_sync_2, {m2a_h, m2a_l, m2b_h, m2b_l, m2c_h, m2c_l}, motor_fault[2]);
motor md_3(sysclk, pwm_phase, motor_dir[3], motor_speed_3, drive_mode_3, hall_sync_3, {m3a_h, m3a_l, m3b_h, m3b_l, m3c_h, m3c_l}, motor_fault[3]);
motor md_4(sysclk, pwm_phase, motor_dir[4], motor_speed_4, drive_mode_4, hall_sync_4, {m4a_h, m4a_l, m4b_h, m4b_l, m4c_h, m4c_l}, motor_fault[4]);
motor md_5(sysclk, pwm_phase, motor_dir[5], motor_speed_5, drive_mode_5, hall_sync_5, {m5a_h, m5a_l, m5b_h, m5b_l, m5c_h, m5c_l}, motor_fault[5]);

// Encoders
encoder counter1(sysclk, {m1enc_a, m1enc_b}, encoder_1);
encoder counter2(sysclk, {m2enc_a, m2enc_b}, encoder_2);
encoder counter3(sysclk, {m3enc_a, m3enc_b}, encoder_3);
encoder counter4(sysclk, {m4enc_a, m4enc_b}, encoder_4);

// Button synchronization for firing kicker manually
reg charge_enable = 1;
reg button_sync = 0;
reg done_sync = 0;
always @(posedge sysclk) begin
	button_sync <= ~discharge;
	done_sync <= ~kdone;
	
	if (button_sync) begin
		charge_enable <= 0;
	end
end

// Kicker
wire lockout;
assign kicker_status = {1'b0, kicker_voltage_ok, kick_select, ~charge_enable, charge, kcharge, lockout, done_sync};

wire kick_pulse;
kicker kicker(sysclk, button_sync, kick_strobe, kick_strength, charge & charge_enable, kcharge, kick_pulse, lockout);

// Send the kick pulse to either the kicker or the chipper
assign kkick = kick_pulse && (kick_select == 0);
assign kchip = kick_pulse && (kick_select == 1);

// Kicker voltage monitor
wire scl_out, sda_out;
kicker_i2c kicker_i2c(sysclk, scl_out, sda_out, ksda, kicker_voltage_ok, kicker_voltage);

assign kscl = scl_out ? 1'bz : 1'b0;
assign ksda = sda_out ? 1'bz : 1'b0;

endmodule
