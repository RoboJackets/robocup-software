`include "motor.v"
`include "kicker.v"
`include "encoder.v"

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
	
	// Microcontroller interface
	input fpga_ncs,
	input mosi, sck,
	inout miso
);

// Status that can be read over SPI
wire [5:1] motor_fault;
reg [15:0] encoder_capture_1 = 0;
reg [15:0] encoder_capture_2 = 0;
reg [15:0] encoder_capture_3 = 0;
reg [15:0] encoder_capture_4 = 0;

// Motor commands
reg [5:1] motor_dir = 0;
reg [8:0] motor_speed_1 = 0;
reg [8:0] motor_speed_2 = 0;
reg [8:0] motor_speed_3 = 0;
reg [8:0] motor_speed_4 = 0;
reg [8:0] motor_speed_5 = 0;

wire [7:0] kicker_status;

// SPI interface
reg [7:0] spi_dr = 0;
reg sck_s = 0, sck_d = 0;
reg mosi_s = 0;
reg ncs_s = 0;
reg mosi_d = 0;
reg [2:0] spi_bit_count = 0;

// Index of the byte in this SPI packet
reg [3:0] spi_byte_count = 0;

// Goes high for one cycle after each byte is received
reg spi_strobe = 0;

reg [7:0] spi_rx[0:10];

always @(posedge sysclk) begin
	// Synchronize SPI signals
	sck_d <= sck_s;
	sck_s <= sck;
	mosi_s <= mosi;
	ncs_s <= fpga_ncs;
	
	spi_strobe <= 0;
	
	if (ncs_s == 1) begin
		// Reset when not selected
		spi_dr <= 8'h02;	// RX byte 0: FPGA SPI interface version
		spi_bit_count <= 0;
		spi_byte_count <= 0;
		spi_rx[0] <= 0;
	end else begin
		if (sck_d == 0 && sck_s == 1) begin
			// SCK rising edge: middle of a bit.  Sample MOSI.
			mosi_d <= mosi_s;
			
			if (spi_bit_count == 7) begin
				// Sampling the last bit of a byte.
				spi_strobe <= 1;
				
				if (spi_byte_count <= 10) begin
					spi_rx[spi_byte_count] <= {spi_dr[6:0], mosi_s};
				end
			end
		end
		
		if (sck_d == 1 && sck_s == 0) begin
			// SCK falling edge: end of a bit.  Change MISO.
			
			spi_bit_count <= spi_bit_count + 1;
			
			if (spi_bit_count == 7) begin
				// Just finished a byte
				
				// Set up to send the next byte.
				// Note that spi_byte has not been incremented at this point,
				// so the byte after the first byte (version, above) is spi_byte==0.
				case (spi_byte_count)
					0: spi_dr <= encoder_capture_1[7:0];
					1: spi_dr <= encoder_capture_1[15:8];
					2: spi_dr <= encoder_capture_2[7:0];
					3: spi_dr <= encoder_capture_2[15:8];
					4: spi_dr <= encoder_capture_3[7:0];
					5: spi_dr <= encoder_capture_3[15:8];
					6: spi_dr <= encoder_capture_4[7:0];
					7: spi_dr <= encoder_capture_4[15:8];
					8: spi_dr <= {3'b000, motor_fault[5:1]};
					9: spi_dr <= kicker_status;
					default: spi_dr <= 0;
				endcase
				
				spi_byte_count <= spi_byte_count + 1;
			end else begin
				// End of a bit: shift on SCK falling edge
				spi_dr <= {spi_dr[6:0], mosi_d};
			end
		end
	end
end

assign miso = fpga_ncs ? 1'bz : spi_dr[7];

reg kick_strobe = 0;
reg [7:0] kick_strength = 0;
reg kick_select = 0;
reg charge_enable = 0;

// Watchdog timer for motor updates.
// When this overflows, all motor outputs are reset.
reg [21:0] watchdog = 0;

always @(posedge sysclk) begin
	if (ncs_s == 1 && spi_byte_count == 11) begin
		watchdog <= 0;
		
		motor_speed_1 <= {spi_rx[1][0], spi_rx[0]};
		motor_dir[1] <= spi_rx[1][1];
		motor_speed_2 <= {spi_rx[3][0], spi_rx[2]};
		motor_dir[2] <= spi_rx[3][1];
		motor_speed_3 <= {spi_rx[5][0], spi_rx[4]};
		motor_dir[3] <= spi_rx[5][1];
		motor_speed_4 <= {spi_rx[7][0], spi_rx[6]};
		motor_dir[4] <= spi_rx[7][1];
		motor_speed_5 <= {spi_rx[9][0], spi_rx[8]};
		motor_dir[5] <= spi_rx[9][1];
		charge_enable <= spi_rx[9][7];
		kick_select <= spi_rx[9][6];
		kick_strength <= spi_rx[10];
		if (spi_rx[10] != 0) begin
			kick_strobe <= 1;
		end
	end else begin
		if (watchdog == 22'h3fffff) begin
			// Watchdog timeout
			motor_speed_1 <= 0;
			motor_speed_2 <= 0;
			motor_speed_3 <= 0;
			motor_speed_4 <= 0;
			motor_speed_5 <= 0;
			charge_enable <= 0;
			motor_dir <= 0;
		end else begin
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

// Motor drivers
motor md_1(sysclk, pwm_phase, motor_dir[1], motor_speed_1, {m1hall_a, m1hall_b, m1hall_c}, {m1a_h, m1a_l, m1b_h, m1b_l, m1c_h, m1c_l}, motor_fault[1]);
motor md_2(sysclk, pwm_phase, motor_dir[2], motor_speed_2, {m2hall_a, m2hall_b, m2hall_c}, {m2a_h, m2a_l, m2b_h, m2b_l, m2c_h, m2c_l}, motor_fault[2]);
motor md_3(sysclk, pwm_phase, motor_dir[3], motor_speed_3, {m3hall_a, m3hall_b, m3hall_c}, {m3a_h, m3a_l, m3b_h, m3b_l, m3c_h, m3c_l}, motor_fault[3]);
motor md_4(sysclk, pwm_phase, motor_dir[4], motor_speed_4, {m4hall_a, m4hall_b, m4hall_c}, {m4a_h, m4a_l, m4b_h, m4b_l, m4c_h, m4c_l}, motor_fault[4]);
motor md_5(sysclk, pwm_phase, motor_dir[5], motor_speed_5, {m5hall_a, m5hall_b, m5hall_c}, {m5a_h, m5a_l, m5b_h, m5b_l, m5c_h, m5c_l}, motor_fault[5]);

// Encoders
wire [15:0] encoder_1;
wire [15:0] encoder_2;
wire [15:0] encoder_3;
wire [15:0] encoder_4;

encoder counter1(sysclk, {m1enc_a, m1enc_b}, encoder_1);
encoder counter2(sysclk, {m2enc_a, m2enc_b}, encoder_2);
encoder counter3(sysclk, {m3enc_a, m3enc_b}, encoder_3);
encoder counter4(sysclk, {m4enc_a, m4enc_b}, encoder_4);

// Capture all encoder counts together at the beginning of a transfer
always @(posedge sysclk) begin
	if (spi_strobe && spi_byte_count == 0) begin
		encoder_capture_1 <= encoder_1;
		encoder_capture_2 <= encoder_2;
		encoder_capture_3 <= encoder_3;
		encoder_capture_4 <= encoder_4;
	end
end

// Button synchronization for firing kicker manually
reg charge_override = 0;
reg button_sync = 0;
reg done_sync = 0;
always @(posedge sysclk) begin
	button_sync <= ~discharge;
	done_sync <= ~kdone;
	
	if (button_sync) begin
		charge_override <= 1;
	end
end

// Kicker
wire lockout;
assign kicker_status = {2'b00, kick_select, charge_override, charge_enable, kcharge, lockout, done_sync};

wire kick_pulse;
kicker kicker(sysclk, button_sync, kick_strobe, kick_strength, charge_enable & ~charge_override, kcharge, kick_pulse, lockout);

// Send the kick pulse to either the kicker or the chipper
assign kkick = kick_pulse && (kick_select == 0);
assign kchip = kick_pulse && (kick_select == 1);

endmodule
