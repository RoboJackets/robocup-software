// ALE, RD, and WR are active low.

// Conventions:
//  Hall effect sensor vectors are [2:0] which maps to {a, b, c}.
//  Half-bridge drive vectors are [1:0] which maps to {high, low}.
//  Motor output vectors are [5:0] which map to
//      {high_a, low_a, high_b, low_b, high_c, low_c}.

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
	input kvdata, kidata,
	output kncs, kclk,
	output klimit,
	input kspare,
	
	// Microcontroller interface
	input fpga_ncs,
	input fs0, fs1,
	input mosi, sck,
	inout miso,

	// This is only included so we can set the pullup configuration for this pin by itself
	input flash_ncs
);

// Status that can be read over SPI
reg [5:1] motor_fault = 0;
wire [5:1] fault_detect;
wire [7:0] hall_count_1;
wire [7:0] hall_count_2;
wire [7:0] hall_count_3;
wire [7:0] hall_count_4;
wire [7:0] hall_count_5;

// SPI interface
reg [7:0] spi_dr = 0;
reg sck_s = 0, sck_d = 0;
reg mosi_s = 0;
reg ncs_s = 0;
reg mosi_d = 0;
reg [2:0] spi_bit_count = 0;

// Index of the byte in this SPI packet
reg [3:0] spi_byte_count = 0;

// First byte in the packet
reg [7:0] spi_command = 0;

// Goes high for one cycle after each byte is received
reg spi_strobe = 0;

reg [7:0] spi_rx = 0;

reg [7:0] test = 8'h55;

always @(posedge sysclk) begin
	sck_d <= sck_s;
	sck_s <= sck;
	mosi_s <= mosi;
	ncs_s <= fpga_ncs;
	
	spi_strobe <= 0;
	
	if (ncs_s == 1) begin
		// Reset when not selected
		spi_dr <= 8'hc9;
		spi_bit_count <= 0;
		spi_byte_count <= 0;
		spi_command <= 0;
	end else begin
		if (sck_d == 0 && sck_s == 1) begin
			// SCK rising edge: middle of a bit.  Sample MOSI.
			mosi_d <= mosi_s;
			
			if (spi_bit_count == 7) begin
				// Sampling the last bit of a byte.
				spi_strobe <= 1;
				spi_rx <= {spi_dr[6:0], mosi_s};
				
				if (spi_byte_count == 0) begin
					// Finished receiving the first byte, which is the command
					spi_command <= {spi_dr[6:0], mosi_s};
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
				// so the byte immediately after the commmand byte is spi_byte==0.
				case (spi_command)
					8'h00: spi_dr <= 8'ha5;
					
					8'h01: case (spi_byte_count)
						0: spi_dr <= hall_count_1;
						1: spi_dr <= hall_count_2;
						2: spi_dr <= hall_count_3;
						3: spi_dr <= hall_count_4;
						4: spi_dr <= hall_count_5;
						5: spi_dr <= {3'b100, fault_detect[5:1]};
					endcase
					
					8'hee: spi_dr <= test;
					default: spi_dr <= 8'h00;
				endcase
				
				spi_byte_count <= spi_byte_count + 1;
			end else begin
				// End of a bit: shift on SCK falling edge
				spi_dr <= {spi_dr[6:0], mosi_d};
			end
		end
	end
	
	if (spi_strobe && spi_command == 8'hdd && spi_byte_count == 1) begin
		test <= spi_rx;
	end
end

assign miso = fpga_ncs ? 1'bz : spi_dr[7];

// PWM clock divider
parameter pwm_period = 5;
reg [2:0] pwm_divider = 0;
reg pwm_clk = 0;
always @(posedge sysclk) begin
    if (pwm_divider == pwm_period) begin
        pwm_clk <= 1;
        pwm_divider <= 0;
    end else begin
        pwm_clk <= 0;
        pwm_divider <= pwm_divider + 1;
    end
end

// Connect motor signals to nets with names that match the schematic
wire [2:0] hall_1;
wire [2:0] hall_2;
wire [2:0] hall_3;
wire [2:0] hall_4;
wire [2:0] hall_5;

wire [5:0] motor_1;
wire [5:0] motor_2;
wire [5:0] motor_3;
wire [5:0] motor_4;
wire [5:0] motor_5;

// The connector for motor 2 is (top to bottom) hall C, B, A.
// The motor is (top to bottom) 3, 1, 2.

assign hall_1 = {m1hall_a, m1hall_b, m1hall_c};
assign hall_2 = {m2hall_a, m2hall_b, m2hall_c};
assign hall_3 = {m3hall_a, m3hall_b, m3hall_c};
assign hall_4 = {m4hall_a, m4hall_b, m4hall_c};
assign hall_5 = {m5hall_a, m5hall_b, m5hall_c};

assign m1a_h = motor_1[5];
assign m1a_l = motor_1[4];
assign m1b_h = motor_1[3];
assign m1b_l = motor_1[2];
assign m1c_h = motor_1[1];
assign m1c_l = motor_1[0];

assign m2a_h = motor_2[5];
assign m2a_l = motor_2[4];
assign m2b_h = motor_2[3];
assign m2b_l = motor_2[2];
assign m2c_h = motor_2[1];
assign m2c_l = motor_2[0];

assign m3a_h = motor_3[5];
assign m3a_l = motor_3[4];
assign m3b_h = motor_3[3];
assign m3b_l = motor_3[2];
assign m3c_h = motor_3[1];
assign m3c_l = motor_3[0];

assign m4a_h = motor_4[5];
assign m4a_l = motor_4[4];
assign m4b_h = motor_4[3];
assign m4b_l = motor_4[2];
assign m4c_h = motor_4[1];
assign m4c_l = motor_4[0];

assign m5a_h = motor_5[5];
assign m5a_l = motor_5[4];
assign m5b_h = motor_5[3];
assign m5b_l = motor_5[2];
assign m5c_h = motor_5[1];
assign m5c_l = motor_5[0];

// Motor drivers
wire set_pwm_1 = spi_strobe && spi_command == 8'h01 && spi_byte_count == 1;
wire set_pwm_2 = spi_strobe && spi_command == 8'h01 && spi_byte_count == 2;
wire set_pwm_3 = spi_strobe && spi_command == 8'h01 && spi_byte_count == 3;
wire set_pwm_4 = spi_strobe && spi_command == 8'h01 && spi_byte_count == 4;
wire set_pwm_5 = spi_strobe && spi_command == 8'h01 && spi_byte_count == 5;

motor md_1(sysclk, pwm_clk, set_pwm_1, spi_rx, hall_1, motor_1, hall_count_1, fault_detect[1]);
motor md_2(sysclk, pwm_clk, set_pwm_2, spi_rx, hall_2, motor_2, hall_count_2, fault_detect[2]);
motor md_3(sysclk, pwm_clk, set_pwm_3, spi_rx, hall_3, motor_3, hall_count_3, fault_detect[3]);
motor md_4(sysclk, pwm_clk, set_pwm_4, spi_rx, hall_4, motor_4, hall_count_4, fault_detect[4]);
motor md_5(sysclk, pwm_clk, set_pwm_5, spi_rx, hall_5, motor_5, hall_count_5, fault_detect[5]);

/*
// Fault register
always @(posedge sysclk) begin
	//FIXME - How to do this with SPI?
    if (write_edge && address_sync == 8'h08) begin
        // Clear given faults
        fault <= fault_detect | (fault & ~data_sync[4:0]);
    end else begin
        // Latch new faults
        fault <= fault_detect | fault;
    end
end
*/

// Encoders
wire [15:0] encoder_1;
wire [15:0] encoder_2;
wire [15:0] encoder_3;
wire [15:0] encoder_4;

encoder counter1(sysclk, {m1enc_a, m1enc_b}, encoder_1);
encoder counter2(sysclk, {m2enc_a, m2enc_b}, encoder_2);
encoder counter3(sysclk, {m3enc_a, m3enc_b}, encoder_3);
encoder counter4(sysclk, {m4enc_a, m4enc_b}, encoder_4);

// Latch all encoders together
reg [15:0] encoder_latch_1 = 0;
reg [15:0] encoder_latch_2 = 0;
reg [15:0] encoder_latch_3 = 0;
reg [15:0] encoder_latch_4 = 0;

/*
always @(posedge sysclk) begin
    if (write_edge && address_sync == 8'h30) begin
        encoder_latch_1 <= encoder_1;
        encoder_latch_2 <= encoder_2;
        encoder_latch_3 <= encoder_3;
        encoder_latch_4 <= encoder_4;
    end
end
*/

// Button synchronization for firing kicker manually
reg charge_override = 0;
reg button_sync = 0;
always @(posedge sysclk) begin
    button_sync <= ~discharge;
    
    if (button_sync) begin
        charge_override <= 1;
    end
end

// Kicker
reg charge_enable = 0;
reg kick_select = 0;
wire write_kick = 0;//(write_edge && address_sync == 8'h20);
wire lockout;
wire [7:0] kicker_status = {2'b00, kick_select, charge_override, charge_enable, kcharge, lockout, ~kdone};

wire kick_write = write_kick;
wire [7:0] kick_strength = 0;//data_sync;

/*
always @(posedge sysclk) begin
    if (write_edge && address_sync == 8'h21) begin
        kick_select <= data_sync[5];
        charge_enable <= data_sync[3];
    end
end
*/

//FIXME - Current limit PWM
assign klimit = 0;

wire kick_pulse;
kicker kicker(sysclk, button_sync, kick_write, kick_strength, charge_enable & ~charge_override, charge, kick_pulse, lockout);

// Send the kick pulse to either the kicker or the chipper
assign kkick = kick_pulse && (kick_select == 0);
assign kchip = kick_pulse && (kick_select == 1);

//FIXME - V/I tracing
assign kclk = 0;
assign kncs = 1;

endmodule
