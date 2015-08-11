// Signal naming conventions:
//	signal_s is a copy of the signal synchronized to the master clock.
//		(only applies to signals sampled from inputs)
//	signal_d is a copy of the signal delayed from signal_s by one clock.
//	signal_d2 is a copy of the signal delayed from signal_s by two clocks.
//
// To prevent glitches:
// All inputs MUST be synchronized before use.
// All outputs MUST be driven directly from registers, not from combinational logic.
// "always" blocks should only be clocked directly by sysclk (18.432MHz).
//		The exception is when using an always block to write combinational logic.
//		ALL registers must be synchronized to sysclk.  Do not use gated or divided clocks.
//
// "begin" and "end" should always be used where applicable to prevent stupid bugs.
// I know it's ugly.
//
// Registers should always be initialized so that the design behaves the same in
// simulation as on the real hardware.
// Use includes rather than a project file to make integration into a simulation easier.

`include "BLDC_Motor.v"
`include "git_version.vh"


module robocup (
	// Clock
	input sysclk,

	// Hall effect sensors
	input [4:0] hall_a, hall_b, hall_c,

	// Encoders
	input [3:0] enc_a, enc_b,

	// 3-half-bridge
	output [4:0] phase_aH, phase_aL, phase_bH, phase_bL, phase_cH, phase_cL;

	// Phase driver chip select pins
	output [4:0] drv_ncs,

	// ADC chip select pins
	output [1:0] adc_ncs,

	input spi_slave_sck, spi_slave_mosi, spi_slave_ncs
	inout spi_slave_miso,

	input spi_master_miso
	output spi_master_sck, spi_master_mosi

	parameter DATA_WIDTH=4;
		parameter CLK_DIVIDER_WIDTH=4;
);


// This is sent as the first byte of every SPI transfer
localparam LOGIC_VERSION = 8'h05;

// Status that can be read over SPI
wire [5:1] motor_fault;
reg [15:0] encoder_capture_1 = 0;
reg [7:0] hall_count_capture_1 = 0;
reg [5:1] motor_dir = 0;
reg [8:0] motor_speed_1 = 0;
reg [1:0] drive_mode_1 = 0;




// SPI interface

// Data register
reg [7:0] spi_dr = 0;

// Synchronized and delayed interface signals
reg sck_s = 0, sck_d = 0;
reg mosi_s = 0;
reg ncs_s = 0, ncs_d = 0;

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

wire 	sck_rising 	=	( sck_d == 0 && sck_s == 1 ), 
		sck_falling	= 	( sck_d == 1 && sck_s == 0 );

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

// Watchdog timer for motor updates.
// When this overflows, all motor outputs are reset.
reg [21:0] watchdog = 0;

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
	end
end


// This is the interface used for communications back to the mbed
spi_slave #(
	.DATA_WIDTH(DATA_WIDTH)
	)
	spi_slave_interface
	(
	.CPOL(1'b0),
	.CPHA(1'b0),
	.datai(slave_data0),
	.datao(slave_data0),
	.sclk(sclk[0]),
	.csb(csb[0]),
	.din(din[0]),
	.dout(dout[0])
);


// This handles all of the SPI communications to/from the motor board
Motor_Board_Comm motor_board_comm_module();


// These are all of the instances for the drive motors and the dribbler
genvar i;
generate for (i = 0; i < 5; i = i + 1)
	begin: Motor_Param_Inst

		wire [1:0] enc[i] = { enc_a[i], enc_b[i] };
		wire [2:0] hall[i] = { hall_a[i], hall_b[i], hall_c[i] };
		wire [5:0] phaseOut[i] = { phase_aH[i], phase_aL[i], phase_bH[i], phase_bL[i], phase_cH[i], phase_cL[i] };

    end
endgenerate
generate for (i = 0; i < 5; i = i + 1)
	begin: Motor_Inst

		BLDC_Motor motor_i (
			.clk(sysclk),
			.duty_cycle(duty_cycle_sync[i]),
			.hall(hall_sync[i]),
			.enc(enc_sync[i]),
			.phaseH(phaseH_setup[i]),
			.phaseL(phaseL_setup[i]),
			.hall_count(hall_count_setup[i]),
			.enc_count(enc_count_setup[i])
		);

    end
endgenerate

endmodule
