// Top module for the FPGA logic

`include "BLDC_Motor.v"
`include "SPI_Slave.v"
//`include "git_version.vh"


module robocup #(
	parameter 		NUM_MOTORS 				= 	(  5 ),
					NUM_HALL_SENS 			= 	( NUM_MOTORS ),
					NUM_ENCODERS 			= 	( NUM_MOTORS - 1 ),

					SPI_MASTER_CPOL 		= 	(  0 ),
					SPI_MASTER_CPHA 		= 	(  0 ),
					SPI_MASTER_DATA_WIDTH 	= 	( 16 ),

					SPI_SLAVE_CPOL 			= 	(  0 ),
					SPI_SLAVE_CPHA 			= 	(  0 ),
					SPI_SLAVE_DATA_WIDTH	= 	(  8 )

    ) (
	// Clock
	input 								sysclk,

	// 3-half-bridge
	output reg	[ NUM_MOTORS - 1:0 ]	phase_aH, 	phase_aL, 	phase_bH, 	phase_bL, 	phase_cH, 	phase_cL,

	// Hall effect sensors
	input		[ NUM_HALL_SENS - 1:0 ]	hall_a, 	hall_b, 	hall_c,

	// Encoders
	input		[ NUM_ENCODERS - 1:0 ] 	enc_a,
	input		[ NUM_ENCODERS - 1:0 ] 	enc_b,

	// Phase driver chip select pins
	output reg	[ NUM_MOTORS - 1:0 ]	drv_ncs,

	// ADC chip select pins
	output reg	[ 1:0 ] 				adc_ncs,

	// SPI Slave pins
	input 								spi_slave_sck, 			spi_slave_mosi, 		spi_slave_ncs,
	output 								spi_slave_miso,

	// SPI Master pins
	input 								spi_master_miso,
	output reg							spi_master_sck, 		spi_master_mosi
);

// THIS MUST BE INCLUDED RIGHT AFTER THE MODULE DECLARATION
`include "log2-macro.v"

// Declare variables for synthesis that is used for generating modules within loops.
// Note that this is only used during synthesis (ie. compilation).
genvar i;
integer j, k;

// Derived parameters 
localparam ENCODER_COUNT_WIDTH 			= 	16;
localparam HALL_COUNT_WIDTH 			= 	( 3 * SPI_SLAVE_DATA_WIDTH ) - ENCODER_COUNT_WIDTH;
localparam DUTY_CYCLE_WIDTH 			= 	10;
localparam WATCHDOG_TIMER_WIDTH 		= 	32;
localparam NUM_ENCODER_GEN 				= 	NUM_ENCODERS;

// Input register synchronization declarations - for the notation, we add a '_s' after the name indicating it is synced
reg	[ 2:0 ] hall_s 	[ NUM_HALL_SENS - 1 : 0 ];
reg	[ 1:0 ] enc_s 	[ NUM_ENCODERS  - 1 : 0 ];
reg			spi_slave_sck_s, 	
			spi_slave_mosi_s,
			spi_slave_ncs_s, 
			spi_slave_ncs_d,	// we have a delayed version of the chip select line
			spi_master_miso_s;

// Sync all of the input pins here into registers for reducing errors resulting noise
always @( posedge sysclk )
begin : SYNC_INPUTS
	// Hall inputs
	for (j = 0; j < NUM_HALL_SENS; j = j + 1)
	begin : GEN_HALL_ARRAY

		hall_s[j]		<=	{ hall_a[j], hall_b[j], hall_c[j] };
	end
	// Encoder inputs
	for (j = 0; j < NUM_ENCODERS; j = j + 1) 
	begin : GEN_ENC_ARRAY

		enc_s[j]		<=	{ enc_a[j], enc_b[j] };
	end
	// SPI slave inputs
	spi_slave_sck_s		<=	spi_slave_sck;
	spi_slave_mosi_s	<=	spi_slave_mosi;
	spi_slave_ncs_s		<=	spi_slave_ncs;
	spi_slave_ncs_d 	<= 	spi_slave_ncs_s; 	// the delayed register is always set from what the synced register is for the slave ncs line
	// SPI master inputs
	spi_master_miso_s	<=	spi_master_miso;
end

// Output register synchronization declarations - for the notation, we add a '_o' after the name indicating it will got to an output pin
wire [ 2:0 ] 				phaseH_o [ NUM_MOTORS - 1:0 ],
			 				phaseL_o [ NUM_MOTORS - 1:0 ];
wire [ NUM_MOTORS - 1:0 ] 	drv_ncs_o;
wire [ 1:0 ] 				adc_ncs_o;
wire 						spi_slave_miso_o, 
							spi_master_sck_o, 
							spi_master_mosi_o;

// Sync all of the output pins for the same reasons we sync all of the input pins - this time in reverse
always @( posedge sysclk )
begin : SYNC_OUTPUTS
	for ( j= 0; j < NUM_MOTORS; j = j + 1 ) 
	begin : GEN_PHASE_ARRAY
		// Phase outputs (HIGH)
		{ phase_aH[j], phase_bH[j], phase_cH[j] }	<=	phaseH_o[j];
		// Phase outputs (LOW)
		{ phase_aL[j], phase_bL[j], phase_cL[j] }	<=	phaseL_o[j];
	end

	drv_ncs 			<= 	drv_ncs_o;
	adc_ncs 			<= 	adc_ncs_o;
	spi_master_sck 		<= 	spi_master_sck_o;
	spi_master_mosi 	<= 	spi_master_mosi_o;
end

// We only drive the slave's data output line if we are selected
assign spi_slave_miso = ( spi_slave_ncs_s == 1 ? 1'bZ : spi_slave_miso_o );

// Internal logic declarations
wire [ ENCODER_COUNT_WIDTH - 1:0 ] enc_count  [ NUM_ENCODERS  - 1:0 ];
wire [ HALL_COUNT_WIDTH	   - 1:0 ] hall_count [ NUM_HALL_SENS - 1:0 ];
wire [ NUM_HALL_SENS 	   - 1:0 ] hall_faults;

reg  [ DUTY_CYCLE_WIDTH	   - 1:0 ] duty_cycle 		 [ NUM_MOTORS    - 1:0 ];

// Command types for SPI access
localparam CMD_UPDATE_MTRS		= 0;
// The command types beginning at 0x10 have selectable read/write types according to the command's MSB.
localparam CMD_WRITE_TYPE 		= 0;
localparam CMD_READ_TYPE 		= 1;
localparam CMD_RW_TYPE_BASE 	= 'h10;
localparam CMD_ENCODER_COUNT 	= CMD_RW_TYPE_BASE + 1;
localparam CMD_HALL_COUNT 		= CMD_RW_TYPE_BASE + 2;
localparam CMD_DUTY_CYCLE 		= CMD_RW_TYPE_BASE + 3;
localparam CMD_VERSION	 		= CMD_RW_TYPE_BASE + 4;
// The command strobes start after the read/write command types
localparam CMD_STROBE_START 		= CMD_RW_TYPE_BASE + 'h10;
localparam CMD_TOGGLE_MOTOR_EN	 	= CMD_RW_TYPE_BASE + CMD_STROBE_START;

// Response & request buffer sizes
localparam SPI_SLAVE_RES_BUF_LEN = 15;
localparam SPI_SLAVE_REQ_BUF_LEN = SPI_SLAVE_RES_BUF_LEN;
localparam SPI_SLAVE_COUNTER_WIDTH = `LOG2(SPI_SLAVE_RES_BUF_LEN);

// These are for triggering the storage of values outside of the SPI's SCK domain
// reg tx_vals_flag = 0,
	reg rx_vals_flag = 0,
	motors_en = 1,
	spi_slave_byte_done_d = 0;
reg [ WATCHDOG_TIMER_WIDTH - 1:0 ] 		watchdog_timer = 0;
reg [ SPI_SLAVE_COUNTER_WIDTH - 1:0 ] 	spi_slave_byte_count = 0;
reg [ SPI_SLAVE_DATA_WIDTH - 1:0 ] 		spi_slave_res_buf [ SPI_SLAVE_RES_BUF_LEN - 1:0 ],		// response & request buffers
										spi_slave_req_buf [ SPI_SLAVE_REQ_BUF_LEN - 1:0 ];
// reg [ SPI_SLAVE_DATA_WIDTH - 1:0 ]		spi_slave_di;	// latched every incoming byte & also store the first byte as the command byte

wire [ SPI_SLAVE_DATA_WIDTH - 1:0 ] spi_slave_do;
wire spi_slave_start_strobe = ( (spi_slave_ncs_s == 0) && (spi_slave_ncs_d == 1) ),
 	 spi_slave_end_strobe	= ( (spi_slave_ncs_s == 1) && (spi_slave_ncs_d == 0) );
wire spi_slave_byte_done;

// wire 								spi_slave_load_byte_flag = (spi_slave_byte_done == 0) && (spi_slave_byte_done_d == 1);

// // `define SIMULATION
// `ifdef SIMULATION
// // Initilization for simulation
// initial begin
// 	for ( j = 0; j < NUM_MOTORS; j = j + 1 ) begin
// 		duty_cycle[j] = 'h7a;
// 		drv_ncs[j] = 1;
// 		enc_count_offset[j] = 0;
// 		hall_count_offset[j] = 0;
// 	end
	
// 	adc_ncs[0] = 1;
// 	adc_ncs[1] = 1;
// 	spi_master_miso_s = 0;
// 	motors_en = 0;

// 	#150 motors_en = 1;
// end
// `endif

// The command_byte is always the first byte in the request buffer when selected. The command byte ignores the MSB.
//wire [ SPI_SLAVE_DATA_WIDTH - 2:0 ] command_byte 	= spi_slave_req_buf[0][ SPI_SLAVE_DATA_WIDTH - 2:0 ];
// The top bit of a command byte is a flag that can be sent over the SPI slave bus to write/read the values for specified commands starting at the CMD_RW_TYPE_BASE address
//wire 								command_rw 		= spi_slave_req_buf[0][ SPI_SLAVE_DATA_WIDTH - 1 ];

wire tx_vals_flag = ( ( spi_slave_byte_count == 1 ) && ( spi_slave_byte_done_d == 1 ) );
wire [ SPI_SLAVE_DATA_WIDTH - 1:0 ]		spi_slave_di = spi_slave_res_buf[ spi_slave_byte_count ];
//assign spi_slave_req_buf[0] = 
wire [ SPI_SLAVE_DATA_WIDTH - 2:0 ] command_byte = ( ( spi_slave_byte_done_d == 1 ) && ( spi_slave_byte_count == 1 ) ) ? spi_slave_do[SPI_SLAVE_DATA_WIDTH - 2:0] : spi_slave_req_buf[0][SPI_SLAVE_DATA_WIDTH - 2:0];
wire 								command_rw 	 = ( ( spi_slave_byte_done_d == 1 ) && ( spi_slave_byte_count == 1 ) ) ? spi_slave_do[SPI_SLAVE_DATA_WIDTH - 1]   : spi_slave_req_buf[0][SPI_SLAVE_DATA_WIDTH - 1];
reg [ SPI_SLAVE_DATA_WIDTH - 2:0 ] command_byte_l = 0;
reg command_rw_l = 0;
// SPI Slave module
SPI_Slave spi_slave_module (
    .clk 			( sysclk 				),
    .SCK 			( spi_slave_sck			),
    .MOSI 			( spi_slave_mosi_s 		),
    .MISO 			( spi_slave_miso_o 		),
    .SSEL 			( spi_slave_ncs_s 		),
    .DONE 			( spi_slave_byte_done 	),
    .BYTE_RECEIVED 	( spi_slave_do			),
    .BYTE_TO_SEND 	( spi_slave_di 			)
);

// When the byte count changes, we need to find our next byte that we want to load for the data out
//always @( negedge spi_slave_byte_sent, posedge spi_slave_ncs_s, posedge spi_slave_end_strobe, posedge spi_slave_start_strobe )
always @( posedge sysclk )
begin : SPI_LOAD_BYTE
	// If the chip select line is toggled and it is now high, we are ending an SPI transfer, so reset everything & take action with what we received
	if ( spi_slave_end_strobe ) begin
		//spi_slave_di <= 0;
		// Signal to do something with the received bytes & save how may bytes were received. We do this here so it will happen after we set the received byte count
		rx_vals_flag <= 1;
		//tx_vals_flag <= 0;
		
	end else if ( spi_slave_start_strobe ) begin
		// Set the command byte if it's the first received byte.
		spi_slave_byte_count <= 0;
		rx_vals_flag <= 0;
		//tx_vals_flag <= 0;

	end else begin
		if ( spi_slave_byte_done ) begin
			// For everything else, increment the byte counter & load the TX/RX buffers with the correct bytes based upon the first received byte
			spi_slave_byte_count <= spi_slave_byte_count + 1;
			// Place the received one in the request buffer
			spi_slave_req_buf[ spi_slave_byte_count ] <= spi_slave_do;

			if ( spi_slave_byte_count == 1 ) begin
			// // 	// If we just received our first byte, set the flag to load the proper bytes that we'll send out
			//	tx_vals_flag <= 1;
			command_byte_l <= command_byte;
			command_rw_l <= command_rw;
			end
		//end else begin
			// For the rest of the bytes, set the next outgoing byte
			//spi_slave_di <= spi_slave_res_buf[ spi_slave_byte_count ];
			
			//	spi_slave_byte_count <= spi_slave_byte_count + 1;
			//tx_vals_flag <= 0;
		end

		rx_vals_flag <= 0;
	end

	// Always load the first byte as the command byte
 	// if (spi_slave_byte_done == 1 && spi_slave_byte_count == 0) begin
 		
 	// end
end



// We keep a delayed version of the byte_done flag from the SPI_Slave module for quickly loading the request type into the response buffer
always @( posedge sysclk )	spi_slave_byte_done_d <= spi_slave_byte_done;

always @( negedge sysclk )
begin : SPI_LOAD_RESPONSE_BUFFER
	
	// Always place the first response byte for an SPI transfer into the zero index of the response buffer
	spi_slave_res_buf[0] <= {2'b10, motors_en, hall_faults};
	//spi_slave_res_buf[0] <= {'b10, motors_en, 'b1, enc_a[0], enc_b[0], enc_a[1], enc_b[1]};
	watchdog_timer <= watchdog_timer + 1;

	// If the flag is set to load the response buffer, reset the flag & do just that. We know that the 'command_byte' is valid if this flag is set.
	if ( tx_vals_flag == 1 ) begin

		if ( command_rw == CMD_READ_TYPE ) begin
			// If the byte count is 2, we need to go back and decode our first byte so we know what data to send out for everything else
			case ( command_byte )
				// Send the encoder counts
				CMD_UPDATE_MTRS : 	
				begin
					// Encoder inputs are latched here so all readings are from the same time
					for (j = 0; j < NUM_ENCODERS; j = j + 1)
					begin : LATCH_ENC_COUNTS_ON_UPDATE
						spi_slave_res_buf[2*j+1]	<=	enc_count[j][ENCODER_COUNT_WIDTH-1:SPI_SLAVE_DATA_WIDTH];
						spi_slave_res_buf[2*j+2]	<=	enc_count[j][SPI_SLAVE_DATA_WIDTH-1:0];
					end

					spi_slave_res_buf[2*NUM_ENCODERS+1] <= watchdog_timer[WATCHDOG_TIMER_WIDTH - 1: (WATCHDOG_TIMER_WIDTH - SPI_SLAVE_DATA_WIDTH)];
					spi_slave_res_buf[2*NUM_ENCODERS+2] <= watchdog_timer[(WATCHDOG_TIMER_WIDTH - SPI_SLAVE_DATA_WIDTH - 1) : (WATCHDOG_TIMER_WIDTH - (2 * SPI_SLAVE_DATA_WIDTH))];
				end

				CMD_ENCODER_COUNT :
				begin
					// Encoder inputs are latched here so all readings are from the same time
					for (j = 0; j < NUM_ENCODERS; j = j + 1) 
					begin : LATCH_ENC_COUNTS
						spi_slave_res_buf[2*j+1] 	<=	enc_count[j][ENCODER_COUNT_WIDTH-1:SPI_SLAVE_DATA_WIDTH];
						spi_slave_res_buf[2*j+2] 	<=	enc_count[j][SPI_SLAVE_DATA_WIDTH-1:0];
					end

					spi_slave_res_buf[2*NUM_ENCODERS+1] <= watchdog_timer[WATCHDOG_TIMER_WIDTH - 1: (WATCHDOG_TIMER_WIDTH - SPI_SLAVE_DATA_WIDTH)];
					spi_slave_res_buf[2*NUM_ENCODERS+2] <= watchdog_timer[(WATCHDOG_TIMER_WIDTH - SPI_SLAVE_DATA_WIDTH - 1) : (WATCHDOG_TIMER_WIDTH - (2 * SPI_SLAVE_DATA_WIDTH))];
				end

				CMD_HALL_COUNT :
				begin
					// Encoder inputs are latched here so all readings are from the same time
					for (j = 0; j < NUM_HALL_SENS; j = j + 1) 
						spi_slave_res_buf[j+1] 	<=	hall_count[j][HALL_COUNT_WIDTH-1:0];
				end

				CMD_DUTY_CYCLE :
				begin
					// Latch the current duty cycles of the motors
					for ( j = 0; j < NUM_MOTORS; j = j + 1 ) 
						begin : UPDATE_DUTY_CYCLES
							spi_slave_res_buf[2*j+1] 	<=	duty_cycle[j][DUTY_CYCLE_WIDTH-1:SPI_SLAVE_DATA_WIDTH];
							spi_slave_res_buf[2*j+2] 	<=	duty_cycle[j][SPI_SLAVE_DATA_WIDTH-1:0];
						end
				end

`ifdef GIT_VERSION_HASH
				CMD_VERSION :
				begin

					{ spi_slave_res_buf[4], spi_slave_res_buf[3], spi_slave_res_buf[2], spi_slave_res_buf[1] }  <= `GIT_VERSION_HASH;
					spi_slave_res_buf[5] <= `GIT_VERSION_DIRTY;
				end
`endif
					
				default : 	
				begin
					// Default is to set everything to 0
					for (j = 0; j < (SPI_SLAVE_RES_BUF_LEN - 1); j = j + 1) 
					begin : RESET_RESPONSE_BUF_ON_READ_DEFAULT
						spi_slave_res_buf[j+1] 	<=	'hAA;
					end
				end

			endcase // command_byte - read
		end
	end // tx_vals_flag

end // SPI_LOAD_RESPONSE_BUFFER


always @( negedge sysclk )
begin : SPI_SORT_REQUEST_BUFFER

	if ( rx_vals_flag == 1 ) begin
		if ( command_rw_l == CMD_WRITE_TYPE ) begin
			// If the byte count is 2, we need to go back and decode our first byte so we know what data to send out for everything else
			case ( command_byte_l )
				CMD_TOGGLE_MOTOR_EN :
				begin
					// Only take action if we received exactly 1 command byte
					if ( spi_slave_byte_count == 0 ) begin
						motors_en <= 0;
					end
				end
			endcase // command_byte - write

		end else begin

			case ( command_byte_l )
				// Send the encoder counts
				CMD_UPDATE_MTRS : 	
				begin
					/* 
					 * Only update the duty cycles if the transfer is what we
					 * expected. The results in the real world could end badly
					 * if the user flips the top and low bytes of the duty 
					 * cycle, so don't do that.
					 */
					if ( (spi_slave_byte_count) == 2*NUM_MOTORS ) begin
						// Set the new duty_cycle values
						for ( j = 0; j < NUM_MOTORS; j = j + 1 ) 
						begin : UPDATE_DUTY_CYCLES
							duty_cycle[j][SPI_SLAVE_DATA_WIDTH-1:0] 				<=	spi_slave_req_buf[2*j+1];
							duty_cycle[j][DUTY_CYCLE_WIDTH-1:SPI_SLAVE_DATA_WIDTH]	<=	spi_slave_req_buf[2*j+2]; 	// The received data bytes start at index 1 (not 0)
						end
					end
				end

				CMD_TOGGLE_MOTOR_EN :
				begin
					// Only take action if we received exactly 1 command byte
					if ( spi_slave_byte_count == 0 ) begin
						motors_en <= 1;
					end
				end

			endcase

		end	// command_rw
	end

end 	// SPI_SORT_REQUEST_BUFFER

always @(posedge sysclk) begin
	if (watchdog_timer > (1 << WATCHDOG_TIMER_WIDTH) - 2) begin
		// for ( j = 0; j < NUM_MOTORS; j = j + 1 ) 
		// begin : WATCHDOG_EXPIRE
		// 	duty_cycle[j]	<=	0;
		// end
	end
end

// This handles all of the SPI bus communications to/from the motor board
// Motor_Board_Comm motor_board_comm_module();

// This is where all of the motors modules are instantiated
generate
	for (i = 0; i < NUM_MOTORS - 1; i = i + 1)
	begin : BLDC_MOTOR_INST
		BLDC_Motor #(
			.MAX_DUTY_CYCLE 	 	( ( 1 << DUTY_CYCLE_WIDTH ) - 1 ),
			.ENCODER_COUNT_WIDTH 	( ENCODER_COUNT_WIDTH 			),
			.HALL_COUNT_WIDTH 		( HALL_COUNT_WIDTH 				)
			) motor (
			.clk 				( sysclk 		),
			.en					( motors_en		),
			.reset_hall_count 	( 0				),
			.reset_enc_count 	( 0				),
			.duty_cycle			( duty_cycle[i]	), 
			.enc 				( enc_s[i] 		),
			//.enc 				( { enc_a[i], enc_b[i] } ),
			.hall 				( hall_s[i] 	),
			.phaseH 			( phaseH_o[i]	),
			.phaseL 			( phaseL_o[i]	), 
			.enc_count 			( enc_count[i] 	),
			.hall_count 		( hall_count[i] ),
			.hall_fault 		( hall_faults[i])
		);
	end
endgenerate

		// BLDC_Motor #(
		// 	.MAX_DUTY_CYCLE 	 	( ( 1 << DUTY_CYCLE_WIDTH ) - 1 ),
		// 	.ENCODER_COUNT_WIDTH 	( ENCODER_COUNT_WIDTH 			),
		// 	.HALL_COUNT_WIDTH 		( HALL_COUNT_WIDTH 				)
		// 	) drib_motor (
		// 	.clk 				( sysclk 		),
		// 	.en					( motors_en		),
		// 	.reset_hall_count 	( 0				),
		// 	.reset_enc_count 	( 0				),
		// 	.duty_cycle			( duty_cycle[NUM_MOTORS-1]	), 
		// 	.enc 				( 2'b00 		),
		// 	//.enc 				( { enc_a[i], enc_b[i] } ),
		// 	.hall 				( hall_s[NUM_MOTORS-1] 	),
		// 	.phaseH 			( phaseH_o[NUM_MOTORS-1]	),
		// 	.phaseL 			( phaseL_o[NUM_MOTORS-1]	), 
		// 	//.enc_count 			( enc_count[i] 	),
		// 	.hall_count 		( hall_count[NUM_MOTORS-1] ),
		// 	.hall_fault 		( hall_faults[NUM_MOTORS-1])
		// );

endmodule 	// RoboCup
