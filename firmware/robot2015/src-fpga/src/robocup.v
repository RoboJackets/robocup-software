
`include "BLDC_Motor.v"
`include "SPI_Slave.v"
`include "Single_Port_RAM_Sync.v"
`include "git_version.vh"

`define NUM_MOTORS 		5
`define NUM_HALL_SENS 	`NUM_MOTORS
`define NUM_ENCODERS 	5


module robocup (
	// Clock
	input 					sysclk,

	// 3-half-bridge
	output reg	[ `NUM_MOTORS - 1:0 ]		phase_aH, 	phase_aL, 	phase_bH, 	phase_bL, 	phase_cH, 	phase_cL,

	// Hall effect sensors
	input 		[ `NUM_HALL_SENS - 1:0 ]	hall_a, 	hall_b, 	hall_c,

	// Encoders
	input 		[ `NUM_ENCODERS - 1:0 ] 	enc_a, 		enc_b,

	// Phase driver chip select pins
	output reg	[ `NUM_MOTORS - 1:0 ]		drv_ncs,

	// ADC chip select pins
	output reg	[ 1:0 ] 	adc_ncs,

	// SPI Slave pins
	input 					spi_slave_sck, 			spi_slave_mosi, 		spi_slave_ncs,
	inout 					spi_slave_miso,

	// SPI Master pins
	output reg				spi_master_sck, 		spi_master_mosi,
	input 					spi_master_miso
);

function integer log2;
  input integer value;
  begin
    value = value-1;
    for (log2=0; value>0; log2=log2+1)
      value = value>>1;
  end
endfunction

// Parameter that define specifications for the spi slave module
localparam SPI_SLAVE_DATA_WIDTH			=	( 8 );
localparam SPI_SLAVE_CLK_DIVIDER_WIDTH	=	( 4 );
localparam SPI_SLAVE_CPOL 				= 	( 1'b0 );
localparam SPI_SLAVE_CPHA 				= 	( 1'b0 );

localparam ENCODER_COUNT_WIDTH 			= 	( 12 );
localparam HALL_COUNT_WIDTH 			= 	( 4  );
localparam DUTY_CYCLE_WIDTH 			= 	( 10 );

localparam NUM_MTRS_GEN 				= 	( `NUM_MOTORS );

// Declare a variable for synthesis that is used for generating modules within loops.
// Note that this is only used during synthesis (ie. compilation).
genvar i;
integer j;


// Input register synchronization declarations - for the notation, we add a '_s' after the name indicating it is synced
reg	[ 2:0 ] hall_s 	[ `NUM_HALL_SENS - 1 : 0 ];
reg	[ 1:0 ] enc_s 	[ `NUM_ENCODERS  - 1 : 0 ];
reg			spi_slave_sck_s, 	
			spi_slave_mosi_s,
			spi_slave_ncs_s,
			spi_master_miso_s;

// Sync all of the input pins here into registers for decreasing any errors from noise
always@(posedge sysclk)
begin : SYNC_INPUTS
	// Hall inputs
	for (j = 0; j < `NUM_HALL_SENS; j = j + 1)
	begin : GEN_HALL_ARRAY

		hall_s[j]		<=	{ hall_a[j], hall_b[j], hall_c[j] };
	end
	// Encoder inputs
	for (j = 0; j < `NUM_ENCODERS; j = j + 1) 
	begin : GEN_ENC_ARRAY

		enc_s[j]		<=	{ enc_a[j], enc_b[j] };
	end
	// SPI slave inputs
	spi_slave_sck_s		<=	spi_slave_sck;
	spi_slave_mosi_s	<=	spi_slave_mosi;
	spi_slave_ncs_s		<=	spi_slave_ncs;
	// SPI master inputs
	spi_master_miso_s	<=	spi_master_miso;

end



// Output register synchronization declarations - for the notation, we add a '_o' after the name indicating it will got to an output pin
wire [ 2:0 ] 				phaseH_o [ `NUM_MOTORS - 1:0 ],
			 				phaseL_o [ `NUM_MOTORS - 1:0 ];
wire [ `NUM_MOTORS - 1:0 ] 	drv_ncs_o;
wire [ 1:0 ] 				adc_ncs_o;
wire 						spi_slave_miso_o, 
							spi_master_sck_o, 
							spi_master_mosi_o;

// Now we iterate over all the motors and define how the 2D array makes its connections
/*
generate
for (i = 0; i < NUM_MTRS_GEN; i = i + 1)
	begin : SET_PHASE_WIRES
		assign phaseH_o[i] = { phase_aH[i], phase_bH[i], phase_cH[i] };
		assign phaseL_o[i] = { phase_aL[i], phase_bL[i], phase_cL[i] };
	end
endgenerate
*/

// Only drive the slave output if we are selected
reg    spi_slave_miso_o_s;
assign spi_slave_miso = spi_slave_ncs_s ? 1'bz : spi_slave_miso_o_s;

// Sync all of the output pins for the same reasons we sync all of the input pins - this time in reverse
always@(posedge sysclk)
begin : SYNC_OUTPUTS
	for (j= 0; j < `NUM_ENCODERS; j = j+ 1) 
	begin : GEN_PHASE_ARRAY
		// Phase outputs (HIGH)
		{ phase_aH[j], phase_bH[j], phase_cH[j] }	<=	phaseH_o[j];
		// Phase outputs (LOW)
		{ phase_aL[j], phase_bL[j], phase_cL[j] }	<=	phaseL_o[j];
	end

	drv_ncs 			<= 	drv_ncs_o;
	adc_ncs 			<= 	adc_ncs_o;
	spi_slave_miso_o_s 	<= 	spi_slave_miso_o;
	spi_master_sck 		<= 	spi_master_sck_o;
	spi_master_mosi 	<= 	spi_master_mosi_o;
end


// We have our own register leading into the SPI Slave module so we have full control to
// ensure the data is setup before we trigger it to start.
reg spi_slave_select = 1'b1;

wire [ SPI_SLAVE_DATA_WIDTH - 1:0 ] spi_slave_do;
reg [ SPI_SLAVE_DATA_WIDTH - 1:0 ] spi_slave_di;

always @( spi_slave_ncs_s )
begin : SPI_CHANGE_EVENT

	if ( spi_slave_ncs_s ) begin
		// The sensitivity list event was a rising edge on 'spi_slave_ncs_s'
		spi_slave_select <= spi_slave_ncs_s;
		spi_slave_di <= 8'h00;
	end else begin
		// The sensitivity list event was a falling edge on 'spi_slave_ncs_s'
		spi_slave_select <= spi_slave_ncs_s;
		spi_slave_di <= 8'hff;	// change this to a status byte, but for now, it's constant for completeness
	end
end



// State names for a FSM that sets up the SPI data by reading/writing memory locations
localparam STATE_IDLE 	= 0;
localparam STATE_READ 	= 1;
localparam STATE_WRITE 	= 2;

// Command types for SPI access
localparam CMD_UPDATE_MTRS		= 0;
localparam CMD_READ_MTR_1_DATA 	= 1;
localparam CMD_READ_MTR_2_DATA 	= 2;
localparam CMD_READ_MTR_3_DATA 	= 3;
localparam CMD_READ_MTR_4_DATA 	= 4;
localparam CMD_READ_MTR_5_DATA 	= 5;

// Internal logic declarations
wire [ ENCODER_COUNT_WIDTH - 1:0 ] enc_count  [ `NUM_ENCODERS  - 1:0 ];
wire [ HALL_COUNT_WIDTH	   - 1:0 ] hall_count [ `NUM_HALL_SENS - 1:0 ];
wire [ DUTY_CYCLE_WIDTH	   - 1:0 ] duty_cycle [ `NUM_MOTORS    - 1:0 ];
wire [ `NUM_HALL_SENS 	   - 1:0 ] hall_faults;


// Synchronous memory for placing readings where they can be read from later
wire [7:0] mem_datao;
Single_Port_RAM_Sync #(
	.ADDR_WIDTH 	( 5 ),
	.DATA_WIDTH 	( 8 )
	)
	ram_mem
	(
	.clk 	( sysclk ),
	.we 	( 1'b0 ),
	.addr 	( 5'b00001 ),
	.din 	( 8'h01 ),
	.dout 	( mem_datao )
);



// Just keeping these here to make sure the for loops play well with ISE WebPACK first
// wire [1:0] enc 		[ `NUM_ENCODERS  - 1 : 0 ] = { enc_a_sync[i], 	enc_b_sync[i] };
// wire [2:0] hall 	[ `NUM_HALL_SENS - 1 : 0 ] = { hall_a[i], 		hall_b[i], 		hall_c[i] };
// wire [5:0] phaseOut	[ `NUM_MOTORS 	 - 1 : 0 ] = { phase_aH[i], 	phase_aL[i], 	phase_bH[i], 	phase_bL[i], 	phase_cH[i], 	phase_cL[i]	};


// This is the interface used for communications back to the mbed
SPI_Slave #(
	.DATA_WIDTH 	( SPI_SLAVE_DATA_WIDTH )
	) spi_slave	(
	.CPOL 			( SPI_SLAVE_CPOL 	),
	.CPHA 			( SPI_SLAVE_CPHA 	),
	.datai 			( spi_slave_di 		),
	.datao 			( spi_slave_do 		),
	.sclk 			( spi_slave_sck_s 	),
	.csb 			( spi_slave_select 	),
	.din 			( spi_slave_mosi_s 	),
	.dout 			( spi_slave_miso_o 	)
);


// This handles all of the SPI bus communications to/from the motor board
// Motor_Board_Comm motor_board_comm_module();


// This is where all of the motors modules are instantiated
generate
	for (i = 0; i < NUM_MTRS_GEN; i = i + 1)
	begin : BLDC_MOTOR_INST
		BLDC_Motor #(
			.ENCODER_COUNT_WIDTH 	( ENCODER_COUNT_WIDTH ),
			.HALL_COUNT_WIDTH 		( HALL_COUNT_WIDTH )
			) motor (
			.clk 			( sysclk 	),
			.en				( 1'b1 		),
			.reset_counts 	( 1'b1 		), 
			.duty_cycle		( duty_cycle[i]	), 
			.enc 			( enc_s[i] 		),
			.hall 			( hall_s[i] 	),
			.phaseH 		( phaseH_o[i]	),
			.phaseL 		( phaseL_o[i]	), 
			.enc_count 		( enc_count[i] 	),
			.hall_count 	( hall_count[i] ),
			.hall_fault 	( hall_faults[i])
		);
	end
endgenerate

endmodule 	// RoboCup
