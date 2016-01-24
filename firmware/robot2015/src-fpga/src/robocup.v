// Top module for the FPGA logic

`include "BLDC_Motor.v"
`include "SPI_Slave.v"
`include "SPI_Master.v"
`include "ClkDivide.v"
`include "git_version.vh"

module robocup #(
    parameter       NUM_MOTORS              =   ( 5 ),
                    NUM_HALL_SENS           =   ( NUM_MOTORS ),
                    NUM_ENCODERS            =   ( NUM_MOTORS - 1 ),

                    SPI_MASTER_CPOL         =   ( 0 ),
                    SPI_MASTER_CPHA         =   ( 1 ),
                    SPI_MASTER_DATA_WIDTH   =   ( 16 ),

                    SPI_SLAVE_CPOL          =   ( 0 ),
                    SPI_SLAVE_CPHA          =   ( 0 ),
                    SPI_SLAVE_DATA_WIDTH    =   ( 8 )

    ) (
    // Clock
    input                               sysclk,

    // 3-half-bridge
    output reg  [ NUM_MOTORS - 1:0 ]    phase_aH,   phase_aL,   phase_bH,   phase_bL,   phase_cH,   phase_cL,

    // Hall effect sensors
    input       [ NUM_HALL_SENS - 1:0 ] hall_a,     hall_b,     hall_c,

    // Encoders
    input       [ NUM_ENCODERS - 1:0 ]  enc_a,
    input       [ NUM_ENCODERS - 1:0 ]  enc_b,

    // Phase driver chip select pins
    output reg  [ NUM_MOTORS - 1:0 ]    drv_ncs,

    // ADC chip select pins
    output reg  [ 1:0 ]                 adc_ncs,

    // SPI Slave pins
    input                               spi_slave_sck,          spi_slave_mosi,         spi_slave_ncs,
    output                              spi_slave_miso,

    // SPI Master pins
    input                               spi_master_miso,
    output reg                          spi_master_sck,         spi_master_mosi
);

// THIS MUST BE INCLUDED RIGHT AFTER THE MODULE DECLARATION
`include "log2-macro.v"

// Declare variables for synthesis that is used for generating modules within loops.
// Note that this is only used during synthesis (ie. compilation).
genvar i;
integer j, k;

// Derived parameters 
localparam ENCODER_COUNT_WIDTH          =   ( 16 );
localparam HALL_COUNT_WIDTH             =   ( (3 * SPI_SLAVE_DATA_WIDTH) - ENCODER_COUNT_WIDTH );
localparam DUTY_CYCLE_WIDTH             =   ( 10 );
localparam NUM_ENCODER_GEN              =   ( NUM_ENCODERS );
localparam NUM_MOTORS_GEN               =   ( NUM_MOTORS );
localparam STARTUP_DELAY_WIDTH          =   (  3 );

// To calculate the watchdog timer's expire time, use the following equation:
// (1/<freq-of-sysclk>) * (2^WATCHDOG_TIMER_CLK_WIDTH) * (2^WATCHDOG_TIMER_WIDTH)
// `<freq-of-sysclk>` will usually be: (18.432*10^6)
// When WATCHDOG_TIMER_CLK_WIDTH = 5, and WATCHDOG_TIMER_WIDTH = 16, the expire time is ~114ms
localparam WATCHDOG_TIMER_CLK_WIDTH     =   (  5 );
localparam WATCHDOG_TIMER_WIDTH         =   ( 16 );

reg sys_rdy = 0;


// Input register synchronization declarations - for the notation, we add a '_s' after the name indicating it is synced
reg [ 2:0 ] hall_s  [ NUM_HALL_SENS - 1 : 0 ];
reg [ 1:0 ] enc_s   [ NUM_ENCODERS  - 1 : 0 ];
reg         spi_slave_sck_s,    
            spi_slave_mosi_s,
            spi_slave_ncs_s, 
            spi_slave_ncs_d,    // we have a delayed version of the chip select line
            spi_master_miso_s;

// Sync all of the input pins here into registers for reducing errors resulting noise
always @( posedge sysclk )
begin : SYNC_INPUTS
    // Hall inputs
    for (j = 0; j < NUM_HALL_SENS; j = j + 1)
    begin : GEN_HALL_ARRAY
        hall_s[j]       <=  { hall_a[j], hall_b[j], hall_c[j] };
    end
    // Encoder inputs
    for (j = 0; j < NUM_ENCODERS; j = j + 1) 
    begin : GEN_ENC_ARRAY
        enc_s[j]        <=  { enc_a[j], enc_b[j] };
    end
    // SPI slave inputs
    spi_slave_sck_s     <=  spi_slave_sck;
    spi_slave_mosi_s    <=  spi_slave_mosi;
    spi_slave_ncs_s     <=  spi_slave_ncs;
    spi_slave_ncs_d     <=  spi_slave_ncs_s;    // the delayed register is always set from what the synced register is for the slave ncs line
    // SPI master inputs
    spi_master_miso_s   <=  spi_master_miso;
end


// Output register synchronization declarations - for the notation, we add a '_o' after the name indicating it will got to an output pin
wire [ 2:0 ]                phaseH_o [ NUM_MOTORS - 1:0 ],
                            phaseL_o [ NUM_MOTORS - 1:0 ];
wire [ NUM_MOTORS - 1:0 ]   drv_ncs_o;
wire [ 1:0 ]                adc_ncs_o;
wire                        spi_slave_miso_o, 
                            spi_master_sck_o, 
                            spi_master_mosi_o;

// Sync all of the output pins for the same reasons we sync all of the input pins - this time in reverse
always @( posedge sysclk )
begin : SYNC_OUTPUTS
    for ( j= 0; j < NUM_MOTORS; j = j + 1 ) 
    begin : GEN_PHASE_ARRAY
        // Phase outputs (HIGH)
        { phase_aH[j], phase_bH[j], phase_cH[j] }   <=  phaseH_o[j];
        // Phase outputs (LOW)
        { phase_aL[j], phase_bL[j], phase_cL[j] }   <=  phaseL_o[j];
    end

    drv_ncs             <=  drv_ncs_o;
    adc_ncs             <=  adc_ncs_o;
    spi_master_sck      <=  spi_master_sck_o;
    spi_master_mosi     <=  spi_master_mosi_o;
end

// Small startup delay
reg [STARTUP_DELAY_WIDTH-1:0]   start_delay_r       = 0;
wire                            start_delay_done    = ( start_delay_r == ((1<<STARTUP_DELAY_WIDTH)-1) );
always @(posedge sysclk) start_delay_r <= start_delay_r + 1;

// We only drive the slave's data output line if we are selected
assign spi_slave_miso = ( spi_slave_ncs_s == 1 ? 1'bZ : spi_slave_miso_o );

// Internal logic declarations
wire [ ENCODER_COUNT_WIDTH  - 1:0 ] enc_count        [ NUM_ENCODERS  - 1:0 ];
wire [ HALL_COUNT_WIDTH     - 1:0 ] hall_count       [ NUM_HALL_SENS - 1:0 ];
wire [ NUM_HALL_SENS        - 1:0 ] hall_conns;
wire [ NUM_HALL_SENS        - 1:0 ] hall_faults;
reg  [ DUTY_CYCLE_WIDTH     - 1:0 ] duty_cycle       [ NUM_MOTORS    - 1:0 ];
reg  [ WATCHDOG_TIMER_WIDTH - 1:0 ] watchdog_timer   [1:0];

// This gets set on a watchdog timer overflow
reg watchdog_trigger = 0;
// Watchdog timer clock
wire wdtclk;
reg [1:0] wdtclk_sr = 0; always @(posedge sysclk) wdtclk_sr <= {wdtclk_sr[0], wdtclk};
wire wdtclk_rising_edge = ( wdtclk_sr == 2'b01 );

// Watchdog timer divided clock
ClkDivide #(
  .WIDTH    ( WATCHDOG_TIMER_CLK_WIDTH  )
  ) wdtclk_gen (
  .CLK_IN   ( sysclk                    ),
  .EN       ( sys_rdy                   ),
  .CLK_OUT  ( wdtclk                    )
);


// Command types for SPI access
localparam CMD_UPDATE_MTRS      = 0;
// The command types beginning at 0x10 have selectable read/write types according to the command's MSB.
localparam CMD_WRITE_TYPE       = 0;
localparam CMD_READ_TYPE        = 1;
localparam CMD_RW_TYPE_BASE     = 'h10;
localparam CMD_ENCODER_COUNT    = CMD_RW_TYPE_BASE + 1;
localparam CMD_HALL_COUNT       = CMD_RW_TYPE_BASE + 2;
localparam CMD_DUTY_CYCLE       = CMD_RW_TYPE_BASE + 3;
localparam CMD_VERSION          = CMD_RW_TYPE_BASE + 4;
// The command strobes start after the read/write command types
localparam CMD_STROBE_START         = CMD_RW_TYPE_BASE + 'h10;
localparam CMD_TOGGLE_MOTOR_EN      = CMD_RW_TYPE_BASE + CMD_STROBE_START;
// Response & request buffer sizes
localparam SPI_SLAVE_RES_BUF_LEN = 15;
localparam SPI_SLAVE_REQ_BUF_LEN = SPI_SLAVE_RES_BUF_LEN;
localparam SPI_SLAVE_COUNTER_WIDTH = `LOG2(SPI_SLAVE_RES_BUF_LEN);

// These are for triggering the storage of values outside of the SPI's SCK domain
reg                                     rx_vals_flag            = 0,
                                        motors_en               = 0,
                                        spi_slave_byte_done_d   = 0;
reg [ SPI_SLAVE_COUNTER_WIDTH - 1:0 ]   spi_slave_byte_count    = 0;
reg [ SPI_SLAVE_DATA_WIDTH - 1:0 ]      spi_slave_res_buf [ SPI_SLAVE_RES_BUF_LEN - 1:0 ],      // response & request buffers
                                        spi_slave_req_buf [ SPI_SLAVE_REQ_BUF_LEN - 1:0 ];

wire [ SPI_SLAVE_DATA_WIDTH - 1:0 ] spi_slave_do;
wire spi_slave_start_flag = ( (spi_slave_ncs_s == 0) && (spi_slave_ncs_d == 1) ),
     spi_slave_end_flag = ( (spi_slave_ncs_s == 1) && (spi_slave_ncs_d == 0) );
wire spi_slave_byte_done;

reg                                     spi_master_start = 0;
reg [ NUM_MOTORS - 1:0 ]                spi_master_sel_num;
reg [ NUM_MOTORS - 1:0 ]                spi_master_sel;
wire                                    spi_master_sel_now;
wire                                    spi_master_busy,
                                        spi_master_valid;
wire [ SPI_MASTER_DATA_WIDTH - 1:0 ]    spi_master_d0,
                                        spi_master_di;

reg [1:0] spi_master_busy_sr = 0;  always @(posedge sysclk) spi_master_busy_sr <= {spi_master_busy_sr[0], spi_master_busy};
wire spi_master_trxfr_done_flag   =   ( spi_master_busy_sr == 2'b10 );

always@(posedge sysclk) spi_master_sel[spi_master_sel_num] <= ~spi_master_sel_now;
assign drv_ncs_o = ~spi_master_sel;

SPI_Master spi_master_module (
    .clk            ( sysclk                ),
    .EN             ( sys_rdy               ),
    .SCK            ( spi_master_sck_o      ),
    .MOSI           ( spi_master_mosi_o     ),
    .MISO           ( spi_master_miso_s     ),
    .SEL            ( spi_master_sel_now    ),
    .START          ( spi_master_start      ),
    .BUSY           ( spi_master_busy       ),
    .VALID          ( spi_master_valid      ),
    .DATA_OUT       ( spi_master_d0         ),
    .DATA_IN        ( spi_master_di         )
);

reg [SPI_MASTER_DATA_WIDTH-1:0] motor_config_regs [2:0];
reg [1:0] data_received_index = 0;

assign spi_master_di = motor_config_regs[data_received_index];

always @(posedge sysclk)
begin : INIT_SYSTEM
    // Init at startup
    if ( start_delay_done == 1 ) begin
        if ( sys_rdy == 0 ) begin
            sys_rdy <= 1;
        end
    end
end

always @(posedge sysclk)
begin : SPI_MASTER_COMM
    // if spi master transfer complete
    if ( sys_rdy == 0 ) begin
        spi_master_start <= 0;
        spi_master_sel_num <= 2;
        motor_config_regs[0] <= (2'h2 << 11) | 'h0380;
        motor_config_regs[1] <= (3<<11) & 'h7800;

    end else if ( spi_master_trxfr_done_flag == 1 ) begin
        if ( spi_master_valid == 1 ) begin

            spi_master_start <= 1;

            if (data_received_index == 2) begin
                data_received_index <= 0;
                spi_master_sel_num <= spi_master_sel_num + 1;

            end else begin
                data_received_index <= data_received_index + 1;
            end

            if ( spi_master_sel_num >= (NUM_MOTORS - 1) ) begin
                spi_master_sel_num <= 0;
            end
        end
    end
end

wire command_byte_rdy = ( ( spi_slave_byte_count == 1 ) && ( spi_slave_byte_done_d == 1 ) );
wire [ SPI_SLAVE_DATA_WIDTH - 1:0 ]  spi_slave_di = spi_slave_res_buf[ spi_slave_byte_count ];
wire [ SPI_SLAVE_DATA_WIDTH - 2:0 ] command_byte = command_byte_rdy ? spi_slave_do[SPI_SLAVE_DATA_WIDTH - 2:0] : spi_slave_req_buf[0][SPI_SLAVE_DATA_WIDTH - 2:0];
wire                                command_rw   = command_byte_rdy ? spi_slave_do[SPI_SLAVE_DATA_WIDTH - 1]   : spi_slave_req_buf[0][SPI_SLAVE_DATA_WIDTH - 1];
reg [ SPI_SLAVE_DATA_WIDTH - 2:0 ] command_byte_l = 0;
reg command_rw_l = 0;

// SPI Slave module
SPI_Slave spi_slave_module (
    .clk            ( sysclk                ),
    .SCK            ( spi_slave_sck         ),
    .MOSI           ( spi_slave_mosi_s      ),
    .MISO           ( spi_slave_miso_o      ),
    .SSEL           ( spi_slave_ncs_s       ),
    .DONE           ( spi_slave_byte_done   ),
    .DATA_IN        ( spi_slave_do          ),
    .DATA_OUT       ( spi_slave_di          )
);

// When the byte count changes, we need to find our next byte that we want to load for the data output
always @( posedge sysclk )
begin : SPI_SLAVE_LOAD_BYTE
    // If the chip select line is toggled and it is now high, we are ending an SPI transfer, so reset everything & take action with what we received
    if ( spi_slave_end_flag ) begin
        // Signal to do something with the received bytes & save how may bytes were received. We do this here so it will happen after we set the received byte count
        rx_vals_flag <= 1;
        
    end else if ( spi_slave_start_flag ) begin
        // Set the command byte if it's the first received byte.
        spi_slave_byte_count <= 0;
        rx_vals_flag <= 0;

    end else begin
        if ( spi_slave_byte_done ) begin
            // For everything else, increment the byte counter & load the TX/RX buffers with the correct bytes based upon the first received byte
            spi_slave_byte_count <= spi_slave_byte_count + 1;
            // Place the received one in the request buffer
            spi_slave_req_buf[ spi_slave_byte_count ] <= spi_slave_do;
        end

        if ( spi_slave_byte_count == 1 ) begin
            // We want to latch this value past when spi_slave_byte_done goes back low
            command_byte_l <= command_byte;
            command_rw_l <= command_rw;
        end

        rx_vals_flag <= 0;
    end
end

// We keep a delayed version of the byte_done flag from the SPI_Slave module for quickly loading the request type into the response buffer
always @( posedge sysclk )  spi_slave_byte_done_d <= spi_slave_byte_done;

reg motor_update_flag = 0;

always @( negedge sysclk )
begin : SPI_SLAVE_LOAD_RESPONSE_BUFFER
    // Always place the first response byte for an SPI transfer into the zero index of the response buffer
    spi_slave_res_buf[0] <= {sys_rdy, watchdog_trigger, motors_en, hall_conns};
    watchdog_timer[1] <= watchdog_timer[0];

    // If the flag is set to load the response buffer, reset the flag & do just that. We know that the 'command_byte' is valid if this flag is set.
    if ( command_byte_rdy == 1 ) begin
        if ( command_rw == CMD_READ_TYPE ) begin
            // If the byte count is 2, we need to go back and decode our first byte so we know what data to send out for everything else
            case ( command_byte )
                // Send the encoder counts
                CMD_UPDATE_MTRS :   
                begin
                    // Encoder inputs are latched here so all readings are from the same time
                    for (j = 0; j < NUM_ENCODERS; j = j + 1)
                    begin : LATCH_ENC_COUNTS_ON_UPDATE
                        spi_slave_res_buf[2*j+1]    <=  enc_count[j][ENCODER_COUNT_WIDTH-1:SPI_SLAVE_DATA_WIDTH];
                        spi_slave_res_buf[2*j+2]    <=  enc_count[j][SPI_SLAVE_DATA_WIDTH-1:0];
                    end
                    // The latched watchdog timer count
                    spi_slave_res_buf[2*NUM_ENCODERS+1] <= {0, watchdog_timer[1][WATCHDOG_TIMER_WIDTH - 1: (WATCHDOG_TIMER_WIDTH - SPI_SLAVE_DATA_WIDTH + 1)]};
                    spi_slave_res_buf[2*NUM_ENCODERS+2] <= watchdog_timer[1][(WATCHDOG_TIMER_WIDTH - SPI_SLAVE_DATA_WIDTH - 0) :0];
                    motor_update_flag <= 1;
                end

                CMD_ENCODER_COUNT :
                begin
                    // Encoder inputs are latched here so all readings are from the same time
                    for (j = 0; j < NUM_ENCODERS; j = j + 1) 
                    begin : LATCH_ENC_COUNTS
                        spi_slave_res_buf[2*j+1]     <=  enc_count[j][ENCODER_COUNT_WIDTH-1:SPI_SLAVE_DATA_WIDTH];
                        spi_slave_res_buf[2*j+2]     <=  enc_count[j][SPI_SLAVE_DATA_WIDTH-1:0];
                    end
                    // The latched watchdog timer count
                    spi_slave_res_buf[2*NUM_ENCODERS+1] <= {0, watchdog_timer[1][WATCHDOG_TIMER_WIDTH - 1: (WATCHDOG_TIMER_WIDTH - SPI_SLAVE_DATA_WIDTH + 1)]};
                    spi_slave_res_buf[2*NUM_ENCODERS+2] <= watchdog_timer[1][(WATCHDOG_TIMER_WIDTH - SPI_SLAVE_DATA_WIDTH - 0) :0];
                end

                CMD_HALL_COUNT :
                begin
                    // Encoder inputs are latched here so all readings are from the same time
                    for (j = 0; j < NUM_HALL_SENS; j = j + 1)
                    begin : LATCH_HALL_COUNTS
                        spi_slave_res_buf[j+1]  <=  hall_count[j][HALL_COUNT_WIDTH-1:0];
                    end
                end

                CMD_DUTY_CYCLE :
                begin
                    // Latch the current duty cycles of the motors
                    for (j = 0; j < NUM_MOTORS; j = j + 1)
                    begin : UPDATE_DUTY_CYCLES
                        spi_slave_res_buf[2*j+1]    <=  duty_cycle[j][DUTY_CYCLE_WIDTH-1:SPI_SLAVE_DATA_WIDTH];
                        spi_slave_res_buf[2*j+2]    <=  duty_cycle[j][SPI_SLAVE_DATA_WIDTH-1:0];
                    end
                end

`ifdef GIT_VERSION_HASH
                CMD_VERSION :
                begin
                    for (j = 0; j < 7; j = j + 1)
                    begin : LATCH_GIT_HASH
                        spi_slave_res_buf[j+1]    <=  (`GIT_VERSION_HASH >> j) & 'hF;
                    end
                    spi_slave_res_buf[9] <= `GIT_VERSION_DIRTY;
                end
`endif
                    
                default :   
                begin
                    // Default is to set everything in the response buffer to 0xAA. This makes is a bit easier to debug the SPI protocol.
                    for (j = 0; j < (SPI_SLAVE_RES_BUF_LEN - 1); j = j + 1) 
                    begin : RESET_RESPONSE_BUF_ON_READ_DEFAULT
                        spi_slave_res_buf[j+1]  <=  'hAA;
                    end
                end
            endcase // command_byte - read
        end
    end else begin
        motor_update_flag <= 0;
    end
end // SPI_LOAD_RESPONSE_BUFFER


always @( negedge sysclk )
begin : SPI_SORT_REQUEST_BUFFER

    if ( sys_rdy == 0 ) begin
        // enable the motors once the system is ready
        motors_en <= 1;

    end else if ( watchdog_trigger == 1 ) begin
        motors_en <= 0;

    end else if ( rx_vals_flag == 1 ) begin
        if ( command_rw_l == CMD_WRITE_TYPE ) begin
            // If the byte count is 2, we need to go back and decode our first byte so we know what data to send out for everything else
            case ( command_byte_l )
                CMD_TOGGLE_MOTOR_EN :
                begin
                    // Only take action if we received exactly 1 extra byte
                    if ( spi_slave_byte_count == 1 ) begin
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
                    if ( ( spi_slave_byte_count) == 2*NUM_MOTORS ) begin
                        // Set the new duty_cycle values
                        for ( j = 0; j < NUM_MOTORS; j = j + 1 )
                        begin : UPDATE_DUTY_CYCLES
                            duty_cycle[j][SPI_SLAVE_DATA_WIDTH-1:0]                 <=  spi_slave_req_buf[2*j+1];
                            duty_cycle[j][DUTY_CYCLE_WIDTH-1:SPI_SLAVE_DATA_WIDTH]  <=  spi_slave_req_buf[2*j+2];   // The received data bytes start at index 1 (not 0)
                        end

                        motors_en <= 1;
                    end
                end

                CMD_TOGGLE_MOTOR_EN :
                begin
                    // Only take action if we received exactly 1 extra byte
                    if ( spi_slave_byte_count == 1 ) begin
                        motors_en <= 1;
                    end
                end
            endcase
        end // command_rw
    end
end     // SPI_SORT_REQUEST_BUFFER


// Signal that is active when the watchdog timer should be reset
reg [1:0] motors_en_sr = 0; always @(posedge sysclk) motors_en_sr <= {motors_en_sr[0], motors_en};
// Toggle the motors on/off, off/on, or set the motor_update_flag to reset the watchdog timer
wire watchdog_timer_reset_flag = (( motors_en_sr == 2'b01 ) && ( motors_en_sr == 2'b10 ) ) || ( motor_update_flag == 1 );

// Watchdog timer
always @( posedge sysclk )
begin : WATCHDOG
    if ( sys_rdy == 0 ) begin
        watchdog_timer[0] <= 0;
    end else begin
        // Reset the timer if flag is set
        if ( watchdog_timer_reset_flag == 1 ) begin
            watchdog_timer[0] <= 0;
            watchdog_trigger <= 0;
        end else if ( wdtclk_rising_edge == 1 ) begin
            if (watchdog_timer[0] > (1 << WATCHDOG_TIMER_WIDTH) - 2) begin
                watchdog_trigger <= 1;
                watchdog_timer[0] <= 0;
            end else begin
                // Increment at every clock cycle from the watchdog timer's clock
                watchdog_timer[0] <= watchdog_timer[0] + 1;
            end
        end
    end
end


// This is where all of the motors modules are instantiated
generate
    for (i = 0; i < NUM_MOTORS - 1; i = i + 1)
    begin : BLDC_MOTOR_INST
        BLDC_Motor #(
            .MAX_DUTY_CYCLE         ( ( 1 << DUTY_CYCLE_WIDTH ) - 1 ),
            .ENCODER_COUNT_WIDTH    ( ENCODER_COUNT_WIDTH           ),
            .HALL_COUNT_WIDTH       ( HALL_COUNT_WIDTH              )
            ) motor (
            .clk                    ( sysclk        ),
            .en                     ( motors_en && sys_rdy ),
            .reset_hall_count       ( ~hall_conns[i] ),
            .reset_enc_count        ( 0             ),
            .duty_cycle             ( duty_cycle[i] ), 
            .enc                    ( enc_s[i]      ),
            .hall                   ( hall_s[i]     ),
            .phaseH                 ( phaseH_o[i]   ),
            .phaseL                 ( phaseL_o[i]   ), 
            .enc_count              ( enc_count[i]  ),
            .hall_count             ( hall_count[i] ),
            .connected              ( hall_conns[i] ),
            .hall_fault             ( hall_faults[i])
        );
    end
endgenerate

endmodule   // RoboCup
