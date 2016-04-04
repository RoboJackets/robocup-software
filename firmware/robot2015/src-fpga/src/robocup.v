// Top module for the FPGA logic

`include "BLDC_Motor.v"
`include "SPI_Slave.v"
`include "SPI_Master.v"
`include "ClkDivide.v"
`include "git_version.vh"

module robocup #(
    parameter       NUM_MOTORS              =   ( 5                 ) ,
                    NUM_HALL_SENS           =   ( NUM_MOTORS        ) ,
                    NUM_ENCODERS            =   ( NUM_MOTORS - 1    ) ,
                    SPI_MASTER_DATA_WIDTH   =   ( 16                ) ,
                    SPI_SLAVE_DATA_WIDTH    =   ( 8                 )

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
localparam HALL_COUNT_WIDTH             =   (  8 );
localparam DUTY_CYCLE_WIDTH             =   (  9 );
localparam STARTUP_DELAY_WIDTH          =   (  5 );

// To calculate the watchdog timer's expire time, use the following equation:
// (1/<freq-of-sysclk>) * (2^WATCHDOG_TIMER_CLK_WIDTH) * (2^WATCHDOG_TIMER_WIDTH)
// `<freq-of-sysclk>` will usually be: (18.432*10^6)
// When WATCHDOG_TIMER_CLK_WIDTH = 5, and WATCHDOG_TIMER_WIDTH = 16, the expire time is ~114ms
localparam WATCHDOG_TIMER_CLK_WIDTH     =   (  5 );
localparam WATCHDOG_TIMER_WIDTH         =   ( 16 );     // KEEP THIS AT 16 to make things easy for the SPI_Slave

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
    // SPI Master outputs
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
reg  [ DUTY_CYCLE_WIDTH     - 1:0 ] duty_cycle       [ NUM_MOTORS    - 1:0 ];
reg  [ WATCHDOG_TIMER_WIDTH - 1:0 ] watchdog_timer   [1:0];

// This gets set on a watchdog timer overflow
reg watchdog_trigger = 0;
// Watchdog timer clock
wire wdt_clk;
reg [1:0] wdt_clk_sr = 0; always @(posedge sysclk) wdt_clk_sr <= {wdt_clk_sr[0], wdt_clk};
wire wdt_clk_rising_edge = ( wdt_clk_sr == 2'b01 );

// Watchdog timer divided clock
ClkDivide #(
  .WIDTH    ( WATCHDOG_TIMER_CLK_WIDTH  )
  ) wdt_clk_gen (
  .CLK_IN   ( sysclk                    ) ,
  .EN       ( sys_rdy                   ) ,
  .CLK_OUT  ( wdt_clk                   )
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
localparam CMD_VERSION1         = CMD_RW_TYPE_BASE + 4;
localparam CMD_VERSION2         = CMD_RW_TYPE_BASE + 5;
localparam CMD_GATE_DRV_STATUS  = CMD_RW_TYPE_BASE + 6;
// The command strobes start after the read/write command types
localparam CMD_STROBE_START         = CMD_RW_TYPE_BASE + 'h10;
localparam CMD_TOGGLE_MOTOR_EN      = CMD_RW_TYPE_BASE + CMD_STROBE_START;
// Response & request buffer sizes
localparam SPI_SLAVE_RES_BUF_LEN = 12;
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
reg [ NUM_MOTORS - 1:0 ]                spi_master_sel_num = 0;
reg [ NUM_MOTORS - 1:0 ]                spi_master_sel = 0;
reg [1:0]                               spi_master_recv_index = 0;
wire                                    spi_master_sel_now;
wire                                    spi_master_busy,
                                        spi_master_valid;
wire [ SPI_MASTER_DATA_WIDTH - 1:0 ]    spi_master_d0,
                                        spi_master_di;

// shift register for detecting falling edge of spi_master_busy signal
reg [1:0] spi_master_busy_sr = 0;  always @(posedge sysclk) spi_master_busy_sr <= {spi_master_busy_sr[0], spi_master_busy};
wire spi_master_trxfr_done_flag   =   ( spi_master_busy_sr == 2'b10 );

// select an SPI slave device according to the spi_master_sel_num index of the signal array
always@(posedge sysclk) spi_master_sel[spi_master_sel_num] <= spi_master_sel_now;
assign drv_ncs_o = ~spi_master_sel;

SPI_Master spi_master_module (
    .clk            ( sysclk                ) ,
    .EN             ( sys_rdy               ) ,
    .SCK            ( spi_master_sck_o      ) ,
    .MOSI           ( spi_master_mosi_o     ) ,
    .MISO           ( spi_master_miso_s     ) ,
    .SEL            ( spi_master_sel_now    ) ,
    .START          ( spi_master_start      ) ,
    .BUSY           ( spi_master_busy       ) ,
    .VALID          ( spi_master_valid      ) ,
    .DATA_OUT       ( spi_master_d0         ) ,
    .DATA_IN        ( spi_master_di         )
);

// The DRV8303 config values we write to each driver
reg [SPI_MASTER_DATA_WIDTH-1:0] spi_master_data_array_out [2:0];
reg [11:0] spi_master_data_array_in  [NUM_MOTORS - 1:0];
reg spi_master_config_state;

// This is where we assign the data we want to send to the selected SPI slave
assign spi_master_di = spi_master_data_array_out[spi_master_recv_index];


// This sets the max output current for the gate pins
//   0 = 1.7A
//   1 = 0.7A
//   2 = 0.25A
localparam DRV8303_GATE_CURRENT = 0;

// This is a strobe bit that resets the gate outputs
//   0 = Normal mode
//   1 = Reset gate driver latched faults (reverts to 0)
localparam DRV8303_GATE_RESET = 1;

// This enables/disables the over-current protection
//   0 = current limit
//   1 = over-current protection latch mode
//   2 = report only
//   3 = over-current protection disabled
localparam DRV8303_OCP_MODE = 0;

// When this is set, only the 3 high side PWM signals should be sent!
localparam DRV8303_3_INPUTS = 0;

// This sets the over-current protection threshold
// DON'T SET THIS PAST 14! Ever!
localparam DRV8303_OC_ADJ_VAL = 13;

// This sets if over-current and/or over-temperature are reported.
//   0 = over-current & over-temperature
//   1 = over-temperature only
//   2 = over-current only
localparam DRV8303_OC_REPORT = 0;

// This sets the ADC gain for the sense resistors
// Note: over-current protection controlled by the DRV8303 itself does not take
// into account anything to do with the shunt resistors. These are only used for
// reading from an external ADC for control loop input.
//   0 = 10V/V
//   1 = 20V/V
//   2 = 40V/V
//   3 = 80V/V
localparam DRV8303_AMP_GAIN = 1;

// This sets how the over-current protection is handled once triggered
//   0 = cycle-by-cycle control
//   1 = timed control
localparam DRV8303_OC_TOFF = 1;

wire sys_begin_startup = ( start_delay_done == 1 ) && ( sys_rdy == 0 );
wire gate_drivers_set_config;

always @(posedge sysclk)
begin : SPI_MASTER_COMM
    // if spi master transfer complete
    if ( sys_begin_startup == 1 ) begin
        // flag the system as ready for all of the other areas of the Verilog
        // ** this will start the initial gate driver configurations **
        sys_rdy <= 1;
    end else if ( gate_drivers_set_config == 1 ) begin
        // start the first SPI master transfer out
        spi_master_start <= 1;
        // set the config values for each driver here
        // config register 0x02: over-current threshold selection & reset gate outputs
        spi_master_data_array_out[0] <=     (2                      << 11)  |
                                            (DRV8303_OC_ADJ_VAL     << 6 )  |
                                            (DRV8303_OCP_MODE       << 4 )  |
                                            (DRV8303_3_INPUTS       << 3 )  |
                                            (DRV8303_GATE_RESET     << 2 )  |
                                            (DRV8303_GATE_CURRENT   << 0 );
        // config register 0x03: over-current cycle off type & shunt amplifier gain (20V/V)
        spi_master_data_array_out[1] <=     (3                      << 11)  |
                                            (DRV8303_OC_TOFF        << 6 )  |
                                            (DRV8303_AMP_GAIN       << 2 )  |
                                            (DRV8303_OC_REPORT      << 0 );
        // the last index here is unused
        spi_master_data_array_out[2] <= 0;
        // enter config state for the gate drivers
        spi_master_config_state <= 1;
        spi_master_sel_num <= 0;

    end else if ( spi_master_trxfr_done_flag == 1 ) begin
        // take appropiate action if the SPI received data is flagged as being valid
        if ( spi_master_valid == 1 ) begin
            // start the next transfer out
            spi_master_start <= 1;

            if ( spi_master_recv_index == 2 ) begin
                // store only bit-7 from address 0x01 since it's the only one with useful information
                spi_master_data_array_in[spi_master_sel_num][11] <= spi_master_d0[7];
                // reset the rx buffer to the beginning
                spi_master_recv_index <= 0;

                if ( spi_master_sel_num >= (NUM_MOTORS - 1) ) begin
                    // reset the selected SPI slave to the first one
                    spi_master_sel_num <= 0;

                    if ( spi_master_config_state == 1 ) begin
                        // store the transactions for reading the status registers, then switch states
                        spi_master_data_array_out[0] <= (1 << 15) | (0 << 11);
                        spi_master_data_array_out[1] <= (1 << 15) | (1 << 11);
                        // disable & exit the config state
                        spi_master_config_state <= 0;
                    end
                end else begin
                    // select the next in line SPI device we will communicate with
                    spi_master_sel_num <= spi_master_sel_num + 1;
                end
            end else if ( spi_master_recv_index == 1 ) begin
                // store the 11 LSB from address 0x00 since that's where the core of what we want is located
                spi_master_data_array_in[spi_master_sel_num][10:0] <= spi_master_d0[10:0];
                spi_master_recv_index <= spi_master_recv_index + 1;
            end else begin
                // increment the rx buffer index on each received set of bytes
                // from the same device
                spi_master_recv_index <= spi_master_recv_index + 1;
            end
        end

    end else begin
        spi_master_start <= 0;
    end
end

wire command_byte_rdy = ( ( spi_slave_byte_count == 1 ) && ( spi_slave_byte_done_d == 1 ) );
wire [ SPI_SLAVE_DATA_WIDTH - 1:0 ]  spi_slave_di = spi_slave_res_buf[ spi_slave_byte_count ];
wire [ SPI_SLAVE_DATA_WIDTH - 2:0 ] command_byte = command_byte_rdy ? spi_slave_do[SPI_SLAVE_DATA_WIDTH - 2:0] : spi_slave_req_buf[0][SPI_SLAVE_DATA_WIDTH - 2:0];
wire                                command_rw   = command_byte_rdy ? spi_slave_do[SPI_SLAVE_DATA_WIDTH - 1]   : spi_slave_req_buf[0][SPI_SLAVE_DATA_WIDTH - 1];
reg [ SPI_SLAVE_DATA_WIDTH - 2:0 ]  command_byte_l = 0;
reg command_rw_l = 0;

// SPI Slave module
SPI_Slave spi_slave_module (
    .clk            ( sysclk                ) ,
    .SCK            ( spi_slave_sck         ) ,
    .MOSI           ( spi_slave_mosi_s      ) ,
    .MISO           ( spi_slave_miso_o      ) ,
    .SSEL           ( spi_slave_ncs_s       ) ,
    .DONE           ( spi_slave_byte_done   ) ,
    .DATA_IN        ( spi_slave_do          ) ,
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
assign gate_drivers_set_config = ( sys_begin_startup == 1 ) || ( motor_update_flag == 1 );

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
                    spi_slave_res_buf[2*NUM_ENCODERS+1] <= watchdog_timer[1][WATCHDOG_TIMER_WIDTH - 1 : (WATCHDOG_TIMER_WIDTH - SPI_SLAVE_DATA_WIDTH + 1)];
                    spi_slave_res_buf[2*NUM_ENCODERS+2] <= watchdog_timer[1][(WATCHDOG_TIMER_WIDTH - SPI_SLAVE_DATA_WIDTH - 1) : 0];
                    motor_update_flag <= 1;
                end

                CMD_ENCODER_COUNT :
                begin
                    // Encoder inputs are latched here so all readings are from the same time
                    for (j = 0; j < NUM_ENCODERS; j = j + 1)
                    begin : LATCH_ENC_COUNTS
                        spi_slave_res_buf[2*j+1]    <=  enc_count[j][ENCODER_COUNT_WIDTH-1:SPI_SLAVE_DATA_WIDTH];
                        spi_slave_res_buf[2*j+2]    <=  enc_count[j][SPI_SLAVE_DATA_WIDTH-1:0];
                    end
                    // The latched watchdog timer count
                    spi_slave_res_buf[2*NUM_ENCODERS+1] <= watchdog_timer[1][WATCHDOG_TIMER_WIDTH - 1: (WATCHDOG_TIMER_WIDTH - SPI_SLAVE_DATA_WIDTH + 1)];
                    spi_slave_res_buf[2*NUM_ENCODERS+2] <= watchdog_timer[1][(WATCHDOG_TIMER_WIDTH - SPI_SLAVE_DATA_WIDTH - 1) : 0];
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
                CMD_VERSION1 :
                begin
                    for (j = 0; j < 10; j = j + 1)
                    begin : LATCH_GIT_HASH1
                        spi_slave_res_buf[j+1]    <=  (`GIT_VERSION_HASH >> (8 * j)) & 'hFF;
                    end
                    spi_slave_res_buf[11] <= `GIT_VERSION_DIRTY;
                end

                CMD_VERSION2 :
                begin
                    for (j = 0; j < 10; j = j + 1)
                    begin : LATCH_GIT_HASH2
                        spi_slave_res_buf[j+1]    <=  (`GIT_VERSION_HASH >> (8 * (10 + j))) & 'hFF;
                    end
                    spi_slave_res_buf[11] <= `GIT_VERSION_DIRTY;
                end

`endif
                CMD_GATE_DRV_STATUS :
                begin
                    for (j = 0; j < NUM_MOTORS; j = j + 1)
                    begin : LATCH_GATE_DRV_STATUS
                        spi_slave_res_buf[2*j+1]    <=  spi_master_data_array_in[j][7:0];
                        spi_slave_res_buf[2*j+2]    <=  {4'h0, spi_master_data_array_in[j][11:8]};
                    end
                end

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

        for (j = 0; j < NUM_MOTORS; j = j + 1)
        begin : INIT_DUTY_CYCLES
            duty_cycle[j] <= 0;
        end

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
                    if ( spi_slave_byte_count == (2 * NUM_MOTORS) ) begin
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
        end else if ( wdt_clk_rising_edge == 1 ) begin
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
    // for (i = 0; i < NUM_ENCODERS; i = i + 1)
    for (i = 0; i < 3; i = i + 1)
    begin : BLDC_MOTOR_INST
        BLDC_Motor #(
            .MAX_DUTY_CYCLE         ( (1 << DUTY_CYCLE_WIDTH) - 1   ) ,
            .ENCODER_COUNT_WIDTH    ( ENCODER_COUNT_WIDTH           ) ,
            .HALL_COUNT_WIDTH       ( HALL_COUNT_WIDTH              )
            ) motor (
            .clk                    ( sysclk                        ) ,
            .en                     ( motors_en && sys_rdy          ) ,
            .reset_enc_count        ( motor_update_flag             ) ,
            .reset_hall_count       ( ~hall_conns[i]                ) ,
            .duty_cycle             ( duty_cycle[i]                 ) ,
            .enc                    ( enc_s[i]                      ) ,
            .hall                   ( hall_s[i]                     ) ,
            .phaseH                 ( phaseH_o[i]                   ) ,
            .phaseL                 ( phaseL_o[i]                   ) ,
            .enc_count              ( enc_count[i]                  ) ,
            .hall_count             ( hall_count[i]                 ) ,
            .connected              ( hall_conns[i]                 )
        );
    end
endgenerate


`ifdef DRIBBLER_MOTOR_EN
BLDC_Motor_No_Encoder #(
    .MAX_DUTY_CYCLE         ( (1 << DUTY_CYCLE_WIDTH) - 1           ) ,
    .HALL_COUNT_WIDTH       ( HALL_COUNT_WIDTH                      )
    ) dribbler_motor (
    .clk                    ( sysclk                                ) ,
    .en                     ( motors_en && sys_rdy                  ) ,
    .reset_hall_count       ( ~hall_conns[NUM_MOTORS-1]             ) ,
    .duty_cycle             ( duty_cycle[NUM_MOTORS-1]              ) ,
    .hall                   ( hall_s[NUM_MOTORS-1]                  ) ,
    .phaseH                 ( phaseH_o[NUM_MOTORS-1]                ) ,
    .phaseL                 ( phaseL_o[NUM_MOTORS-1]                ) ,
    .hall_count             ( hall_count[NUM_MOTORS-1]              ) ,
    .connected              ( hall_conns[NUM_MOTORS-1]              )
);
`endif

endmodule   // RoboCup
