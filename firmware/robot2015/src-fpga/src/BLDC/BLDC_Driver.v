/*
*  BLDC_Driver.v
*
*  A BLDC controller that includes hall sensor error detection and incremental
*  duty cycles during startup for reducing motor startup current.
*
*/

`ifndef _BLDC_DRIVER_
`define _BLDC_DRIVER_

`include "Hall_Effect_Sensor.v"
`include "Phase_Driver.v"

/*
*  If `STARTUP_INCREMENT_COMPLETELY` is defined, the state machine will ignore the
*  parameter, `STARTUP_END_DUTY_CYCLE`, and stay in `STATE_STARTUP` until reaching
*  the desired duty cycle. When not defined, the state machine will exit `STATE_STARTUP`
*  when the duty cycle is equal to `STARTUP_END_DUTY_CYCLE`.
*/
// `define STARTUP_INCREMENT_COMPLETELY

// Use a fixed time startup period vs a duty cycle thereshold for exiting the startup state
`define STARTUP_FIXED_TIME_RAMPING


// BLDC_Driver module
module BLDC_Driver ( clk, en, hall, duty_cycle, direction, phaseH, phaseL, connected, fault );

// Module parameters - passed parameters will overwrite the values here
parameter PHASE_DRIVER_MAX_COUNTER =            ( 'h3FF );
parameter MAX_DUTY_CYCLE =                      ( 'h3FF );
parameter DUTY_CYCLE_STEP_RES =                 ( 1 );
parameter DEAD_TIME =                           ( 8 );

// Local parameters - can not be altered outside this module
`include "log2-macro.v"     // This must be included here
localparam DUTY_CYCLE_WIDTH =   `LOG2( MAX_DUTY_CYCLE );

// Module inputs/outputs
input clk, en;
input [2:0] hall;
input [DUTY_CYCLE_WIDTH-1:0] duty_cycle;
input direction;
output reg [2:0] phaseH, phaseL;
output reg connected = 0;
output reg fault = 0;
// ===============================================


// Local parameters that can not be altered outside of this file
// ===============================================
localparam NUM_PHASES =                  3;  // This will always be constant
localparam HALL_STATE_STEADY_COUNT =    31;  // Threshold value in determining when the hall effect sensor is locked into an error state

localparam STARTUP_COUNTER_WIDTH =      12;  // Counter for ticking the startup pwm duty_cycle changes. Time expires when register overflows to 0
localparam STARTUP_STEP_COUNTER_WIDTH =  7;  // The counter that tracks the number of startup cycle periods. ie. how many times the duty cycle has been updated
// The startup time is equal to (1/18.432) * 2^(STARTUP_COUNTER_WIDTH + STARTUP_STEP_COUNTER_WIDTH)
// For now, we're shooting for somewhere in the range of 25ms to 30ms.

// Derived local parameters
// ===============================================
localparam STARTUP_END_DUTY_CYCLE =         ( MAX_DUTY_CYCLE >> 2 );                        // Divide by 4 to get 25% of the max speed for startup state
localparam STARTUP_END_STEP_COUNT =         ( (1 << STARTUP_STEP_COUNTER_WIDTH) - 1 );      // Startup state exiting computed using this value and the MIN_DUTY_CYCLE for fixed time ramping
localparam STARTUP_PERIOD_CLOCK_CYCLES =    ( 1 << (DUTY_CYCLE_WIDTH + 3) );                // The number of input clock cycles in one period of the startup counter's clock - this is best
                                                                                            // set to a value that evenly divides into the PWM period from 'Phase_Driver.v'
localparam HALL_CHECK_COUNTER_WIDTH =       `LOG2( HALL_STATE_STEADY_COUNT );               // Counter used for reduced sampling of the hall effect sensor
localparam PHASE_DRIVER_COUNTER_WIDTH =     `LOG2( PHASE_DRIVER_MAX_COUNTER );
localparam MIN_DUTY_CYCLE =                 ( MAX_DUTY_CYCLE * 2 / 100 );                   // 5% of the max


// State machine declarations for readability
// ===============================================
localparam STATE_STOP           = 0;
localparam STATE_STARTUP        = 1;
localparam STATE_RUN            = 2;
localparam STATE_POLL_HALL      = 3;
localparam STATE_ERR            = 4;


// Register and Wire declarations
// ===============================================

// Synced input/output registers
reg                         en_s                                = 0,
                            fault_s                             = 0;
reg  [2:0]                  hall_s                              = 0;
reg  [DUTY_CYCLE_WIDTH-1:0] duty_cycle_s                        = 0;
wire [2:0]                  phaseH_s,
                            phaseL_s;

// State machine state register & hall sensor connection checking
reg  [2:0]                  state                               = STATE_STOP;

// Connections between the `Phase_Driver` and `Hall_Effect_Sensor` modules
wire [2:0] z, u;

// Counter registers for hall check and startup timers
reg  [STARTUP_COUNTER_WIDTH-1:0]      startup_counter           = 0;
reg  [STARTUP_STEP_COUNTER_WIDTH-1:0] startup_step_count        = 0;
reg  [DUTY_CYCLE_WIDTH-1:0]           startup_duty_cycle_step   = 0;

// Count values for consecuitive hall effect sensor states
reg  [HALL_CHECK_COUNTER_WIDTH-1:0]   hall_hardware_err_cnt     = 0,
                                      hall_disconnected_cnt     = 0,
                                      hall_reconnect_cnt        = 0,
                                      hall_check_time           = 0;

// Error flags
reg hardware_fault_latched = 0,
    disconnect_fault_latched = 0;

wire hall_recheck = ( hall_hardware_err_cnt == HALL_STATE_STEADY_COUNT ) ||
                    ( hall_disconnected_cnt == HALL_STATE_STEADY_COUNT ) ||
                    ( hall_reconnect_cnt    == HALL_STATE_STEADY_COUNT );

reg startup_wait_delay = 0;

// Variable used for instantiation of the number of phases
genvar j;

// Show the expected startup length during synthesis. Assumes an 18.432MHz input clock.
initial begin
// Make sure only one of these are defined - default to STARTUP_FIXED_TIME_RAMPING
// being enabled if both are defined
`ifdef STARTUP_FIXED_TIME_RAMPING
`ifdef STARTUP_INCREMENT_COMPLETELY
`undef STARTUP_INCREMENT_COMPLETELY
`endif
`endif
`ifdef STARTUP_INCREMENT_COMPLETELY
`ifdef STARTUP_FIXED_TIME_RAMPING
`undef STARTUP_INCREMENT_COMPLETELY
`endif
`endif

`ifdef STARTUP_INCREMENT_COMPLETELY
    $display ("Motor startup state will hold until at requested duty cycle [STARTUP_INCREMENT_COMPLETELY = YES]");
    $display ("The startup duty cycle increments every %d clock cycles", STARTUP_PERIOD_CLOCK_CYCLES);
    $display ("  --  duty cycle ranges from %d to %d during startup", MIN_DUTY_CYCLE, STARTUP_END_DUTY_CYCLE);
`endif

`ifdef STARTUP_FIXED_TIME_RAMPING
    $display ("Motor startup state based on a fixed time duration [STARTUP_FIXED_TIME_RAMPING = YES]");
    $display ("The startup state exits after %d periods of startup duty cycle updates", STARTUP_END_STEP_COUNT);
    $display ("  --  there are %d clock edges in every startup duty cycle period", STARTUP_PERIOD_CLOCK_CYCLES);
`endif
    $display ("  --  duty cycle ranges from %d to %d over the fully range of possible values\n", MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
end

wire commutation_switch = hall_s != hall;
reg [2:0] commutation_startup_count = 0;
always @(posedge clk) begin
    if ( commutation_switch == 1 ) begin
        if ( commutation_startup_count < 5 ) begin
            commutation_startup_count <= commutation_startup_count + 1;
        end
    end
end

// Begin main logic
always @(posedge clk)
begin : MOTOR_STATES

    connected <= ~disconnect_fault_latched;

    if ( en == 0 ) begin
        fault <= 0;
        fault_s <= 0;
        en_s <= 0;
        duty_cycle_s <= 0;
        state <= STATE_STOP;

    end else begin
        // Sync. all the inputs and outputs to the clock who's source may not be internal logic
        hall_s <= hall;
        phaseH <= phaseH_s;
        phaseL <= phaseL_s;

        // Increment everytime
        hall_check_time <= hall_check_time + 1;

        if ( fault_s == 1 ) begin
            fault_s <= 0;
            en_s <= 0;
            duty_cycle_s <= 0;

            if ( disconnect_fault_latched == 1 ) begin
                state <= STATE_POLL_HALL;
            end else begin
                state <= STATE_ERR;
            end
        end else begin
            case (state)

                STATE_STOP: begin

                    en_s <= 0;
                    startup_step_count <= 0;

                    if ( duty_cycle > MIN_DUTY_CYCLE ) begin
                        duty_cycle_s <= MIN_DUTY_CYCLE; // initalize the starting ramp duty cycle.
                        state <= STATE_STARTUP;

                        // set the amount that we increment the duty cycle on every startup update period
                        `ifdef STARTUP_FIXED_TIME_RAMPING
                        // startup_duty_cycle_step <= ((duty_cycle - MIN_DUTY_CYCLE) >> (`LOG2(STARTUP_END_STEP_COUNT + 1) + 1));
                        startup_duty_cycle_step <= 1;
                        `else
                        startup_duty_cycle_step <= 1;
                        `endif

                    end else begin
                        duty_cycle_s <= 0;
                        state <= STATE_STOP;
                    end
                end    // STATE_STOP

                STATE_STARTUP: begin

                    if ( duty_cycle <= MIN_DUTY_CYCLE ) begin
                        // Exit the startup state if the duty cycle is ever below the min starting duty cycle.
                        en_s <= 0;
                        state <= STATE_STOP;
                    end else begin
                        en_s <= 1;

                        // make sure that we didn't push out all the bits in this number when we divided
                        if ( startup_duty_cycle_step == 0 ) begin
                            startup_duty_cycle_step <= 1;
                        end

                        `ifdef STARTUP_FIXED_TIME_RAMPING
                        // Exit the startup phase based on a fixed time startup period
                        if ( startup_step_count < STARTUP_END_STEP_COUNT ) begin
                        `else
                        // Transition to the run state once target duty cycle reached the max startup duty cycle.
                        `ifdef STARTUP_INCREMENT_COMPLETELY
                        if ( ( duty_cycle_s < STARTUP_END_DUTY_CYCLE ) | ( duty_cycle_s < duty_cycle ) ) begin
                        `else
                        if ( ( duty_cycle_s < STARTUP_END_DUTY_CYCLE ) & ( duty_cycle_s < duty_cycle ) ) begin
                        `endif
                        `endif

                            if ( startup_counter == 0 ) begin
                                startup_counter <= 1;
                                startup_step_count <= startup_step_count + 1;
                                duty_cycle_s <= duty_cycle_s + startup_duty_cycle_step;
                            end else begin
                                startup_counter <= startup_counter + 1;
                            end

                        end else begin
                            duty_cycle_s <= duty_cycle;
                            state <= STATE_RUN;
                        end
                    end
                end    // STATE_STARTUP

                STATE_RUN: begin
                    // Stay in the running state as long as the given duty cycle can be used to spin the motor
                    duty_cycle_s <= duty_cycle;

                    if ( duty_cycle <= MIN_DUTY_CYCLE ) begin
                        // Stop the motor if below the minimum required duty cycle
                        en_s <= 0;
                        state <= STATE_STOP;
                    end
                end    // STATE_RUN

                STATE_POLL_HALL: begin
                    // Wait in this state until a hall sensor is connected and is giving a valid input
                    if ( disconnect_fault_latched == 0 ) begin
                        state <= STATE_STOP;
                    end
                end    // STATE_POLL_HALL

                STATE_ERR: begin
                    // Die if a motor error occurs at this level. Set the `fault` output HIGH.
                    // This is reset by toggling the `en` input for one clock cycle - or by
                    // disconnecting and reconnecting the motor.

                    // reset once the motor is disconnected
                    if ( hall_s == 3'b111 ) begin
                        disconnect_fault_latched <= 1;
                        hardware_fault_latched <= 0;
                        fault <= 0;
                        state <= STATE_POLL_HALL;
                    end else begin
                        // otherwise, stay in this state and assert the fault signal
                        fault <= 1;
                        state <= STATE_ERR;
                    end
                end    // STATE_ERR

                default: begin
                    en_s <= 0;
                    duty_cycle_s <= duty_cycle;
                    state <= STATE_STOP;
                end    // default

            endcase
        end


        // Check for a hall sensor that is physically at fault somewhere along the line or unconnected by storing previous samples in a buffer array.
        if ( (hardware_fault_latched != 1) & (hall_check_time == 0) ) begin
            if ( hall_s == 3'b000 ) begin
                hall_hardware_err_cnt <= hall_hardware_err_cnt + 1;
            end else if ( hall_s == 3'b111 ) begin
                hall_disconnected_cnt <= hall_disconnected_cnt + 1;
            end else begin
                hall_disconnected_cnt <= 0;
                hall_hardware_err_cnt <= 0;
            end

            if ( state == STATE_POLL_HALL ) begin
                hall_reconnect_cnt <= hall_reconnect_cnt + 1;
            end else begin
                hall_reconnect_cnt <= 0;
            end
        end


        // Check if the buffer holds the same error value at every index. Set the motor to an error state if so, or reset the values otherwise.
        if ( hall_recheck == 1 ) begin
            if ( hall_hardware_err_cnt == HALL_STATE_STEADY_COUNT ) begin
                // There's probably some really bad things going on. The inputs have pull-up resistors on all 3 lines, and this would be a state of 3b'000.
                hardware_fault_latched <= 1;
                fault_s <= 1;
            end else if ( hall_disconnected_cnt == HALL_STATE_STEADY_COUNT ) begin
                // Most likely disconnected - the state would be 3'b111
                disconnect_fault_latched <= 1;
                fault_s <= 1;
            end else if ( (hall_reconnect_cnt == HALL_STATE_STEADY_COUNT) && (hardware_fault_latched != 1) ) begin
                // A hall sensor was connected again after being powered up
                disconnect_fault_latched <= 0;
            end

            hall_hardware_err_cnt <= 0;
            hall_disconnected_cnt <= 0;
            hall_reconnect_cnt <= 0;
         end

    end
end  // MOTOR_STATES


// The Hall_Effect_Sensor module does not use synced inputs - no need to as long as we sync things at the top module.
Hall_Effect_Sensor hallEffectSensor ( .hall( hall_s ), .direction( direction ), .u( u ), .z( z ) );


// The Phase_Driver module does not use synced inputs. Synchronization is taken care of within this module because it must be used along with the Hall_Effect_Sensor module.
generate
for (j = 0; j < NUM_PHASES; j = j + 1)
begin : GEN_PHASE_DRIVER
    Phase_Driver #(
        .DEAD_TIME              ( DEAD_TIME                         ) ,
        .COUNTER_WIDTH          ( PHASE_DRIVER_COUNTER_WIDTH        ) ,
        .MAX_COUNTER            ( PHASE_DRIVER_MAX_COUNTER          ) ,
        .DUTY_CYCLE_WIDTH       ( DUTY_CYCLE_WIDTH                  ) ,
        .MAX_DUTY_CYCLE         ( MAX_DUTY_CYCLE                    ) ,
        .DUTY_CYCLE_STEP_RES    ( DUTY_CYCLE_STEP_RES               )
        ) motorPhaseDriver (
        .clk                    ( clk                               ) ,
        .duty_cycle             ( (u[j] == 1) ? duty_cycle_s : 0    ) ,
        .high_z                 ( (z[j] || ~en_s ) ? 1 : 0          ) ,
        .pwm_high               ( phaseH_s[j]                       ) ,
        .pwm_low                ( phaseL_s[j]                       )
    );
end
endgenerate

endmodule

`endif
