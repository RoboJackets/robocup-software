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
*  parameter, `END_STARTUP_DUTY_CYCLE`, and stay in `STATE_STARTUP` until reaching
*  the desired duty cycle. When not defined, the state machine will exit `STATE_STARTUP`
*  when the duty cycle is equal to `END_STARTUP_DUTY_CYCLE`.
*/
`define STARTUP_INCREMENT_COMPLETELY


// BLDC_Driver module
module BLDC_Driver ( clk, en, hall, duty_cycle, phaseH, phaseL, fault );

// Module parameters - passed parameters will overwrite the values here
parameter PHASE_DRIVER_MAX_COUNTER =            ( 'h3FF );
parameter MAX_DUTY_CYCLE =                      ( 'h3FF );
parameter MIN_DUTY_CYCLE =                      ( 0 );
parameter DUTY_CYCLE_STEP_RES =                 ( 1 );
parameter DEAD_TIME =                           ( 2 );

// Local parameters - can not be altered outside this module
`include "log2-macro.v"     // This must be included here
localparam DUTY_CYCLE_WIDTH =   `LOG2( MAX_DUTY_CYCLE );

// Module inputs/outputs
input clk, en;
input [2:0] hall;
input [DUTY_CYCLE_WIDTH-1:0] duty_cycle;
output reg [2:0] phaseH, phaseL;
output reg fault = 0;
// ===============================================


// Local parameters that can not be altered outside of this file
// ===============================================
localparam NUM_PHASES =                 3;  // This will always be constant
localparam STARTUP_COUNTER_WIDTH =      5; // Counter for startup time period. Time expires when register overflows to 0 and is incremented according to HALL_CHECK_COUNTER_WIDTH
localparam HALL_STATE_STEADY_COUNT =    31; // Threshold value in determining when the hall effect sensor is locked into an error state

// Derived local parameters
// ===============================================
localparam END_STARTUP_DUTY_CYCLE =         ( MAX_DUTY_CYCLE >> 2 );                        // Divide by 4 to get 25% of the max speed for startup state
localparam STARTUP_DUTY_CYCLE_STEPS =       ( END_STARTUP_DUTY_CYCLE - MIN_DUTY_CYCLE );    // Get the number of steps between the min. and max. duty cycles for startup state
localparam STARTUP_PERIOD_CLOCK_CYCLES =    ( 1 << STARTUP_COUNTER_WIDTH );                 // The number of input clock cycles in one period of the startup counter's clock
localparam HALL_CHECK_CLOCK_WIDTH =         ( STARTUP_COUNTER_WIDTH );                      // Width of counter for checking the hall effect sensor inputs at a reduced frequency.
localparam HALL_CHECK_COUNTER_WIDTH =       `LOG2( HALL_STATE_STEADY_COUNT );             // Counter used for reduced sampling of the hall effect sensor
localparam PHASE_DRIVER_COUNTER_WIDTH =     `LOG2( PHASE_DRIVER_MAX_COUNTER );

// State machine declarations for readability
// ===============================================
localparam STATE_STOP =         0;
localparam STATE_STARTUP =      1;
localparam STATE_RUN =          2;
localparam STATE_POLL_HALL =    3;
localparam STATE_ERR =          4;


// Register and Wire declarations
// ===============================================

// Synced input/output registers
reg en_s, fault_s = 0;
reg [2:0] hall_s = 0;
reg [DUTY_CYCLE_WIDTH-1:0] duty_cycle_s = 0;
wire [2:0] phaseH_s, phaseL_s;

// State machine state register & hall sensor connection checking
reg [2:0] state = 0;

// Connections between the `Phase_Driver` and `Hall_Effect_Sensor` modules
wire [2:0] z, u;

// Counter registers for hall check and startup timers
reg [STARTUP_COUNTER_WIDTH-1:0] startup_counter = 0;

// Count values for consecuitive hall effect sensor states
reg [HALL_CHECK_COUNTER_WIDTH-1:0]  hall_hardware_err_cnt = 0,
                                    hall_disconnected_cnt = 0,
                                    hall_reconnect_cnt = 0,
                                    hall_check_time = 0;

// Error flags
reg hardware_fault_latched = 0,
    disconnect_fault_latched = 0;

// Variable used for instantiation of the number of phases
genvar j;

// Show the expected startup length during synthesis. Assumes an 18.432MHz input clock.
initial begin
`ifdef STARTUP_INCREMENT_COMPLETELY
    $display ("Motor startup state will hold until at requested duty cycle. [STARTUP_INCREMENT_COMPLETELY = YES]");
`endif
    $display ("The startup duty cycle increments every %d clock cycles.", STARTUP_PERIOD_CLOCK_CYCLES);
    $display ("    Duty cycle range during motor startup: %d to %d.", MIN_DUTY_CYCLE, END_STARTUP_DUTY_CYCLE);
    $display ("    The full range of the duty cycle is from %d to %d.\n", MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
end


// Begin main logic
always @(posedge clk)
begin : MOTOR_STATES

    if ( en == 0 ) begin
        fault <= 0;
        fault_s <= 1'b0;
        en_s <= 0;
        duty_cycle_s <= 0;
        state <= STATE_STOP;
    end else begin
        // Sync. all the inputs and outputs to the clock who's source may not be internal logic
        hall_s <= hall;
        phaseH <= phaseH_s;
        phaseL <= phaseL_s;

        // Increment everytime
        startup_counter <= startup_counter + 1;
        hall_check_time <= hall_check_time + 1;
        
        if ( fault_s == 1 ) begin
            fault_s <= 1'b0;
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
                    if ( duty_cycle > MIN_DUTY_CYCLE ) begin
                        duty_cycle_s <= MIN_DUTY_CYCLE; // initalize the starting ramp duty cycle.
                        state <= STATE_STARTUP;
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

                        if ( duty_cycle_s > duty_cycle ) begin
                            // If this were to occur, the input duty cycle was changed to a lower value than what it was when entering STATE_STARTUP.
                            // Enter the running state since the MIN_DUTY_CYCLE parameter and [slightly] higher level motion control is assumed to take care of the lowest possible motor speed.
                            duty_cycle_s <= duty_cycle;
                            state <= STATE_RUN;

                        end else begin
                            // Transition to the run state once target duty cycle reached the max startup duty cycle.
                            `ifdef STARTUP_INCREMENT_COMPLETELY
                            if ( ( duty_cycle_s < END_STARTUP_DUTY_CYCLE ) | ( duty_cycle_s < duty_cycle ) ) begin
                            `else
                            if ( ( duty_cycle_s < END_STARTUP_DUTY_CYCLE ) & ( duty_cycle_s < duty_cycle ) ) begin
                            `endif

                                if ( startup_counter == 0 ) begin
                                    startup_counter <= 1;
                                    duty_cycle_s <= duty_cycle_s + 1;
                                end

                            end else begin
                                duty_cycle_s <= duty_cycle;
                                state <= STATE_RUN;
                            end
                        end

                    end
                end    // STATE_STARTUP

                STATE_RUN: begin

                    if ( duty_cycle <= MIN_DUTY_CYCLE ) begin
                        // Stop the motor if below the minimum required duty cycle
                        en_s <= 0;
                        state <= STATE_STOP;
                    end else begin
                        // Stay in the running state as long as the given duty cycle can be used to spin the motor
                        en_s <= 1;
                        duty_cycle_s <= duty_cycle;
                    end
                end    // STATE_RUN

                STATE_POLL_HALL: begin
                    // Wait in this state until a hall sensor is connected and is giving a valid input
                    state <= STATE_POLL_HALL;
                end    // STATE_POLL_HOLL

                STATE_ERR: begin
                    // Die if a motor error occurs at this level. Set the `fault` output HIGH.
                    // This is reset by toggling the `en` input for one clock cycle.
                    fault <= 1;
                    state <= STATE_ERR;
                end    // STATE_ERR

                default: begin
                    en_s <= 0;
                    duty_cycle_s <= duty_cycle;
                    state <= STATE_STOP;
                end    // default

            endcase
        end


        // Check for a hall sensor that is physically at fault somewhere along the line or unconnected by storing previous samples in a buffer array.
        if ( ( hardware_fault_latched != 1 ) & ( hall_check_time == 0 ) ) begin
            if ( hall_s == 3'b000 ) begin
                hall_hardware_err_cnt <= hall_hardware_err_cnt + 1;
            end else if ( hall_s == 3'b111 ) begin
                hall_disconnected_cnt <= hall_disconnected_cnt + 1;
            end else if ( state == STATE_POLL_HALL ) begin
                hall_reconnect_cnt <= hall_reconnect_cnt + 1;
            end else begin
                hall_hardware_err_cnt <= 0;
                hall_disconnected_cnt <= 0;
                hall_reconnect_cnt <= 0;
            end
        end


        // Check if the buffer holds the same error value at every index. Set the motor to an error state if so, or reset the values otherwise.
        if ( ( hall_hardware_err_cnt == HALL_STATE_STEADY_COUNT ) | ( hall_disconnected_cnt == HALL_STATE_STEADY_COUNT ) | ( hall_reconnect_cnt == HALL_STATE_STEADY_COUNT ) ) begin
            if ( hall_hardware_err_cnt == HALL_STATE_STEADY_COUNT ) begin
                // There's probably some really bad things going on. The inputs have pull-up resistors on all 3 lines, and this would be a state of 3b'000.
                hardware_fault_latched <= 1;
                fault_s <= 1'b1;
            end else if ( hall_disconnected_cnt == HALL_STATE_STEADY_COUNT ) begin
                // Most likely disconnected where the state would be 3'b111
                disconnect_fault_latched <= 1;
                fault_s <= 1'b1;
            end else if ( ( hall_reconnect_cnt == HALL_STATE_STEADY_COUNT ) & ( hardware_fault_latched != 1 ) ) begin
                // A hall sensor was connected again after being turned on initially.
                disconnect_fault_latched <= 0;
                state <= STATE_STOP;
            end

            hall_hardware_err_cnt <= 0;
            hall_disconnected_cnt <= 0;
            hall_reconnect_cnt <= 0;
         end

    end
end  // MOTOR_STATES


// The Hall_Effect_Sensor module does not use synced inputs.
Hall_Effect_Sensor hallEffectSensor ( .hall( hall_s ), .u( u ), .z( z ) );


// The Phase_Driver module does not use synced inputs. Synchronization is taken care of within this module because it must be used along with the Hall_Effect_Sensor module.
generate
for (j = 0; j < NUM_PHASES; j = j + 1)
begin : GEN_PHASE_DRIVER
    Phase_Driver #(
        .DEAD_TIME ( DEAD_TIME ) ,
        .COUNTER_WIDTH ( PHASE_DRIVER_COUNTER_WIDTH ) ,
        .MAX_COUNTER ( PHASE_DRIVER_MAX_COUNTER ) ,
        .DUTY_CYCLE_WIDTH ( DUTY_CYCLE_WIDTH ) ,
        .MAX_DUTY_CYCLE ( MAX_DUTY_CYCLE ) ,
        .DUTY_CYCLE_STEP_RES ( DUTY_CYCLE_STEP_RES )
        ) motorPhaseDriver (
        .clk ( clk ) ,
        .duty_cycle ( ( u[j] == 1 ) ? duty_cycle_s : 1'b0 ) ,
        .high_z ( ( z[j] || ~en_s ) ? 1'b1 : 1'b0 ) ,
        .pwm_high ( phaseH_s[j] ) ,
        .pwm_low ( phaseL_s[j] )
    );
end
endgenerate

endmodule

`endif
