/*
*  BLDC_Motor.v
*
*  A BLDC controller that includes hall sensor error detection and incremental
*  duty cycles during startup for reducing motor startup current.
*
*/

`ifndef _BLDC_MOTOR_
`define _BLDC_MOTOR_

`define DRIBBLER_MOTOR_EN
`undef DRIBBLER_MOTOR_EN

`include "BLDC_Driver.v"
`include "BLDC_Hall_Counter.v"
`include "BLDC_Encoder_Counter.v"


// BLDC_Motor module
module BLDC_Motor ( clk, en, reset_enc_count, reset_hall_count, duty_cycle, enc, hall, phaseH, phaseL, enc_count, hall_count, connected );

// Module parameters - passed parameters will overwrite the values here
parameter MAX_DUTY_CYCLE =          ( 'h1FF             );
parameter MAX_DUTY_CYCLE_COUNTER =  ( MAX_DUTY_CYCLE    );
parameter ENCODER_COUNT_WIDTH =     ( 15                );
parameter HALL_COUNT_WIDTH =        ( 7                 );

// Local parameters - can not be altered outside this module
`include "log2-macro.v"     // This must be included here
localparam DUTY_CYCLE_WIDTH =   `LOG2( MAX_DUTY_CYCLE );

// Module inputs/outputs
input clk, en, reset_enc_count, reset_hall_count;
input [DUTY_CYCLE_WIDTH:0] duty_cycle;  // 1 more than requested
input [1:0] enc;
input [2:0] hall;
output [2:0] phaseH, phaseL;
output [ENCODER_COUNT_WIDTH-1:0] enc_count;
output [HALL_COUNT_WIDTH-1:0] hall_count;
output connected;
// ===============================================

wire hall_connected, hall_fault;

// Show the expected startup length during synthesis. Assumes an 18.432MHz input clock.
initial begin
    $display ("Duty cycle width from BLDC_Motor.v:\t%d", DUTY_CYCLE_WIDTH);
end

// Instantiation of all the modules required for complete functioning with all sensors
// ===============================================
BLDC_Encoder_Counter #(         // Instantiation of the encoder for counting the ticks
    .COUNT_WIDTH                ( ENCODER_COUNT_WIDTH       )
    ) encoder_counter (
    .clk                        ( clk                       ) ,
    .reset                      ( reset_enc_count           ) ,
    .enc                        ( enc                       ) ,
    .count                      ( enc_count                 )
);

BLDC_Hall_Counter #(            // Instantiation of the hall effect sensor's counter
    .COUNTER_WIDTH              ( HALL_COUNT_WIDTH          )
    ) hall_counter (
    .clk                        ( clk                       ) ,
    .reset                      ( reset_hall_count          ) ,
    .hall                       ( hall                      ) ,
    .count                      ( hall_count                )
);

BLDC_Driver #(                  // Instantiation of the motor driving module
    .PHASE_DRIVER_MAX_COUNTER   ( MAX_DUTY_CYCLE_COUNTER <<1) ,
    .MAX_DUTY_CYCLE             ( MAX_DUTY_CYCLE << 1       ) ,
    .DUTY_CYCLE_STEP_RES        ( 1                         ) ,
    .DEAD_TIME                  ( 8                         )
    ) bldc_motor (
    .clk                        ( clk                       ) ,
    .en                         ( en                        ) ,
    .hall                       ( hall                      ) ,
    .duty_cycle                 ( duty_cycle << 1           ) ,
    .phaseH                     ( phaseH                    ) ,
    .phaseL                     ( phaseL                    ) ,
    .connected                  ( hall_connected            ) ,
    .fault                      ( hall_fault                )
);

assign connected = hall_connected & ~(hall_fault);

endmodule


`ifdef DRIBBLER_MOTOR_EN1

// BLDC_Motor module - no encoder
module BLDC_Motor_No_Encoder ( clk, en, reset_hall_count, duty_cycle, hall, phaseH, phaseL, hall_count, connected );

// Module parameters - passed parameters will overwrite the values here
parameter MAX_DUTY_CYCLE =          ( 'h1FF             );
parameter MAX_DUTY_CYCLE_COUNTER =  ( MAX_DUTY_CYCLE    );
parameter HALL_COUNT_WIDTH =        ( 7                 );

// Local parameters - can not be altered outside this module
`include "log2-macro.v"     // This must be included here
localparam DUTY_CYCLE_WIDTH =   `LOG2( MAX_DUTY_CYCLE );

// Module inputs/outputsrecirculation
input clk, en, reset_hall_count;
input [DUTY_CYCLE_WIDTH:0] duty_cycle;  // 1 more than requested
input [2:0] hall;
output [2:0] phaseH, phaseL;
output [HALL_COUNT_WIDTH-1:0] hall_count;
output connected;
// ===============================================

wire hall_connected, hall_fault;

// Show the expected startup length during synthesis. Assumes an 18.432MHz input clock.
initial begin
    $display ("Duty cycle width from BLDC_Motor.v:\t%d", DUTY_CYCLE_WIDTH);
end

BLDC_Hall_Counter #(            // Instantiation of the hall effect sensor's counter
    .COUNTER_WIDTH              ( HALL_COUNT_WIDTH          )
    ) hall_counter (
    .clk                        ( clk                       ) ,
    .reset                      ( reset_hall_count          ) ,
    .hall                       ( hall                      ) ,
    .count                      ( hall_count                )
);

BLDC_Driver #(                  // Instantiation of the motor driving module
    .PHASE_DRIVER_MAX_COUNTER   ( MAX_DUTY_CYCLE_COUNTER <<1) ,
    .MAX_DUTY_CYCLE             ( MAX_DUTY_CYCLE << 1       ) ,
    .DUTY_CYCLE_STEP_RES        ( 1                         ) ,
    .DEAD_TIME                  ( 8                         )
    ) bldc_motor (
    .clk                        ( clk                       ) ,
    .en                         ( en                        ) ,
    .hall                       ( hall                      ) ,
    .duty_cycle                 ( duty_cycle << 1           ) ,
    .phaseH                     ( phaseH                    ) ,
    .phaseL                     ( phaseL                    ) ,
    .connected                  ( hall_connected            ) ,
    .fault                      ( hall_fault                )
);

assign connected = hall_connected & ~(hall_fault);

endmodule

`endif  // DRIBBLER_MOTOR_EN

`endif
