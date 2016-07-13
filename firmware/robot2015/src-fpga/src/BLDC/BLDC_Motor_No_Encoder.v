/*
*  BLDC_Motor.v
*
*  A BLDC controller that includes hall sensor error detection, but
*  does not have any support for encoders.
*
*/

`ifndef _BLDC_MOTOR_NO_ENCODER_
`define _BLDC_MOTOR_NO_ENCODER_

`include "robocup.vh"

`include "BLDC_Driver.v"
`include "BLDC_Hall_Counter.v"


// BLDC_Motor module - no encoder
module BLDC_Motor_No_Encoder ( clk, en, reset_hall_count, duty_cycle, hall, phaseH, phaseL, hall_count, connected );

// Module parameters - passed parameters will overwrite the values here
parameter MAX_DUTY_CYCLE =          ( 'h1FF             );
parameter MAX_DUTY_CYCLE_COUNTER =  ( MAX_DUTY_CYCLE    );
parameter HALL_COUNT_WIDTH =        ( 7                 );

// Local parameters - can not be altered outside this module
`include "log2-macro.v"     // This must be included here
localparam DUTY_CYCLE_WIDTH =   `LOG2( MAX_DUTY_CYCLE );

// Module inputs/outputs
input clk, en, reset_hall_count;
input [DUTY_CYCLE_WIDTH-1:0] duty_cycle;
input [2:0] hall;
output [2:0] phaseH, phaseL;
output [HALL_COUNT_WIDTH-1:0] hall_count;
output connected;
// ===============================================

wire hall_connected, hall_fault;

// Show the expected startup length during synthesis. Assumes an 18.432MHz input clock.
initial begin
    $display ("Duty cycle width from BLDC_Motor_No_Encoder.v:\t%d", DUTY_CYCLE_WIDTH);
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
    .PHASE_DRIVER_MAX_COUNTER   ( MAX_DUTY_CYCLE_COUNTER    ) ,
    .MAX_DUTY_CYCLE             ( MAX_DUTY_CYCLE            ) ,
    .DUTY_CYCLE_STEP_RES        ( 1                         ) ,
    .DEAD_TIME                  ( 20                        )
    ) bldc_motor (
    .clk                        ( clk                       ) ,
    .en                         ( en                        ) ,
    .hall                       ( hall                      ) ,
    .direction                  ( 'b0                       ) ,
    .duty_cycle                 ( { duty_cycle[DUTY_CYCLE_WIDTH-2:0], 1'b0 } ) ,
    .phaseH                     ( phaseH                    ) ,
    .phaseL                     ( phaseL                    ) ,
    .connected                  ( hall_connected            ) ,
    .fault                      ( hall_fault                )
);

assign connected = hall_connected & ~(hall_fault);

wire pid_clk;

// PI controller's clock
ClkDivide #(
  .WIDTH    ( 16                        )
  ) wdt_clk_gen (
  .CLK_IN   ( clk                       ) ,
  .EN       ( en                        ) ,
  .CLK_OUT  ( pid_clk                   )
);

endmodule

`endif // _BLDC_MOTOR_
