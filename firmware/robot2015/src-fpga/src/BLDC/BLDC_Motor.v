/*
*  BLDC_Motor.v
*  
*  A BLDC controller that includes hall sensor error detection and incremental
*  duty cycles during startup for reducing motor startup current.
*  
*/

`ifndef _BLDC_MOTOR_
`define _BLDC_MOTOR_

`include "BLDC_Driver.v"
`include "BLDC_Hall_Counter.v"
`include "BLDC_Encoder_Counter.v"


/*
============================ 
DRV8303 REGISTER INFORMATION
============================

SPI Format:
============================
- 16-bit length
- 1 read/write bit [15]
- 4 address bits [14..11]
- 11 data bits [10..0]
============================

Address 0x00: R (auto reset after read)
============================
bit [10]:   FAULT
bit [9]:    GVDD_UV
bit [8]:    PVDD_UV
bit [7]:    OTSD
bit [6]:    OTW
bit [5]:    AH_OC
bit [4]:    AL_OC
bit [3]:    BH_OC
bit [2]:    BL_OC
bit [1]:    CH_OC
bit [0]:    CL_OC
============================

Address 0x01: R (auto reset after read)
============================
bit  [7]:      GVDD_OV
bits [6..4]:   RESERVED
bits [3..0]:   DEV_ID
============================

Address 0x02: R/W
============================
bits [10..6]: OC_ADJ_SET (write with 0x0D everytime)
bits [5..4]:  OC_MODE
bit  [3]:     PWM_MODE
bit  [2]:     GATE_RESET
bits [1..0]:  GATE_CURRENT
============================

Address 0x03: R/W
============================
bits [10..7]:   RESERVED
bit  [6]:       OC_TOFF
bit  [5]:       DC_CAL_CH2 (write 0x01 here, then read DC offset value and write back to 0x00)
bit  [4]:       DC_CAL CH1 (write 0x01 here, then read DC offset value and write back to 0x00)
bits [3..2]:    GAIN (set to 0x01 everytime)
bits [1..0]:    OCTW_SET
============================
*/


// BLDC_Motor module
module BLDC_Motor ( clk, en, reset_enc_count, reset_hall_count, duty_cycle, enc, hall, phaseH, phaseL, enc_count, hall_count, connected, hall_fault );

// Module parameters - passed parameters will overwrite the values here
parameter MIN_DUTY_CYCLE =          ( 0 );
parameter MAX_DUTY_CYCLE =          ( 'h3FF );
parameter MAX_DUTY_CYCLE_COUNTER =  ( MAX_DUTY_CYCLE );
parameter ENCODER_COUNT_WIDTH =     ( 15 );
parameter HALL_COUNT_WIDTH =        ( 7 );

// Local parameters - can not be altered outside this module
`include "log2-macro.v"     // This must be included here
localparam DUTY_CYCLE_WIDTH =   `LOG2( MAX_DUTY_CYCLE );

// Module inputs/outputs
input clk, en, reset_enc_count, reset_hall_count;
input [DUTY_CYCLE_WIDTH-1:0] duty_cycle;
input [1:0] enc;
input [2:0] hall;
output [2:0] phaseH, phaseL;
output [ENCODER_COUNT_WIDTH-1:0] enc_count;
output [HALL_COUNT_WIDTH-1:0] hall_count;
output connected;
output hall_fault;
// ===============================================

// Show the expected startup length during synthesis. Assumes an 18.432MHz input clock.
initial begin
    $display ("Duty cycle width from BLDC_Motor.v:\t%d", DUTY_CYCLE_WIDTH);
end

// Instantiation of all the modules required for complete functioning with all sensors
// =============================================connected==
BLDC_Encoder_Counter #(         // Instantiation of the encoder for counting the ticks
    .COUNT_WIDTH                ( ENCODER_COUNT_WIDTH )
    ) encoder_counter (
    .clk                        ( clk ) ,
    .reset                      ( reset_enc_count ) ,
    .enc                        ( enc ) ,
    .count                      ( enc_count )
);

BLDC_Hall_Counter #(            // Instantiation of the hall effect sensor's counter
    .COUNTER_WIDTH              ( HALL_COUNT_WIDTH )
    ) hall_counter (
    .clk                        ( clk ) ,
    .reset                      ( reset_hall_count ) ,
    .hall                       ( hall ) ,
    .count                      ( hall_count )
);

BLDC_Driver #(                  // Instantiation of the motor driving module
    .PHASE_DRIVER_MAX_COUNTER   ( MAX_DUTY_CYCLE_COUNTER ) ,
    .MAX_DUTY_CYCLE             ( MAX_DUTY_CYCLE ) ,
    .MIN_DUTY_CYCLE             ( MIN_DUTY_CYCLE ) ,
    .DUTY_CYCLE_STEP_RES        ( 1 ) ,
    .DEAD_TIME                  ( 2 )
    ) bldc_motor (
    .clk                        ( clk ) ,
    .en                         ( en ) ,
    .hall                       ( hall ) ,
    .duty_cycle                 ( duty_cycle ) ,
    .phaseH                     ( phaseH ) ,
    .phaseL                     ( phaseL ) ,
    .connected                  ( connected ) ,
    .fault                      ( hall_fault )
);

endmodule

`endif
