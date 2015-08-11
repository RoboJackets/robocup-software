`include "Phase_Driver_Include.vh"
`include "Motor_Driver.v"
`include "Hall_Counter.v"
`include "Encoder_Counter.v"


module BLDC_Motor(clk, duty_cycle, hall, enc, phaseH, phaseL, hall_count, enc_count);

input clk;
input [`DUTY_CYCLE_WIDTH - 1 : 0] duty_cycle;
input [2:0] hall;
input [1:0] enc;
output [2:0] phaseH, phaseL;
output [7:0] hall_count;
output [15:0] enc_count;

Motor_Driver        motor_driver_module(clk, duty_cycle, hall, phaseH, phaseL);
Encoder_Counter     encoder_counter_module(clk, enc, enc_count);
Hall_Counter        hall_counter_module(clk, hall, hall_count);

endmodule
