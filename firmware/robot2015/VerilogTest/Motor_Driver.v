`include "phase_driver.vh"

module Motor_Driver(clock, duty_cycle, h, phaseH, phaseL);

input clock;
input [2:0] h;
input [`DUTY_CYCLE_WIDTH - 1 : 0] duty_cycle;
wire [2:0] z;
wire [2:0] u;

output [2:0] phaseH, phaseL;

wire [2:0] phaseH, phaseL;

Hall_Effect_Sensor hallEffectSensor (clock, h, u, z);

Phase_Driver phaseDriver1 (clock, (u[2] == 1) ? duty_cycle : 0, z[2], phaseH[2], phaseL[2]);
Phase_Driver phaseDriver2 (clock, (u[1] == 1) ? duty_cycle : 0, z[1], phaseH[1], phaseL[1]);
Phase_Driver phaseDriver3 (clock, (u[0] == 1) ? duty_cycle : 0, z[0], phaseH[0], phaseL[0]);

endmodule