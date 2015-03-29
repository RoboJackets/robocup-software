`include "phase_driver.vh"

module Motor_Driver(clock, duty_cycle, h, phase1h, phase1l, phase2h, phase2l, phase3h, phase3l);

input clock;
input [2:0] h;
input [`DUTY_CYCLE_WIDTH - 1 : 0] duty_cycle;
wire [2:0] z;
wire [2:0] u;

output phase1h, phase1l, phase2h, phase2l, phase3h, phase3l;

wire phase1h, phase1l, phase2h, phase2l, phase3h, phase3l;

Hall_Effect_Sensor hallEffectSensor (h, u, z);

Phase_Driver phaseDriver1 (clock, (u[2] == 1) ? duty_cycle : 0, z[2], phase1h, phase1l);
Phase_Driver phaseDriver2 (clock, (u[1] == 1) ? duty_cycle : 0, z[1], phase2h, phase2l);
Phase_Driver phaseDriver3 (clock, (u[0] == 1) ? duty_cycle : 0, z[0], phase3h, phase3l);

endmodule