`include "Phase_Driver.vh"
`include "Hall_Effect_Sensor.v"
`include "Phase_Driver.v"
//`include "git_version.vh"


module robocup(
    input clock,
    input [2:0] hall,
    input [`DUTY_CYCLE_WIDTH - 1 : 0] duty_cycle,
    output [2:0] phaseH, phaseL
);

wire [2:0] off_phase;
wire [2:0] high_phase;

Hall_Effect_Sensor hallEffectSensor (clock, hall, high_phase, off_phase);

Phase_Driver phaseDriver1 (clock, (high_phase[2] == 1) ? duty_cycle : 0, off_phase[2], phaseH[2], phaseL[2]);
Phase_Driver phaseDriver2 (clock, (high_phase[1] == 1) ? duty_cycle : 0, off_phase[1], phaseH[1], phaseL[1]);
Phase_Driver phaseDriver3 (clock, (high_phase[0] == 1) ? duty_cycle : 0, off_phase[0], phaseH[0], phaseL[0]);

endmodule
