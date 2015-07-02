`include "phase_driver.vh"
`include "Hall_Effect_Sensor.v"
`include "Phase_Driver.v"
`include "git_version.vh"


module robocup(clock, h, phaseHInv, phaseLInv); //re-add "duty_cycle,"  as port 2 after done testing

input clock;
input [2:0] h;
//input [`DUTY_CYCLE_WIDTH - 1 : 0] duty_cycle; re-add this line when done testing //TODO make FPGA read dutycycle
wire [2:0] z;
wire [2:0] u;

output [2:0] phaseHInv, phaseLInv;
wire [2:0] phaseHInv, phaseLInv, phaseH, phaseL;
reg [`DUTY_CYCLE_WIDTH - 1 : 0] tempDC = 10'h00F;

//Motor driver IRS2336DS requires inverted phase driver input
assign phaseHInv = phaseH; //Jon's driver  doesn't need inverted outputs
assign phaseLInv = phaseL; //TODO remove inverted outputs

Hall_Effect_Sensor hallEffectSensor (clock, h, u, z);

Phase_Driver phaseDriver1 (clock, (u[2] == 1) ? tempDC/*duty_cycle*/ : 0, z[2], phaseH[2], phaseL[2]);
Phase_Driver phaseDriver2 (clock, (u[1] == 1) ? tempDC/*duty_cycle*/ : 0, z[1], phaseH[1], phaseL[1]);
Phase_Driver phaseDriver3 (clock, (u[0] == 1) ? tempDC/*duty_cycle*/ : 0, z[0], phaseH[0], phaseL[0]);

endmodule
