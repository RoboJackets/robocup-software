`include "Phase_Driver_Include.vh"

module Motor_Driver(clk, duty_cycle, h, phaseHInv, phaseLInv);

input clk;
input [2:0] h;
input [`DUTY_CYCLE_WIDTH - 1 : 0] duty_cycle;
wire [2:0] z;
wire [2:0] u;

output [2:0] phaseHInv, phaseLInv;
//assign duty_cycle = 8'h81;
wire [2:0] phaseHInv, phaseLInv, phaseH, phaseL;

//Motor driver IRS2336DS requires inverted phase driver input
assign phaseHInv = ~phaseH;
assign phaseLInv = ~phaseL;

Hall_Effect_Sensor hallEffectSensor (clk, h, u, z);

genvar i;
generate for (i = 0; i < 3; i = i + 1) begin
    Phase_Driver phaseDriver1 (clk, (u[i] == 1) ? duty_cycle : 0, z[i], phaseH[i], phaseL[i]);
end endgenerate

endmodule
