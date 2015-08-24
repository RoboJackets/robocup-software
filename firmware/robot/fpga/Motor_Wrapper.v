`include "../../robot2015/fpga/Phase_Driver.vh"
`include "../../robot2015/fpga/Hall_Effect_Sensor.v"
`include "../../robot2015/fpga/Phase_Driver.v"
`include "../../robot2015/fpga/Motor_Driver.v"

module Motor_Wrapper (
    input clock,
    input pwm_phase,
    input dir,
    input duty_cycle,
    input drive_mode,
    input hall,
    output wire [5:0] phase_outputs,
    output fault
    );
wire [2:0] phaseH, phaseL;

Motor_Driver motorDriver(clock, hall, duty_cycle, dir, phaseH, phaseL, fault);
assign phase_outputs = {phaseH[2], phaseL[2], phaseH[1], phaseL[1], phaseH[0], phaseL[0]};
//sysclk, pwm_phase, motor_dir[1], motor_speed_1, drive_mode_1,
// hall_sync_1, {m1a_h, m1a_l, m1b_h, m1b_l, m1c_h, m1c_l}, motor_fault[1])

endmodule
