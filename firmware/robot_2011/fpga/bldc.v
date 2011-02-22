module pwm_switch (
    input brake,
    input pwm,
    input [1:0] motor_in,
    output [1:0] motor_out
);

parameter HIGH = 2'b10;
parameter LOW  = 2'b01;
parameter OFF  = 2'b00;

assign motor_out = (pwm == 1) ? motor_in :
                   (brake == 1) ? ((motor_in == OFF || motor_in == 2'b11) ? OFF : LOW) :
                   OFF;

endmodule

module bldc (
    input clk,
    input pwm,
    input direction,    // 0=CW, 1=CCW
    
    // Hall-effect sensors
    input [2:0] hall,
    
    // Half-bridge outputs
    output reg [5:0] out = 0
);

parameter HIGH = 2'b10;
parameter LOW  = 2'b01;
parameter OFF  = 2'b00;

// Commutation
wire [5:0] com_out =
    (hall == 3'b101) ? {HIGH, LOW,  OFF} :
    (hall == 3'b100) ? {HIGH, OFF,  LOW} :
    (hall == 3'b110) ? {OFF,  HIGH, LOW} :
    (hall == 3'b010) ? {LOW,  HIGH, OFF} :
    (hall == 3'b011) ? {LOW,  OFF,  HIGH} :
    (hall == 3'b001) ? {OFF,  LOW,  HIGH} :
                       {OFF,  OFF,  OFF};

// Direction
// For CCW: high->low, low->high, 00->11 (off->off)
wire [5:0] dir_out = direction ? ~com_out : com_out;

// PWM
// Just gating the commutation output.
//assign out = pwm ? dir_out : 0;
wire brake = 1;
wire [5:0] out_a;
pwm_switch sa(brake, pwm, dir_out[5:4], out_a[5:4]);
pwm_switch sb(brake, pwm, dir_out[3:2], out_a[3:2]);
pwm_switch sc(brake, pwm, dir_out[1:0], out_a[1:0]);

always @(posedge clk) begin
    out <= out_a;
end

endmodule
