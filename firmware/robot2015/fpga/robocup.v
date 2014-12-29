//  This is the "root" Verilog file for the robot

module robocup (
    input sysclk,
    output reg justin
);

reg a_value = 0;
wire bit = a_value;

always @(posedge sysclk)
begin
    justin <= bit;
end

endmodule
