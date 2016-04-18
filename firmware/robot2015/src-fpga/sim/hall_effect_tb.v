
`include "Hall_Effect_Sensor.v"


module hall_effect_tb;

/*Output values to 'test.vcd' file */
initial begin
    $dumpfile("test.vcd");
    $dumpvars(0, hall_effect_tb);
end

reg clk, dir;
reg [2:0] h;

wire [2:0] u;
wire [2:0] z;

initial begin
    clk = 1;
    dir = 1;
end

always begin
    forever #16000 dir = !dir;
end

always begin
    forever #0.5 clk = !clk;
end

always begin
	#1000 h = 3'b000;
	#1000 h = 3'b101;
	#1000 h = 3'b100;
	#1000 h = 3'b110;
	#1000 h = 3'b010;
	#1000 h = 3'b011;
	#1000 h = 3'b001;
	#1000 h = 3'b111;
end

Hall_Effect_Sensor hallEffectSensor(h, dir, u, z);

endmodule
