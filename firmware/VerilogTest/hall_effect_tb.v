
//`include "phase_driver.vh"

module hall_effect_tb;

/*Output values to 'test.vcd' file */ 
initial
 begin
    $dumpfile("test.vcd");
    $dumpvars(0,hall_effect_tb);
 end
reg clock;
reg [2:0] h;
wire [2:0] u;
wire [2:0] z;


 always begin
 	 #1 clock = !clock;
 end

always begin
	#0 h = 3'b000;
	#1 h = 3'b101;
	#2 h = 3'b100;
	#3 h = 3'b110;
	#4 h = 3'b010;
	#5 h = 3'b011;
	#6 h = 3'b001;
	#7 h = 3'b111;
end

Hall_Effect_Sensor hallEffectSensor(clock, h,u,z);


endmodule
