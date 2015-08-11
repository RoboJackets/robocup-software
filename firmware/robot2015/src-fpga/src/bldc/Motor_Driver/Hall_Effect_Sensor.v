`include "Phase_Driver_Include.vh"

//http://www.maxonmotor.com/medias/sys_master/root/8815461662750/EC-Technology-short-and-to-the-point-14-EN-32-35.pdf?attachment=true
module Hall_Effect_Sensor(clock,h, u, z);
input clock;
input	[2:0]	h; 	//Hall Effect sensor input
output 	[2:0]	u; 	//duty cycle output
output	[2:0]	z;	//if phase should be high impedance

reg	[2:0]	u;
reg	[2:0]	z;

always @ (posedge clock) begin
	if (h == 3'b000) begin 	//(h1, h2, h3) Failure state
		u = 3'b000;		//(u1, u2, u3)
		z = 3'b111;		//(z1, z2, z3)
	end else if (h == 3'b101) begin
		u = 3'b100;
		z = 3'b001;
	end else if (h == 3'b100) begin
		u = 3'b100;
		z = 3'b010;
	end else if (h == 3'b110) begin
		u = 3'b010;
		z = 3'b100;
	end else if (h == 3'b101) begin
		u = 3'b010;
		z = 3'b001;
	end else if (h == 3'b011) begin
		u = 3'b001;
		z = 3'b010;
	end else if (h == 3'b001) begin
		u = 3'b001;
		z = 3'b100;
	end else if (h == 3'b000) begin //failure state
		u = 3'b000;
		z = 3'b111;	
	end
end

endmodule
