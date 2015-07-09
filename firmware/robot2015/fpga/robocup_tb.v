
`include "Phase_Driver.vh"
`include "robocup.v"


module Motor_Driver_tb;

/*Output values to 'test.vcd' file */ 
initial
 begin
    $dumpfile("test.vcd");
    $dumpvars(0,Motor_Driver_tb);
 end

reg clock;
reg [2:0] h;
reg [`DUTY_CYCLE_WIDTH - 1 : 0] duty_cycle;


wire [2:0] phaseH , phaseL ;


robocup motorDriver (clock, h, duty_cycle, phaseH , phaseL );

initial begin
	h = 3'b000;
	duty_cycle = 8'h80; //dutycycle
	clock = 0;

end


 always begin
 	 #1 clock = !clock;
 end

 always begin
 	#360000 $finish;
 end

 always begin
 
 	
 	
 
 	
	#10000 h = 3'b001;
	#10000 h = 3'b011;
	#10000 h = 3'b010;
	#10000 h = 3'b110;
	#10000 h = 3'b100;
	#10000 h = 3'b101;



	
	

end

endmodule
