
`include "Phase_Driver.vh"
`include "Motor_Driver.v"


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
reg dir = 0;
wire fault;

wire [2:0] phaseH , phaseL ;


Motor_Driver motorDriver (clock, h, duty_cycle, dir, phaseH , phaseL, fault );

initial begin
	h = 3'b000;
	duty_cycle = 10'b1100000000; //dutycycle
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
