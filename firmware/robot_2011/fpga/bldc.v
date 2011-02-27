module bldc (
	input direction,    // 0=CW, 1=CCW
	
	// Hall-effect sensors
	input [2:0] hall,
	
	// Half-bridge outputs
	output [5:0] out
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
assign out = direction ? ~com_out : com_out;

endmodule
