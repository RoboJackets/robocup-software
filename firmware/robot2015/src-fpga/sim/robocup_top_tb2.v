

module RoboCup_Top_tb;


/*Output values to file */ 
initial begin
    $dumpfile("RoboCup_Top_tb-results.vcd");
    $dumpvars(0,RoboCup_Top_tb);
 end

localparam NUM_MOTORS = 5;

reg clk = 0;
reg [NUM_MOTORS-1:0] halls_a = 0;
reg [NUM_MOTORS-1:0] halls_b = 0;
reg [NUM_MOTORS-1:0] halls_c = 0;
reg [NUM_MOTORS-1:0] encoders_a = 0;
reg [NUM_MOTORS-1:0] encoders_b = 0;
reg spi_slave_miso;
wire spi_master_sck;
reg spi_master_mosi;


wire [NUM_MOTORS-1:0] phases_aH;
wire [NUM_MOTORS-1:0] phases_aL;
wire [NUM_MOTORS-1:0] phases_bH;
wire [NUM_MOTORS-1:0] phases_bL;
wire [NUM_MOTORS-1:0] phases_cH;
wire [NUM_MOTORS-1:0] phases_cL;
wire [NUM_MOTORS-1:0] drv_ncs;
wire [1:0] adc_ncs;
wire spi_master_miso;
reg spi_slave_sck = 0;
wire spi_slave_mosi;
wire spi_slave_ncs;



robocup robocup_top_module (
	.sysclk				(clk),
	.phase_aH 			(phases_aH),
	.phase_aL			(phase_aL),
	.phase_bH			(phase_bH),
	.phase_bL			(phase_bL),
	.phase_cH			(phase_cH),
	.phase_cL			(phase_cL),
	.hall_a 			(halls_a),
	.hall_b 			(halls_b),
	.hall_c 			(halls_c),
	.enc_a 				(encoders_a),
	.enc_b 				(encoders_b),
	.drv_ncs 			(drv_ncs),
	.adc_ncs 			(adc_ncs),
	.spi_slave_sck 		(spi_slave_sck),
	.spi_slave_mosi 	(spi_slave_mosi),
	.spi_slave_ncs 		(spi_slave_ncs),
	.spi_slave_miso 	(spi_slave_miso),
	.spi_master_sck 	(spi_master_sck),
	.spi_master_mosi 	(spi_master_mosi),
	.spi_master_miso 	(spi_master_miso)
);

initial begin
	#50000 $finish;
end

always begin
 	 #1 clk = !clk;
end

always begin
	#500 spi_slave_sck = !spi_slave_sck;
end


always begin
	#0 		halls_a = 5'b00000;
			halls_b = 5'b00000;
			halls_c = 5'b00000;

	#1000 	halls_a = 5'b11111;
			halls_b = 5'b00000;
			halls_c = 5'b11111;

	#1000 	halls_a = 5'b11111;
			halls_b = 5'b00000;
			halls_c = 5'b00000;

	#1000 	halls_a = 5'b11111;
			halls_b = 5'b11111;
			halls_c = 5'b00000;

	#1000 	halls_a = 5'b00000;
			halls_b = 5'b11111;
			halls_c = 5'b00000;

	#1000 	halls_a = 5'b00000;
			halls_b = 5'b11111;
			halls_c = 5'b11111;

	#1000 	halls_a = 5'b00000;
			halls_b = 5'b00000;
			halls_c = 5'b11111;

	#1000 	halls_a = 5'b11111;
			halls_b = 5'b11111;
			halls_c = 5'b11111;
end

endmodule
