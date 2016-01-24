`timescale 10ns/1ns

module RoboCup_Top_tb;

/*Output values to file */ 
initial begin
    $dumpfile("RoboCup_Top_tb-results.vcd");
    $dumpvars(0,RoboCup_Top_tb);
end

localparam NUM_MOTORS = 5;
localparam SPI_SLAVE_DATA_WIDTH = 8;
localparam SPI_S_CPOL = 0;
localparam SPI_S_CPHA = 0;

reg clk;
reg [NUM_MOTORS-1:0] halls_a;
reg [NUM_MOTORS-1:0] halls_b;
reg [NUM_MOTORS-1:0] halls_c;
reg [NUM_MOTORS-1:0] encoders_a = 0;
reg [NUM_MOTORS-1:0] encoders_b = 0;
reg spi_slave_sck = 1;
reg spi_slave_mosi = 1;
reg spi_slave_ncs = 1;

wire [NUM_MOTORS-1:0] phases_aH;
wire [NUM_MOTORS-1:0] phases_aL;
wire [NUM_MOTORS-1:0] phases_bH;
wire [NUM_MOTORS-1:0] phases_bL;
wire [NUM_MOTORS-1:0] phases_cH;
wire [NUM_MOTORS-1:0] phases_cL;
wire [NUM_MOTORS-1:0] drv_ncs;
wire [1:0] adc_ncs;
wire spi_master_sck;
wire spi_master_miso;
wire spi_master_mosi;
wire spi_slave_miso;


robocup #(
    .NUM_MOTORS             ( NUM_MOTORS ),
    .SPI_MASTER_CPOL        (  0 ),
    .SPI_MASTER_CPHA        (  0 ),
    .SPI_MASTER_DATA_WIDTH  ( 16 ),
    .SPI_SLAVE_CPOL         (  SPI_S_CPOL ),
    .SPI_SLAVE_CPHA         (  SPI_S_CPHA ),
    .SPI_SLAVE_DATA_WIDTH   ( SPI_SLAVE_DATA_WIDTH )
    ) robocup_top_module (
	.sysclk				(clk),
	.phase_aH 			(phases_aH[NUM_MOTORS-1:0]),
	.phase_aL			(phases_aL[NUM_MOTORS-1:0]),
	.phase_bH			(phases_bH[NUM_MOTORS-1:0]),
	.phase_bL			(phases_bL[NUM_MOTORS-1:0]),
	.phase_cH			(phases_cH[NUM_MOTORS-1:0]),
	.phase_cL			(phases_cL[NUM_MOTORS-1:0]),
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

// This is set when the inputs should begin to simulation motors spinning
reg start_spinning_motors = 0;

task spi_on;
    begin
        spi_slave_ncs = 0; #1;
    end
endtask


task spi_off;
    begin
        #1 spi_slave_ncs = 1; #4;
    end
endtask


task spi;
    input [SPI_SLAVE_DATA_WIDTH-1:0] in;
    integer i;
    begin
        if (SPI_S_CPHA) begin
            for ( i = SPI_SLAVE_DATA_WIDTH-1; i >= 0; i = i - 1 ) begin
                spi_slave_sck = !SPI_S_CPOL;  
                spi_slave_mosi = in[i]; #16;
            end
        end else begin
            for ( i = SPI_SLAVE_DATA_WIDTH-1; i >= 0; i = i - 1 ) begin
                    spi_slave_mosi = in[i]; #8;
                    spi_slave_sck = !SPI_S_CPOL; #8;
            end
        end
    end
endtask

initial begin
    forever begin
        @(spi_slave_sck) begin
            #8 spi_slave_sck = SPI_S_CPOL;
        end
    end
end

// Valid hall sensor states
reg [2:0] hall_states [5:0];

// Initilization of everything
initial begin
    clk = 1;
    spi_slave_mosi = 0;
    spi_slave_sck = SPI_S_CPOL;

    hall_states[0] = 3'b101;
    hall_states[1] = 3'b100;
    hall_states[2] = 3'b110;
    hall_states[3] = 3'b010;
    hall_states[4] = 3'b011;
    hall_states[5] = 3'b001;

    #200 start_spinning_motors = 1;
end

task motors_on;
    begin
        spi_on();
        spi(8'hb0);
        spi(8'h00);
        spi_off();
    end
endtask

task motors_off;
    begin
        spi_on();
        spi(8'h30);
        spi(8'h00);
        spi_off();
    end
endtask

reg [10:0] duty_cycle_level;
integer i;
// Send an SPI transfer on the slave bus once the motors are up and running
initial wait ( start_spinning_motors ) begin
    duty_cycle_level = 0;

    // Read encoder counts & update motors - dual transfer type
    #100 spi_on();
    spi(8'h80);
    spi(8'hec);     // Duty cycle 0 low bits
    spi(8'h03);     // Duty cycle 0 top bits
    spi(8'hbd);
    spi(8'h01);
    spi(8'he3);
    spi(8'h01);
    spi(8'h4b);
    spi(8'h01);
    spi(8'hf6);
    spi(8'h02);
    spi_off();
    
    // Read hall counts
    #100 spi_on();
    spi(8'h92);
    spi(8'h00);     // Anything can be sent for all of these
    spi(8'h00);
    spi(8'h00);
    spi(8'h00);
    spi(8'h00);
    spi_off();

    // Set hall counts
    #100 spi_on();
    spi(8'h12);
    spi(8'h08);
    spi(8'h0a);
    spi(8'h02);
    spi(8'h05);
    spi(8'h03);
    spi_off();

    // Now read them back
    #100 spi_on();
    spi(8'h92);
    spi(8'h00);     // Anything can be sent for all of these
    spi(8'h00);
    spi(8'h00);
    spi(8'h00);
    spi(8'h00);
    spi_off();

    // Read duty cycles
    #100 spi_on();
    spi(8'h93);
    spi(8'h00);
    spi(8'h00);
    spi(8'h00);
    spi(8'h00);
    spi(8'h00);
    spi(8'h00);
    spi(8'h00);
    spi(8'h00);
    spi(8'h00);
    spi(8'h00);
    spi_off();

    // Read duty cycles, but send data & end it early (shouldn't affect anything)
    #100 spi_on();
    spi(8'h93);
    spi(8'h08);
    spi(8'h0a);
    spi(8'h02);
    spi(8'h05);
    spi(8'h03);
    spi_off();

    // Span all SPI command byte values
    // for ( i = 0; i < (1 << 8); i = i + 1 ) begin
    //     #1000 spi_on();
    //     spi(i);
    //     spi_off();
    // end

    // repeat(5) begin
    //     #100000 motors_off();
    //     #100000 motors_on();
    // end

    // Increment duty cycles
    forever begin
        #5000 spi_on();
        spi(8'h80);
        for ( i = 0; i < NUM_MOTORS; i = i + 1 ) begin
            spi(duty_cycle_level[7:0]);
            spi(duty_cycle_level[10:8]>>8);
        end
        spi_off();
        duty_cycle_level = duty_cycle_level + 10;
    end
end


// Encoder input simulation
initial begin	
    encoders_a = 5'b11111;
	#20 encoders_b = 5'b00000;
	forever wait ( start_spinning_motors ) begin
        #40 encoders_a = ~encoders_a;
        #40 encoders_b = ~encoders_b;
    end
end


// Simulation of hall sensors transistions. Each motor's phase is slightly delayed
integer ii, jj;
always wait ( start_spinning_motors ) begin
	for ( jj = 0; jj < 6; jj = jj + 1 ) begin
		#10000 for ( ii = 0; ii < NUM_MOTORS; ii = ii + 1 ) begin
			#100 { halls_a[ii], halls_b[ii], halls_c[ii] } = hall_states[jj];
		end
	end
end


initial begin
	forever #0.5 clk = !clk;
	// #500000 $finish;
end


endmodule
