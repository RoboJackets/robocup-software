`timescale 100ns/10ns

`include "kicker_i2c.v"

module main;
	reg clk = 0;
	reg sda_in = 1;
	wire scl_out;
	wire sda_out;
	wire slave_ok;
	wire [7:0] result;
	
	kicker_i2c dut(
		clk,
		scl_out,
		sda_out,
		sda_in,
		slave_ok,
		result
	);
	
	// Clock
	initial begin
		clk = 0;
		forever begin
			#0.5 clk = ~clk;
		end
	end
    
    // I2C slave
    initial begin
		#390 sda_in = 0;
		#46; #46 sda_in = 1;
    end
    
	// Logging
	initial begin
		$dumpvars;
		$dumpall;
		#2000
		$finish;
	end
endmodule