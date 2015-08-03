// This module reads from the I2C voltage ADC on the kicker.
// The parent is expected to handle the open-collector I2C drivers.  This makes simulation cleaner.

`ifndef _robojackets_robocup_kicker_i2c_
`define _robojackets_robocup_kicker_i2c_

module kicker_i2c (
	input sysclk,
	
	output reg scl_out = 1,
	output reg sda_out = 1,
	input sda_in,
	
	output reg slave_ok = 0,
	output reg [7:0] result = 0
);

localparam State_Start = 0;
localparam State_Slave_Address = 1;
localparam State_Slave_ACK = 2;
localparam State_Data_High = 3;
localparam State_Master_ACK = 4;
localparam State_Data_Low = 5;
localparam State_Master_NACK = 6;
localparam State_Stop = 7;

reg [2:0] state = 0;
reg [2:0] bit_count = 0;
reg [5:0] clock_div = 0;

localparam SCL_Start = 10;
localparam SCL_Fall = 22;
localparam SCL_Setup = 33;
localparam SCL_Period = 46;

// Addresss and R/W bit for the voltage ADC
wire [7:0] Voltage_Select = 8'ha1;

reg sda_in_s = 0;

reg [15:0] voltage_data = 0;

always @(posedge sysclk) begin
	sda_in_s <= sda_in;
	
	if (clock_div == (SCL_Period - 1)) begin
		clock_div <= 0;
	end else begin
		clock_div <= clock_div + 1;
	end

	if (clock_div == 0) begin
		scl_out <= 1;
	end else if (clock_div == SCL_Fall) begin
		scl_out <= 0;
	end
	
	case (state)
		State_Start: begin
			if (clock_div == 0) begin
				sda_out <= 1;
			end else if (clock_div == SCL_Start) begin
				sda_out <= 0;
			end else if (clock_div == SCL_Setup) begin
				state <= State_Slave_Address;
				bit_count <= 0;
			end
		end
		
		State_Slave_Address: begin
			sda_out <= Voltage_Select[7 - bit_count];
			if (clock_div == SCL_Setup) begin
				if (bit_count == 7) begin
					// Release SDA so the slave can acknowledge
					sda_out <= 1;
					state <= State_Slave_ACK;
				end
				bit_count <= bit_count + 1;
			end
		end
		
		State_Slave_ACK: begin
			if (clock_div == 1) begin
				slave_ok = ~sda_in_s;
			end
			if (clock_div == SCL_Setup) begin
				if (slave_ok == 1) begin
					state <= State_Data_High;
				end else begin
					state <= State_Start;
				end
			end
		end
		
		State_Data_High: begin
			// SDA is still open on our end from State_Slave_ACK
			if (clock_div == 1) begin
				voltage_data[15 - bit_count] = sda_in_s;
			end else if (clock_div == SCL_Setup) begin
				if (bit_count == 7) begin
					state <= State_Master_ACK;
					sda_out <= 0;
				end
				bit_count <= bit_count + 1;
			end
		end
		
		State_Master_ACK: begin
			if (clock_div == SCL_Setup) begin
				state <= State_Data_Low;
				sda_out <= 1;
			end
		end
		
		State_Data_Low: begin
			if (clock_div == 1) begin
				voltage_data[7 - bit_count] = sda_in_s;
			end else if (clock_div == SCL_Setup) begin
				if (bit_count == 7) begin
					state <= State_Master_NACK;
				end
				bit_count <= bit_count + 1;
			end
		end
		
		State_Master_NACK: begin
			// SDA is still not driven from the end of State_Master_ACK
			if (clock_div == SCL_Setup) begin
				result <= voltage_data[11:4];
				state <= State_Stop;
				// Drive SDA low so it can go high later in State_Stop
				sda_out <= 0;
			end
		end
		
		State_Stop: begin
			if (clock_div == SCL_Start) begin
				sda_out <= 1;
			end else if (clock_div == SCL_Setup) begin
				state <= State_Start;
			end
		end
	endcase
end

endmodule

`endif