/*
*  BLDC_Hall_Counter.v
*  
*  Give the hall effect readings as an input and this gives the number times they have changed states as an output.
*  A synchronous reset input is used to reset the count number.
*  
*/


// BLDC_Hall_Counter module
module BLDC_Hall_Counter ( clk, reset, hall, count );

// Module parameters
parameter COUNTER_WIDTH;

// Module inputs/outputs
input clk, reset;
input [2:0] hall;
output reg [COUNTER_WIDTH-1:0] count = 0;
// ===============================================


// Local parameters that can not be altered outside of this file
// ===============================================
localparam STEP_1 = 3'b101;
localparam STEP_2 = 3'b100;
localparam STEP_3 = 3'b110;
localparam STEP_4 = 3'b010;
localparam STEP_5 = 3'b011;
localparam STEP_6 = 3'b001;


// Register and Wire declarations
// ===============================================
reg [2:0] hall_d = 0;   // The hall effect sensor value delayed by one clock cycle


// Combinational logic
// ===============================================
assign count_up =
    ( ( hall_d == STEP_1 ) && ( hall == STEP_2 ) ) ||
    ( ( hall_d == STEP_2 ) && ( hall == STEP_3 ) ) ||
    ( ( hall_d == STEP_3 ) && ( hall == STEP_4 ) ) ||
    ( ( hall_d == STEP_4 ) && ( hall == STEP_5 ) ) ||
    ( ( hall_d == STEP_5 ) && ( hall == STEP_6 ) ) ||
    ( ( hall_d == STEP_6 ) && ( hall == STEP_1 ) );

assign count_down =
    ( ( hall_d == STEP_6 ) && ( hall == STEP_5 ) ) ||
    ( ( hall_d == STEP_5 ) && ( hall == STEP_4 ) ) ||
    ( ( hall_d == STEP_4 ) && ( hall == STEP_3 ) ) ||
    ( ( hall_d == STEP_3 ) && ( hall == STEP_2 ) ) ||
    ( ( hall_d == STEP_2 ) && ( hall == STEP_1 ) ) ||
    ( ( hall_d == STEP_1 ) && ( hall == STEP_6 ) );


// Begin main logic
always @(posedge clk) begin : HALL_COUNTER

    hall_d <= hall;

    if ( reset == 1'b1 ) begin
        count <= 0;
    end else begin
        if ( count_up == 1 ) begin
            count <= count + 1;
        end else if ( count_down == 1 ) begin
            count <= count - 1;
        end
    end

end

endmodule
