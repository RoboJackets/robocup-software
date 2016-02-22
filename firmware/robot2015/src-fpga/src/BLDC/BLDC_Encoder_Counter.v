/*
*  BLDC_Encoder_Counter.v
*  
*  Give the encoder readings as an input and this gives the number times they have changed states as an output.
*  A synchronous reset input is used to reset the count number.
*  
*/

`ifndef _BLDC_ENCODER_COUNTER_
`define _BLDC_ENCODER_COUNTER_

// BLDC_Driver module
module BLDC_Encoder_Counter ( clk, reset, enc, count );

// Module parameters
parameter COUNT_WIDTH = ( 15 );

// Module inputs/outputs
input clk, reset;
input [1:0] enc;
output reg [COUNT_WIDTH-1:0] count = 0;
// ===============================================


// Local parameters that can not be altered outside of this file.
// ===============================================
localparam STEP_0 = 'b00;
localparam STEP_1 = 'b01;
localparam STEP_2 = 'b10;
localparam STEP_3 = 'b11;


// Register and Wire declarations
// ===============================================
reg [1:0] enc_d;    // The delayed encoder tick by one clock cycle

wire count_up =
    ( ( enc_d == STEP_0 ) && ( enc == STEP_1 ) ) ||
    ( ( enc_d == STEP_1 ) && ( enc == STEP_3 ) ) ||
    ( ( enc_d == STEP_3 ) && ( enc == STEP_2 ) ) ||
    ( ( enc_d == STEP_2 ) && ( enc == STEP_0 ) );

wire count_down =
    ( ( enc_d == STEP_2 ) && ( enc == STEP_3 ) ) ||
    ( ( enc_d == STEP_3 ) && ( enc == STEP_1 ) ) ||
    ( ( enc_d == STEP_1 ) && ( enc == STEP_0 ) ) ||
    ( ( enc_d == STEP_0 ) && ( enc == STEP_2 ) );


// Begin main logic
always @( posedge clk ) begin : ENCODER_COUNTER

    enc_d <= enc;

    if ( reset == 1 ) begin
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

`endif
