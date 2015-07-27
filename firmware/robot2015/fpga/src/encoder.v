
`ifndef _robojackets_robocup_encoder_
`define _robojackets_robocup_encoder_

module encoder (
    input clk,
    input [1:0] enc,
    output reg [15:0] count = 0
);

reg [1:0] enc_sync = 0;
reg [1:0] last_enc = 0;

// 00
// 01
// 11
// 10

wire count_up =
    (last_enc == 2'b00 && enc_sync == 2'b01) ||
    (last_enc == 2'b01 && enc_sync == 2'b11) ||
    (last_enc == 2'b11 && enc_sync == 2'b10) ||
    (last_enc == 2'b10 && enc_sync == 2'b00);

wire count_down =
    (last_enc == 2'b10 && enc_sync == 2'b11) ||
    (last_enc == 2'b11 && enc_sync == 2'b01) ||
    (last_enc == 2'b01 && enc_sync == 2'b00) ||
    (last_enc == 2'b00 && enc_sync == 2'b10);

always @(posedge clk) begin
    if (count_up) begin
        count <= count + 1;
    end else if (count_down) begin
        count <= count - 1;
    end

    enc_sync <= enc;
    last_enc <= enc_sync;
end

endmodule

`endif
