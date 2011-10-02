module hall_counter (
    input clk,
    input [2:0] hall,
    output reg [7:0] count = 0
);

reg [2:0] last_hall = 0;

parameter STEP_1 = 3'b101;
parameter STEP_2 = 3'b100;
parameter STEP_3 = 3'b110;
parameter STEP_4 = 3'b010;
parameter STEP_5 = 3'b011;
parameter STEP_6 = 3'b001;

assign count_up =
    (last_hall == STEP_1 && hall == STEP_2) ||
    (last_hall == STEP_2 && hall == STEP_3) ||
    (last_hall == STEP_3 && hall == STEP_4) ||
    (last_hall == STEP_4 && hall == STEP_5) ||
    (last_hall == STEP_5 && hall == STEP_6) ||
    (last_hall == STEP_6 && hall == STEP_1);

assign count_down =
    (last_hall == STEP_6 && hall == STEP_5) ||
    (last_hall == STEP_5 && hall == STEP_4) ||
    (last_hall == STEP_4 && hall == STEP_3) ||
    (last_hall == STEP_3 && hall == STEP_2) ||
    (last_hall == STEP_2 && hall == STEP_1) ||
    (last_hall == STEP_1 && hall == STEP_6);

always @(posedge clk) begin
    if (count_up) begin
        count <= count + 1;
    end else if (count_down) begin
        count <= count - 1;
    end

    last_hall <= hall;
end

endmodule
