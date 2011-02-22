module pwm (
    input clk,
    input pwm_clk,
    input [6:0] level,
    output reg out = 0
);

reg [6:0] counter = 0;
reg [6:0] level_effective = 0;
always @(posedge clk) begin
    if (pwm_clk) begin
        // Count to 7E so a level of 7F will produce a 100% duty cycle.
        if (counter == 7'h7e) begin
            counter <= 0;
            
            // Use the new level
            level_effective <= level;
        end else begin
            counter <= counter + 1;
        end
    end
    
    out <= counter < level_effective;
end

endmodule
