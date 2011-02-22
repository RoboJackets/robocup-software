// THIS IS SAFETY CRITICAL CODE.
// Don't break it.  Don't even touch it unless you fully understand why you
// shouldn't.
//
// This module ensures that both FETs in one half-bridge can't be on at the
// same (or nearly same) time.  There is a delay from turning off one FET to
// turning on the opposing one.
//
// Commanding both FETs on will result in both being turned off.  Every other
// combination is executed as expected, potentially with a delay.

module half_bridge (
    input clk,
    input [1:0] in,
    output reg [1:0] out = 0
);

////////
// Anti-shoot-through timer

// How many cycles to wait
parameter Timer_Stop = 13;

reg timer_run = 0;
reg [3:0] timer = 0;
always @(posedge clk) begin
    if (timer_run == 0) begin
        // Stopped: keep reset
        timer <= 0;
    end else if (timer != Timer_Stop) begin
        // Running, not done
        timer <= timer + 1;
    end
end

wire timer_done = (timer_run == 1 && timer == Timer_Stop);

////////
// State machine

// The last low or high output
reg [1:0] last_active = 0;

always @(posedge clk) begin
    if (in == 2'b00 || in == 2'b11) begin
        // Turn both outputs off immediately.
        out <= 2'b00;
        
        // Start the timer.  The outputs can only change to the opposite
        // value after the timer has run out.
        timer_run <= 1;
    end else if (out == 2'b00 && (in == last_active || timer_done == 1)) begin
        // Requesting high or low outputs.
        // Either the outputs are going to their last value
        // (and thus they won't shoot through) or the timer has expired.
        // Set the requested outputs.
        timer_run <= 0;
        out <= in;
        last_active <= in;
    end else if (in != out) begin
        // Requesting different outputs: turn both off and start the timer
        timer_run <= 1;
        out <= 2'b00;
    end
end

endmodule
