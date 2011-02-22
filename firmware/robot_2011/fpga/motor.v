// Motor driver and encoder

module motor(
    input clk,
    input pwm_clk,
    
    input write,
    input [7:0] data_in,
    
    input [2:0] hall,
    output [5:0] motor_out,
    output [7:0] hall_count,
    output fault
);

// Synchronize the hall inputs to the clock
reg [2:0] hall_sync;
always @(posedge clk) begin
    hall_sync <= hall;
end

// Motor driver
wire [5:0] bridge_drive;
wire pwm_out;
reg direction = 0;
reg [6:0] pwm_level = 0;

pwm pwm(clk, pwm_clk, pwm_level, pwm_out);
bldc bldc(clk, pwm_out, direction, hall_sync, bridge_drive);
half_bridge half_bridge[1:3](clk, bridge_drive, motor_out);

hall_counter hall_counter(clk, hall_sync, hall_count, fault);

reg [19:0] watchdog_timer = 0;

always @(posedge clk) begin
    if (write) begin
        watchdog_timer <= 0;
        {direction, pwm_level} <= data_in;
    end else begin
        if (watchdog_timer == 20'hfffff) begin
             pwm_level <= 0;
        end else begin
            watchdog_timer <= watchdog_timer + 1;
        end
    end
end

endmodule
