
`ifndef _robojackets_robocup_kicker_
`define _robojackets_robocup_kicker_

module kicker (
    input clk,
    
    input manual,
    input strobe,
    input [7:0] strength,
    
    input charge_enable,
    
    output charge,
    output fire,
    output lockout
);

localparam Prescale_Divisor = 575;
localparam Lockout_Time = 5119;

localparam State_Charge = 0;
localparam State_Fire = 1;
localparam State_Lockout = 2;

reg [9:0] prescalar = 0;
reg [2:0] state = State_Charge;
reg [7:0] trigger_counter = 0;
reg [13:0] lockout_counter = 0;

assign charge = (state == State_Charge) & charge_enable;
assign fire = (state == State_Fire);
assign lockout = (state == State_Lockout);

wire run_prescalar = (state == State_Fire) || (state == State_Lockout);
wire prescale_clock = (prescalar == Prescale_Divisor);

always @(posedge clk) begin
    if (run_prescalar == 0 || prescale_clock) begin
        prescalar <= 0;
    end else begin
        prescalar <= prescalar + 1;
    end
    
    case (state)
        State_Charge: begin
            if (strobe) begin
                trigger_counter <= strength;
                state <= State_Fire;
            end else if (manual) begin
                trigger_counter <= 255;
                state <= State_Fire;
            end
        end
        
        State_Fire: begin
            if (prescale_clock) begin
                if (trigger_counter == 0) begin
                    // Done firing, go to lockout.
                    lockout_counter <= 0;
                    state <= State_Lockout;
                end else begin
                    trigger_counter <= trigger_counter - 1;
                end
            end
        end
        
        State_Lockout: begin
            if (strobe) begin
                // Keep the lockout timer reset if we are getting
                // repeated kick commands.
                lockout_counter <= 0;
            end else if (prescale_clock) begin
                if (lockout_counter == Lockout_Time) begin
                    // Lockout time done
                    state <= State_Charge;
                end else begin
                    lockout_counter <= lockout_counter + 1;
                end
            end
        end
    endcase
end

endmodule

`endif
