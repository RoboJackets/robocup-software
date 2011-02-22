`timescale 1ns/1ns

`include "half_bridge.v"

module main;
    reg clk;
    
    // Clock
    initial begin
        clk = 0;
        forever begin
            #50 clk = ~clk;
        end
    end
    
    reg [1:0] in;
    wire [1:0] out;
    
    // Split into wires to make the waveforms easier to read in gtkwave
    wire in_h, in_l, out_h, out_l;
    assign {in_h, in_l} = in;
    assign {out_h, out_l} = out;
    
    half_bridge hb(clk, in, out);
    
    // Test stimulus
    integer i;
    initial begin
        in = 2'b00;
        
        #25;
        
        // Glitch test:
        // Set an output, wait for it to be valid,
        // set both off, and see if the outputs turn back on too quickly.
                in = 2'b01;
        #1600   in = 2'b00;
        #100    in = 2'b10;
        #1600;
        
        // Low -> High
                in = 2'b01;
        #1600   in = 2'b10;
        #1600;
        
        // High -> Low
                in = 2'b01;
        #1600;
        
        // Shoot-through test:
        // Go from every possible input to shoot-through.
        // Outputs should be off when input is 11.
        for (i = 0; i < 4; i=i+1) begin
                in = i;
                #1600 in = 2'b11;
                #1600;
        end
        #1600;
        
        $finish;
    end
    
    // Shoot-through detection
    always @(out) begin
        if (out == 2'b11) begin
            $display("FAILED: Shoot-through");
            $finish;
        end
    end
    
    // Logging
    initial begin
        $dumpvars;
        $dumpall;
    end
endmodule
