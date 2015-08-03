`timescale 100ns/10ns

`include "robocup.v"

module main;
    reg clk = 0;
    
    reg m1hall_a = 0, m1hall_b = 0, m1hall_c = 0;
    reg m2hall_a = 0, m2hall_b = 0, m2hall_c = 0;
    reg m3hall_a = 0, m3hall_b = 0, m3hall_c = 0;
    reg m4hall_a = 0, m4hall_b = 0, m4hall_c = 0;
    reg m5hall_a = 0, m5hall_b = 0, m5hall_c = 0;
    wire m1a_h, m1a_l, m1b_h, m1b_l, m1c_h, m1c_l;
    wire m2a_h, m2a_l, m2b_h, m2b_l, m2c_h, m2c_l;
    wire m3a_h, m3a_l, m3b_h, m3b_l, m3c_h, m3c_l;
    wire m4a_h, m4a_l, m4b_h, m4b_l, m4c_h, m4c_l;
    wire m5a_h, m5a_l, m5b_h, m5b_l, m5c_h, m5c_l;
    wire m1enc_a, m1enc_b;
    wire m2enc_a, m2enc_b;
    wire m3enc_a, m3enc_b;
    wire m4enc_a, m4enc_b;
    
    wire discharge = 0;
    wire kdone, kcharge, kkick, kchip, kvdata, kidata, kncs, kclk, klimit, kspare;
    reg fpga_ncs = 1, mosi = 0, sck = 0;
    wire miso, flash_ncs;
    wire fs0, fs1;
    
    robocup top(
        clk,
        m1hall_a, m1hall_b, m1hall_c,
        m2hall_a, m2hall_b, m2hall_c,
        m3hall_a, m3hall_b, m3hall_c,
        m4hall_a, m4hall_b, m4hall_c,
        m5hall_a, m5hall_b, m5hall_c,
        m1a_h, m1a_l, m1b_h, m1b_l, m1c_h, m1c_l,
        m2a_h, m2a_l, m2b_h, m2b_l, m2c_h, m2c_l,
        m3a_h, m3a_l, m3b_h, m3b_l, m3c_h, m3c_l,
        m4a_h, m4a_l, m4b_h, m4b_l, m4c_h, m4c_l,
        m5a_h, m5a_l, m5b_h, m5b_l, m5c_h, m5c_l,
        m1enc_a, m1enc_b,
        m2enc_a, m2enc_b,
        m3enc_a, m3enc_b,
        m4enc_a, m4enc_b,
        discharge,
        kdone, kcharge, kkick, kchip,
        flash_ncs,
        fpga_ncs, mosi, sck, miso
    );

    // Clock
    initial begin
        clk = 0;
        forever begin
            #1 clk = ~clk;
        end
    end
    
    task spi_on;
        begin
            fpga_ncs = 0; #2;
        end
    endtask
    
    task spi_off;
        begin
            #2 fpga_ncs = 1; #4;
        end
    endtask
    
    task spi;
        input [7:0] in;
        
        begin
            mosi = in[7]; #8 sck = 1; #8 sck = 0;
            mosi = in[6]; #8 sck = 1; #8 sck = 0;
            mosi = in[5]; #8 sck = 1; #8 sck = 0;
            mosi = in[4]; #8 sck = 1; #8 sck = 0;
            mosi = in[3]; #8 sck = 1; #8 sck = 0;
            mosi = in[2]; #8 sck = 1; #8 sck = 0;
            mosi = in[1]; #8 sck = 1; #8 sck = 0;
            mosi = in[0]; #8 sck = 1; #8 sck = 0;
        end
    endtask
    
    initial begin
    #4.5 spi_on();
        spi(8'h68);
        spi(8'h79);
        spi(8'hbc);
        spi(8'hde);
        spi_off();
        spi_on();
        spi(8'h01);
        spi(8'h0);
        spi(8'h0);
        spi(8'h0);
        spi(8'h0);
        spi_off();
    end

    // Logging
    initial begin
        $dumpvars;
        $dumpall;
        #2000
        $finish;
    end
endmodule
