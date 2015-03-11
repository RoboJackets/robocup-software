`include "phase_driver.vh"

module hall_effect_tb.vh

/*Output values to 'test.vcd' file */
initial
 begin
    $dumpfile("test.vcd");
    $dumpvars(0,pwm_tb);
 end






endmodule