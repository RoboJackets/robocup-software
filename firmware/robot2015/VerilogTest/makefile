
test: pwm_tb.v PWM.v  phase_driver.vh makefile
	iverilog -E -o test.pre *.v && iverilog -o test.vvp test.pre

run: test
	vvp test.vvp && gtkwave test.vcd

clean: 
	rm *.vvp *.pre *.vcd
