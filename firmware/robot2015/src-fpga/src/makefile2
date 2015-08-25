#Using icarus to compile and simulate the testbench filefile
test: *.v *.vh
	iverilog -E -o test.pre robocup_tb.v && iverilog -o test.vvp test.pre

run: test
	vvp test.vvp && gtkwave test.vcd


clean: 
	rm *.vvp *.pre *.vcd
