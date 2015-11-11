# ICARUS VERILOG & GTKWAVE MAKEFILE
# MADE BY WILLIAM GIBB FOR HACDC
# williamgibb@gmail.com
# 
# USE THE FOLLOWING COMMANDS WITH THIS MAKEFILE
#	"make check" - compiles your verilog design - good for checking code
#	"make simulate" - compiles your design+TB & simulates your design
#	"make display" - compiles, simulates and displays waveforms
# 
###############################################################################
#
# CHANGE THESE THREE LINES FOR YOUR DESIGN
#

#TOOL INPUT
#SOURCEDIR = $(shell pwd)
#SRC := $(shell find $(SOURCEDIR) -name '*.v')
SRC = robocup.v
TESTBENCH = robocup_top_tb.v
TBOUTPUT = robocup_top_tb-out.lxt
TOP_MODULE_NAME = RoboCup_Top_tb

###############################################################################
# BE CAREFUL WHEN CHANGING ITEMS BELOW THIS LINE
###############################################################################

#TOOLS
COMPILER = iverilog 
SIMULATOR = vvp
VIEWER = gtkwave

#TOOL OPTIONS
COFLAGS = -s $(TOP_MODULE_NAME) -v -o
SFLAGS = -v
SOUTPUT = 			#SIMULATOR OUTPUT TYPE

#TOOL OUTPUT
COUTPUT = compiler.vvp		#COMPILER OUTPUT

###############################################################################
#MAKE DIRECTIVES

default: $(COUTPUT) display

check: $(TESTBENCH) $(SRC)
	$(COMPILER) -v $(SRC)

simulate: $(COUTPUT)
#	$(SIMULATOR) $(SFLAGS) $(COUTPUT) $(SOUTPUT)
# $(VIEWER) RoboCup_Top_tb-results.vcd
display: $(TBOUTPUT)
	$(VIEWER) $(TBOUTPUT) &
#MAKE DEPENDANCIES
$(TBOUTPUT): $(COUTPUT) $(TESTBENCH)
	$(SIMULATOR) -m $(RoboCup_Top_tb) $(COUTPUT) $(TBOUTPUT)

$(COUTPUT): $(TESTBENCH) *.v
	$(COMPILER) -DICARUS_VERILOG -E -o test.pre *.v && $(COMPILER) -o $(COUTPUT) test.pre
