# The top level module should define the variables below then include
# this file.  The files listed should be in the same directory as the
# Makefile.  
#
#   variable	description
#   ----------  -------------
#   project	project name (top level module should match this name)
#   top_module  top level module of the project
#   libdir	path to library directory
#   libs	library modules used
#   VFILES	all local .v files
#   xilinx_cores  all local .xco files
#   vendor      vendor of FPGA (xilinx, altera, etc.)
#   family      FPGA device family (spartan3e) 
#   part        FPGA part name (xc4vfx12-10-sf363)
#   flashsize   size of flash for mcs file (16384)
#   optfile     (optional) xst extra opttions file to put in .scr
#   MAP_OPTS    (optional) options to give to map
#   PAR_OPTS    (optional) options to give to par
#   intstyle    (optional) intstyle option to all tools
#
#   files 		description
#   ----------  	------------
#   $(project).ucf	ucf file
#
# Library modules should have a modules.mk in their root directory,
# namely $(libdir)/<libname>/module.mk, that simply adds to the VFILES
# and xilinx_cores variable.
#
# all the .xco files listed in xilinx_cores will be generated with core, with
# the resulting .v and .ngc files placed back in the same directory as
# the .xco file.
#
# TODO: .xco files are device dependant, should use a template based system

project = robocup
vendor = xilinx
family = spartan3e
PART = xc3s100etq144-4
top_module = robocup

VFILES = $(wildcard \.\./src/*.v)

MAP_OPTS = -timing -ol high -detail -pr b -register_duplication -w
PAR_OPTS = -ol high

COREGEN_WORK_DIR = ./coregen-tmp
XFLOW_TMP_DIR = xflow

# ** This is where the path to WebPACK is set **
ISE_DIR = /opt/Xilinx/14.7/ISE_DS
XIL_EXECS_DIR = $(ISE_DIR)/ISE/bin/lin64
export PATH := $(XIL_EXECS_DIR):$(PATH)

libmks = $(patsubst %,$(libdir)/%/module.mk,$(libs)) 
mkfiles = Makefile $(libmks) Makefile
include $(libmks)

corengcs = $(foreach core,$(xilinx_cores),$(core:.xco=.ngc))
local_corengcs = $(foreach ngc,$(corengcs),$(notdir $(ngc)))
VFILES += $(foreach core,$(xilinx_cores),$(core:.xco=.v))
junk += $(local_corengcs)



# ========== .PHONY ==========
.PHONY: default xilinx_cores clean twr etwr


# ========== default ==========
default: $(project).bit $(project).mcs
all: default



# ========== xilinx_cores ==========
xilinx_cores: $(corengcs)



# ========== twr ==========
twr: $(project).twr



# ========== etwr ==========
etwr: $(project)_err.twr



# ========== $(2): $(1) ==========
define cp_template	
$(2): $(1)
	cp $(1) $(2)
endef
$(foreach ngc,$(corengcs),$(eval $(call cp_template,$(ngc),$(notdir $(ngc)))))



# ========== %.ngc %.v ==========
%.ngc %.v: %.xco
	@if [ -d $(COREGEN_WORK_DIR) ]; then \
		rm -rf $(COREGEN_WORK_DIR)/*; \
	else \
		mkdir -p $(COREGEN_WORK_DIR); \
	fi
	cd $(COREGEN_WORK_DIR); \
	coregen -b $$OLDPWD/$<; \
	cd -
	xcodir=`dirname $<`; \
	basename=`basename $< .xco`; \
	@if [ ! -r $(COREGEN_WORK_DIR/$$basename.ngc) ]; then \
		echo "'$@' wasn't created."; \
		exit 1; \
	else \
		cp $(COREGEN_WORK_DIR)/$$basename.v $(COREGEN_WORK_DIR)/$$basename.ngc $$xcodir; \
	fi

date = $(shell date +%F-%H-%M)
junk += $(COREGEN_WORK_DIR)
junk += *.xrpt



# ========== programming_files ==========
programming_files: $(project).bit $(project).mcs
	@mkdir -p $@/$(date)
	@mkdir -p $@/latest
	for x in .bit .mcs .cfi _bd.bmm; do cp $(project)$$x $@/$(date)/$(project)$$x; cp $(project)$$x $@/latest/$(project)$$x; done
	xst -help | head -1 | sed 's/^/#/' | cat - $(project).scr > $@/$(date)/$(project).scr



# ========== $(project).mcs ==========
$(project).mcs: $(project).bit
ifdef flashsize
	promgen -w -s $(flashsize) -p mcs -o $@ -u 0 $^

junk += $(project).mcs $(project).cfi $(project).prm
endif





# ========== $(project).bit ==========
$(project).bit: $(project)_par.ncd
	bitgen $(intstyle) -g DriveDone:yes -g StartupClk:Cclk -w $(project)_par.ncd $(project).bit

junk += $(project).bgn $(project).bit $(project).drc $(project)_bd.bmm



# ========== $(project)_par.ncd ==========
$(project)_par.ncd: $(project).ncd
	@if par $(intstyle) $(PAR_OPTS) -w $(project).ncd $(project)_par.ncd; then \
		:; \
	else \
		$(MAKE) etwr; \
	fi

junk += $(project)_par.ncd $(project)_par.par $(project)_par.pad 
junk += $(project)_par_pad.csv $(project)_par_pad.txt 
junk += $(project)_par.grf $(project)_par.ptwx
junk += $(project)_par.unroutes $(project)_par.xpi



# ========== $(project).ncd ==========
$(project).ncd: $(project).ngd
	@if [ -r $(project)_par.ncd ]; then \
		cp $(project)_par.ncd smartguide.ncd; \
		smartguide="-smartguide smartguide.ncd"; \
	else \
		smartguide=""; \
	fi; \
	map $(intstyle) $(MAP_OPTS) $$smartguide $<

junk += $(project).ncd $(project).pcf $(project).ngm $(project).mrp $(project).map
junk += smartguide.ncd $(project).psr 
junk += $(project)_summary.xml $(project)_usage.xml



# ========== $(project).ngd ==========
$(project).ngd: $(project).ngc $(project).ucf $(project).bmm
	ngdbuild $(intstyle) $(project).ngc -bm $(project).bmm

junk += $(project).ngd $(project).bld



# ========== $(project).ngc ==========
$(project).ngc: $(VFILES) $(local_corengcs) $(project).scr $(project).prj
	xst $(intstyle) -ifn $(project).scr

junk += xlnx_auto* $(top_module).lso $(project).srp 
junk += netlist.lst xst $(project).ngc



# ========== $(project).prj ==========
$(project).prj: $(VFILES) $(mkfiles)
	for src in $(VFILES); do echo "verilog work $$src" >> $(project).tmpprj; done
	sort -u $(project).tmpprj > $(project).prj
	@rm -f $(project).tmpprj

junk += $(project).prj



# ========== $(project).scr ==========
optfile += $(wildcard $(project).opt)
top_module ?= $(project)
$(project).scr: $(optfile) $(mkfiles) ./xilinx.opt
	@echo "run" > $@
	@echo "-p $(PART)" >> $@
	@echo "-top $(top_module)" >> $@
	@echo "-ifn $(project).prj" >> $@
	@echo "-ofn $(project).ngc" >> $@
	@cat ./xilinx.opt $(optfile) >> $@

junk += $(project).scr



# ========== $(project).post_map.twr ==========
$(project).post_map.twr: $(project).ncd
	trce -e 10 $< $(project).pcf -o $@

junk += $(project).post_map.twr $(project).post_map.twx smartpreview.twr



# ========== $(project).twr ==========
$(project).twr: $(project)_par.ncd
	trce $< $(project).pcf -o $(project).twr

junk += $(project).twr $(project).twx smartpreview.twr



# ========== $(project)_err.twr ==========
$(project)_err.twr: $(project)_par.ncd
	trce -e 10 $< $(project).pcf -o $(project)_err.twr

junk += $(project)_err.twr $(project)_err.twx


ifeq ("x","x")	# block comment rigging...
xflow: $(project).ucf
	@if [ -d $(XFLOW_TMP_DIR) ]; then \
		rm -rf $(XFLOW_TMP_DIR); \
	else \
		mkdir -p $(XFLOW_TMP_DIR); \
	fi
	cd $(XFLOW_TMP_DIR)
	xflow -wd $(XFLOW_TMP_DIR) -p $(PART) -synth xst_verilog -implement fast_runtime -config bitgen $(project).ucf

junk += $(XFLOW_TMP_DIR);
endif



# ========== clean ==========
clean::
	@rm -rf $(junk)
	@if [ -d _xmsgs ]; then	\
		rm -rf _xmsgs; 		\
	fi
	

