ifneq ($(words $(CURDIR)),1)
 $(error unsupported: gnu make cannot build in directories containing spaces, build elsewhere: '$(CURDIR)')
endif

# check verilator
export VERILATOR_ROOT=/Users/tony/.local/artifacts/verilator
# If $VERILATOR_ROOT isn't in the environment, we assume it is part of a
# package install, and verilator is in your path. otherwise find the
# binary relative to $VERILATOR_ROOT (such as when inside the git sources).
ifeq ($(VERILATOR_ROOT),)
VERILATOR = verilator
VERILATOR_COVERAGE = verilator_coverage
else
export VERILATOR_ROOT
VERILATOR = $(VERILATOR_ROOT)/bin/verilator
VERILATOR_COVERAGE = $(VERILATOR_ROOT)/bin/verilator_coverage
endif


VERILATOR_FLAGS =
# VERILATOR_FLAGS += -cc --exe
VERILATOR_FLAGS += --binary
VERILATOR_FLAGS += --x-assign 0
VERILATOR_FLAGS += -Wall -Wno-fatal
VERILATOR_FLAGS += -Wno-unused
VERILATOR_FLAGS += --trace --trace-structs
VERILATOR_FLAGS += --assert
# VERILATOR_FLAGS += --coverage
VERILATOR_FLAGS += --build -j
# run verilator in debug mode
#VERILATOR_FLAGS += --debug
# add this trace to get a backtrace in gdb
#VERILATOR_FLAGS += --gdbbt


######################################################################

# GENHTML = genhtml
# Create annotated source
# VERILATOR_COV_FLAGS += --annotate tmp/annotated
# # A single coverage hit is considered good enough
# VERILATOR_COV_FLAGS += --annotate-min 1
# # Create LCOV info
# VERILATOR_COV_FLAGS += --write-info tmp/coverage.info
# # Input file from Verilator
# VERILATOR_COV_FLAGS += tmp/coverage.dat

######################################################################

# ifeq ($(shell which iverilog),)
# $(error did not find icarus verilog in path. make sure all requirements are installed)
# endif
WAVEFORM_VIEWER ?= surfer
# WAVEFORM_VIEWER=gtkwave -6

# export ROOT_DIR := $(shell pwd)
ROOT_DIR := $(dir $(realpath $(lastword $(MAKEFILE_LIST))))

TOP_MODULE = soc_tb
TOP_FILE = $(ROOT_DIR)/tb/$(TOP_MODULE).sv

# TOP_MODULE = soc
# TOP_FILE = $(ROOT_DIR)src/$(TOP_MODULE).sv
WAVEFORM_FILE = $(ROOT_DIR)waves/$(TOP_MODULE).vcd
VERILATOR_INPUT = $(TOP_FILE) -Isrc -Itb --top $(TOP_MODULE) -DBENCH
SURFER_STATE = $(ROOT_DIR)waves/surfer_state.ron

all: vl wa

vl: clean
	@echo
	@echo "-- VERILATE ----------------"
	@$(VERILATOR) --version
	$(VERILATOR) $(VERILATOR_FLAGS) $(VERILATOR_INPUT)

	@echo
	@echo "-- RUN ---------------------"
	@mkdir -p tmp
	@mkdir -p waves
	obj_dir/V$(TOP_MODULE)

	# @echo
	# @echo "-- COVERAGE ----------------"
	# @rm -rf logs/annotated
	# $(VERILATOR_COVERAGE) $(VERILATOR_COV_FLAGS)

	@echo
	@echo "-- DONE --------------------"

wa:
	$(WAVEFORM_VIEWER) $(WAVEFORM_FILE) --state-file $(SURFER_STATE) > $(ROOT_DIR)waves/$(WAVEFORM_VIEWER).log 2>&1 &

schematic:
	netlistsvg boards/synthesized.json -o boards/synthesized.svg

clean:
	$(RM) -R tmp
	$(RM) -R obj_dir
	$(RM) -R .Xil
	$(RM) -R vivado_**
	$(RM) -R vivado.jou
	$(RM) -R vivado.log
