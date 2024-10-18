TOP = TopMain
FPGATOP = NutShellFPGATop
BUILD_DIR = $(abspath ./build)
RTL_DIR=$(BUILD_DIR)/rtl
TOP_V = $(RTL_DIR)/$(TOP).v
SCALA_FILE = $(shell find ./src/main/scala -name '*.scala')
TEST_FILE = $(shell find ./src/test/scala -name '*.scala')           \
	    $(shell find ./difftest/src/main/scala -name '*.scala')

USE_READY_TO_RUN_NEMU = true

SIMTOP = top.TopMain
IMAGE ?= ready-to-run/linux.bin

DATAWIDTH ?= 64
BOARD ?= sim  # sim  pynq  axu3cg
CORE  ?= inorder  # inorder  ooo  embedded

.DEFAULT_GOAL = verilog

help:
	mill -i NutShell.runMain top.$(TOP) --help BOARD=$(BOARD) CORE=$(CORE)

MILL_ARGS  = -td $(@D) --output-file $(@F)
MILL_ARGS += BOARD=$(BOARD) CORE=$(CORE)
MILL_ARGS += --gen-mem-verilog full

# Coverage support
ifneq ($(FIRRTL_COVER),)
MILL_ARGS += COVER=$(FIRRTL_COVER)
endif

$(TOP_V): $(SCALA_FILE)
	mkdir -p $(@D)
	mill -i NutShell.runMain top.$(TOP) $(MILL_ARGS)     \
		--repl-seq-mem -c:$(FPGATOP):-o:$(@D)/$(@F).conf \
		--infer-rw $(FPGATOP)
	@sed -i -e 's/_\(aw\|ar\|w\|r\|b\)_\(\|bits_\)/_\1/g' $@
	@git log -n 1 >> .__head__
	@git diff >> .__diff__
	@sed -i 's/^/\/\// ' .__head__
	@sed -i 's/^/\/\//' .__diff__
	@cat .__head__ .__diff__ $@ > .__out__
	@mv .__out__ $@
	@rm .__head__ .__diff__

deploy: build/top.zip


build/top.zip: $(TOP_V)
	@zip -r $@ $< $<.conf build/*.anno.json

.PHONY: deploy build/top.zip

verilog: $(TOP_V)

SIM_TOP = SimTop
SIM_TOP_V = $(RTL_DIR)/$(SIM_TOP).sv
$(SIM_TOP_V): $(SCALA_FILE) $(TEST_FILE)
	mkdir -p $(@D)
	mill -i NutShell.test.runMain $(SIMTOP) $(MILL_ARGS) \
		--repl-seq-mem -c:$(SIM_TOP):-o:$(@D)/$(@F).conf \
		--infer-rw $(SIM_TOP)
	@sed -i -e 's/$$fatal/xs_assert(`__LINE__)/g' $(SIM_TOP_V)

sim-verilog: $(SIM_TOP_V)

emu: sim-verilog
	@$(MAKE) -C ./difftest emu RTL_SUFFIX=sv WITH_CHISELDB=0 WITH_CONSTANTIN=0

xfuzz:
	@$(MAKE) -C ./ccover build

init:
	git submodule update --init
	$(MAKE) -C ./difftest init

clean:
	rm -rf $(BUILD_DIR)

bsp:
	mill -i mill.bsp.BSP/install

idea:
	mill -i mill.scalalib.GenIdea/idea

sds:
	$(MAKE) emu EMU_TRACE=1 -j16 EMU_THREADS=4

.PHONY: verilog emu clean help
