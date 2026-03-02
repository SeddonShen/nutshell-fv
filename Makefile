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

src: sim-verilog

fuzzer:
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

# --- Standalone Cache module generation for BMCFuzz ---
CACHE_COVER ?= mux,control
CACHE_DIR    = $(BUILD_DIR)/modules/cache

gen-cache:
	mkdir -p $(CACHE_DIR)
	NOOP_HOME=$(abspath .) mill -i NutShell.runMain top.CacheGenMain \
		-td $(CACHE_DIR) \
		COVER=$(CACHE_COVER)

# --- Verilator EMU for standalone Cache (BMCFuzz) ---
CACHE_EMU_DIR = $(BUILD_DIR)/emu-cache
CACHE_EMU_BIN = $(CACHE_EMU_DIR)/emu-cache
CACHE_TB_CPP  = $(abspath bmctest/emu/cache_tb.cpp)

VLTR_FLAGS  = --cc --exe --build -j
VLTR_FLAGS += -DSYNTHESIS
VLTR_FLAGS += --top-module StandaloneCache
VLTR_FLAGS += --Mdir $(CACHE_EMU_DIR)/obj
VLTR_FLAGS += -o $(abspath $(CACHE_EMU_BIN))
VLTR_FLAGS += --trace
VLTR_FLAGS += -Wno-fatal -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND

ifdef CACHE_COV
VLTR_FLAGS += --coverage
# Use -O0 for coverage builds: -O2 on 4MB+ instrumented Slow.cpp is extremely slow (~200s).
# Coverage collection does not need aggressive optimization.
VLTR_CFLAGS = -O0 -DVM_TRACE=1 -DVM_COVERAGE=1 -DVM_SAVABLE=0
else
VLTR_FLAGS += --savable
VLTR_CFLAGS = -O2 -DVM_TRACE=1 -DVM_SAVABLE=1
endif
VLTR_FLAGS += -CFLAGS "$(VLTR_CFLAGS)"

emu-cache: gen-cache
	mkdir -p $(CACHE_EMU_DIR)
	verilator $(VLTR_FLAGS) \
		$(CACHE_DIR)/StandaloneCache.sv \
		$(wildcard $(CACHE_DIR)/*.v) \
		$(CACHE_TB_CPP)
	@echo "[emu-cache] Binary: $(CACHE_EMU_BIN)"

# --- libFuzzer harness for coverage-guided fuzz (seed as corpus) ---
CACHE_FUZZ_DIR  = $(BUILD_DIR)/emu-cache-fuzz
CACHE_FUZZ_BIN  = $(CACHE_FUZZ_DIR)/emu-cache-fuzz

VLTR_FUZZ_FLAGS  = --cc --exe --build -j
VLTR_FUZZ_FLAGS += -DSYNTHESIS
VLTR_FUZZ_FLAGS += --top-module StandaloneCache
VLTR_FUZZ_FLAGS += --Mdir $(CACHE_FUZZ_DIR)/obj
VLTR_FUZZ_FLAGS += -o $(abspath $(CACHE_FUZZ_BIN))
VLTR_FUZZ_FLAGS += -Wno-fatal -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND
VLTR_FUZZ_FLAGS += -CFLAGS "-DCACHE_FUZZ=1 -fsanitize=fuzzer -g -O2"
VLTR_FUZZ_FLAGS += -LDFLAGS "-fsanitize=fuzzer"

emu-cache-fuzz: gen-cache
	mkdir -p $(CACHE_FUZZ_DIR)
	verilator $(VLTR_FUZZ_FLAGS) \
		$(CACHE_DIR)/StandaloneCache.sv \
		$(wildcard $(CACHE_DIR)/*.v) \
		$(CACHE_TB_CPP)
	@echo "[emu-cache-fuzz] Binary: $(CACHE_FUZZ_BIN)"
	@echo "  Run: $(CACHE_FUZZ_BIN) seed.bin"

.PHONY: verilog emu clean help gen-cache emu-cache emu-cache-fuzz
