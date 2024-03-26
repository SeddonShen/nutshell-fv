#include <cstdlib>
#include <cassert>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include "difftest.h"

//#include "VSimTop__Dpi.h"
#include "common.h"
#include "VNutShellSimTop.h"
#if VM_TRACE
#include <verilated_vcd_c.h>	// Trace file format header
#endif


class Emulator {
  const char *image;
  std::shared_ptr<VNutShellSimTop> dut_ptr;
#if VM_TRACE
  VerilatedVcdC* tfp;
#endif

  // emu control variable
  uint32_t seed;
  uint64_t max_cycles, cycles;
  uint64_t log_begin, log_end, log_level;

  std::vector<const char *> parse_args(int argc, const char *argv[]);

  static const struct option long_options[];
  static void print_help(const char *file);

  void read_emu_regs(rtlreg_t *r) {
#define macro(x) r[x] = dut_ptr->io_difftest_r_##x
    macro(0); macro(1); macro(2); macro(3); macro(4); macro(5); macro(6); macro(7);
    macro(8); macro(9); macro(10); macro(11); macro(12); macro(13); macro(14); macro(15);
    macro(16); macro(17); macro(18); macro(19); macro(20); macro(21); macro(22); macro(23);
    macro(24); macro(25); macro(26); macro(27); macro(28); macro(29); macro(30); macro(31);
    r[DIFFTEST_THIS_PC] = dut_ptr->io_difftest_thisPC;
#ifndef __RV32__
    r[DIFFTEST_MSTATUS] = dut_ptr->io_difftest_mstatus;
    r[DIFFTEST_SSTATUS] = dut_ptr->io_difftest_sstatus;
    r[DIFFTEST_MEPC   ] = dut_ptr->io_difftest_mepc;
    r[DIFFTEST_SEPC   ] = dut_ptr->io_difftest_sepc;
    r[DIFFTEST_MCAUSE ] = dut_ptr->io_difftest_mcause;
    r[DIFFTEST_SCAUSE ] = dut_ptr->io_difftest_scause;
#endif
  }

  public:
  // argv decay to the secondary pointer
  Emulator(int argc, const char *argv[]):
    image(nullptr),
    dut_ptr(new std::remove_reference<decltype(*dut_ptr)>::type),
    seed(0), max_cycles(-1), cycles(0),
    log_begin(0), log_end(-1), log_level(LOG_ALL)
  {
    // init emu
    auto args = parse_args(argc, argv);

    // srand
    srand(seed);
    srand48(seed);
    Verilated::randReset(2);

    // set log time range and log level
    dut_ptr->io_logCtrl_log_begin = log_begin;
    dut_ptr->io_logCtrl_log_end = log_end;
    dut_ptr->io_logCtrl_log_level = log_level;

    // init ram
    extern void init_ram(const char *img);
    init_ram(image);

    // init device
    extern void init_device(void);
    init_device();
    extern void print_ram_data(void);
    print_ram_data();
    // init core
    reset_ncycles(10);
  }

  void reset_ncycles(size_t cycles) {
    for(int i = 0; i < cycles; i++) {
      dut_ptr->reset = 1;
      dut_ptr->clock = 0;
      dut_ptr->eval();
      dut_ptr->clock = 1;
      dut_ptr->eval();
      dut_ptr->reset = 0;
    }
  }

  void single_cycle() {
    dut_ptr->clock = 0;
    dut_ptr->eval();

    dut_ptr->clock = 1;
    dut_ptr->eval();

#if VM_TRACE
    tfp->dump(cycles);
#endif

    cycles ++;

  }

  void execute_cycles(uint64_t n) {
    extern bool is_finish();
    extern void poll_event(void);
    extern uint32_t uptime(void);
    extern void set_abort(void);
    uint32_t lasttime = 0;
    uint64_t lastcommit = n;
    int hascommit = 0;
    const int stuck_limit = 2000;

#if VM_TRACE
    Verilated::traceEverOn(true);	// Verilator must compute traced signals
    VL_PRINTF("Enabling waves...\n");
    tfp = new VerilatedVcdC;
    dut_ptr->trace(tfp, 99);	// Trace 99 levels of hierarchy
    tfp->open("vlt_dump.vcd");	// Open the dump file
#endif

    while (!is_finish() && n > 0) {
      extern paddr_t read_instr(paddr_t rIdx);
      if (dut_ptr->io_symmemIMemIF_fetchEnable)
      {
        dut_ptr->io_symmemIMemIF_instruction = read_instr(dut_ptr->io_symmemIMemIF_address);
        dut_ptr->io_symmemIMemIF_instructionReady = 1;
        printf("cycles:%d, fetchEnable: %08x %08x\n", cycles, dut_ptr->io_symmemIMemIF_fetchEnable, dut_ptr->io_symmemIMemIF_instructionReady);
      }

      if(dut_ptr->io_symmemDMemIF_enable) {
        if(dut_ptr->io_symmemDMemIF_readWrite) {
          extern void write_data(paddr_t wIdx, paddr_t wdata);
          dut_ptr->io_symmemDMemIF_dataReady = 1;
          write_data(dut_ptr->io_symmemDMemIF_address, dut_ptr->io_symmemDMemIF_writeData);
          // printf io_symmemDMemIF_writeData
          printf("[DMEM_EMU_WRITE] addr: %08x, wdata: %08x, r/w: %d, enable: %d, wrStobe: %08x\n", 
            dut_ptr->io_symmemDMemIF_address, 
            dut_ptr->io_symmemDMemIF_writeData, 
            dut_ptr->io_symmemDMemIF_readWrite, 
            dut_ptr->io_symmemDMemIF_enable, 
            dut_ptr->io_symmemDMemIF_wrStrobe
          );
        }
        else {
          extern paddr_t read_data(paddr_t rIdx);
          dut_ptr->io_symmemDMemIF_dataReady = 1;
          dut_ptr->io_symmemDMemIF_readData = read_data(dut_ptr->io_symmemDMemIF_address);
          // printf io_symmemDMemIF_readData
          printf("[DMEM_EMU_READ] addr: %08x, rdata: %08x, r/w: %d, enable: %d, wrStobe: %08x\n", 
            dut_ptr->io_symmemDMemIF_address, 
            dut_ptr->io_symmemDMemIF_readData, 
            dut_ptr->io_symmemDMemIF_readWrite, 
            dut_ptr->io_symmemDMemIF_enable, 
            dut_ptr->io_symmemDMemIF_wrStrobe
          );
        }
      }

      // printf dut_ptr->io_symmemDMemIF
      printf("[DMEM] addr: %08x, wdata: %08x, r/w: %d, enable: %d, wrStobe: %08x\n", 
        dut_ptr->io_symmemDMemIF_address, 
        dut_ptr->io_symmemDMemIF_writeData, 
        dut_ptr->io_symmemDMemIF_readWrite, 
        dut_ptr->io_symmemDMemIF_enable, 
        dut_ptr->io_symmemDMemIF_wrStrobe
      );
      single_cycle();
      dut_ptr->io_symmemIMemIF_instructionReady = 0;
      dut_ptr->io_symmemDMemIF_dataReady = 0;
      // if(dut_ptr->io_symmemIMemIF_fetchEnable) {
      //   dut_ptr->io_symmemIMemIF_instruction = read_instr(dut_ptr->io_symmemIMemIF_address);
      // }
      // printf dut_ptr->io_symmemIMemIF_fetchEnable
      n --;

      if (lastcommit - n > stuck_limit && hascommit) {
        eprintf("No instruction commits for %d cycles, maybe get stuck\n"
            "(please also check whether a fence.i instruction requires more than %d cycles to flush the icache)\n",
            stuck_limit, stuck_limit);
#if VM_TRACE
        tfp->close();
#endif
        set_abort();
      }
      // printf("TestSSDIO: %08x %08x\n", dut_ptr->io_symmemIMemIF_fetchEnable, dut_ptr->io_symmemIMemIF_instructionReady);
      // dut_ptr->io_symmemIMemIF_instructionReady = 1;
      // printf("TestSSDIO: %08x %08x\n", dut_ptr->io_symmemIMemIF_fetchEnable, dut_ptr->io_symmemIMemIF_instructionReady);
      // dut_ptr->io_symmemIMemIF_instructionReady = 0;
      if(dut_ptr->io_difftest_commit) {
        printf("commit:%d, cycles: %d, PC:%08x, Inst:%08x Rd:%08x RdData:%08x\n", dut_ptr->io_difftest_commit, cycles, dut_ptr->io_difftest_thisPC, dut_ptr->io_difftest_thisINST, dut_ptr->io_difftest_rvfi_rd_addr, dut_ptr->io_difftest_rvfi_rd_wdata);

        // printf("commit:%d, cycles: %d, PC:%08x, Inst:%08x Reg1:%08x\n", dut_ptr->io_difftest_commit, cycles, dut_ptr->io_difftest_thisPC, dut_ptr->io_difftest_thisINST, dut_ptr->io_difftest_r_1);
      }
      // printf("commit:%d, cycles: %d, PC:%08x, Inst:%08x Reg2:%08x\n", dut_ptr->io_difftest_commit, cycles, dut_ptr->io_difftest_thisPC, dut_ptr->io_difftest_thisINST, dut_ptr->io_difftest_r_2);
      if (!hascommit && (uint32_t)dut_ptr->io_difftest_thisPC == 0x80000000) {
        hascommit = 1;
        // extern void init_difftest(rtlreg_t *reg);
        rtlreg_t reg[DIFFTEST_NR_REG];
        read_emu_regs(reg);
        // init_difftest(reg);
      }

      // difftest
      if (dut_ptr->io_difftest_commit && hascommit) {
        rtlreg_t reg[DIFFTEST_NR_REG];
        read_emu_regs(reg);

//         extern int difftest_step(rtlreg_t *reg_scala, uint32_t this_inst,
//           int isMMIO, int isRVC, int isRVC2, uint64_t intrNO, int priviledgeMode, int isMultiCommit);
//         if (dut_ptr->io_difftestCtrl_enable) {
//           if (difftest_step(reg, dut_ptr->io_difftest_thisINST,
//               dut_ptr->io_difftest_isMMIO, dut_ptr->io_difftest_isRVC, dut_ptr->io_difftest_isRVC2,
//               dut_ptr->io_difftest_intrNO, dut_ptr->io_difftest_priviledgeMode, 
//               dut_ptr->io_difftest_isMultiCommit)) {
// #if VM_TRACE
//             tfp->close();
// #endif
//             set_abort();
//           }
//         }
        lastcommit = n;
      }

      uint32_t t = uptime();
      if (t - lasttime > 100) {
        poll_event();
        lasttime = t;
      }
    }
    extern void print_ram_data(void);
    print_ram_data();
  }

  void cache_test(uint64_t n) {
    while (n > 0) {
      single_cycle();
      n --;
    }
  }

  void execute() {
//#define CACHE_TEST

#ifdef CACHE_TEST
    eprintf(ANSI_COLOR_MAGENTA "This is random test for cache.\n" ANSI_COLOR_RESET);
    cache_test(max_cycles);
#else
    execute_cycles(max_cycles);
#endif
  }
  uint64_t get_cycles() const { return cycles; }
  uint64_t get_max_cycles() const { return max_cycles; }
};
