/**************************************************************************************
 * cache_tb.cpp — Verilator testbench for standalone NutShell Cache
 *
 * Part of the BMCFuzz hybrid testing framework.
 * Drives StandaloneCache with seed-derived SimpleBus transactions, collects
 * coverage, and supports snapshot save/restore for BMC-guided fuzzing.
 *
 * Seed format:
 *   A flat binary buffer consumed 46 bytes per clock cycle.  Each 46-byte
 *   block (struct SeedCycle) is memory-mapped to the DUT input signals.
 *   When the seed is shorter than (cycles * 46), it wraps around.
 *
 * Build:  make emu-cache           (plain)
 *         make emu-cache CACHE_COV=1  (with Verilator toggle/line coverage)
 *         make emu-cache-fuzz       (libFuzzer harness for coverage-guided fuzz)
 *
 * Run:
 *   ./build/emu-cache/emu-cache --seed seed.bin --cycles 5000
 *   ./build/emu-cache/emu-cache --seed seed.bin --snap-save snap.bin
 *   ./build/emu-cache/emu-cache --seed seed.bin --snap-load snap.bin --vcd wave.vcd
 *
 * Fuzz (libFuzzer, seed.bin as corpus):
 *   ./build/emu-cache-fuzz/emu-cache-fuzz seed.bin
 **************************************************************************************/

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "VStandaloneCache.h"
#include "verilated.h"
#if VM_SAVABLE
#include "verilated_save.h"
#endif

#if VM_TRACE
#include "verilated_vcd_c.h"
#endif

#if VM_COVERAGE
#include "verilated_cov.h"
#endif

/* ═══════════════════════════════════════════════════════════════════════
 * Seed layout — 46 bytes per cycle, packed, no padding.
 *
 * The fuzzer generates a raw byte stream; this testbench interprets each
 * consecutive 46-byte chunk as one clock cycle's worth of DUT inputs.
 * ═══════════════════════════════════════════════════════════════════════ */

#pragma pack(push, 1)
struct SeedCycle {
    /* CPU-side request  (io_in — SimpleBusUC, userBits=16) */
    uint32_t addr;          /*  4B  io_in_req_bits_addr  [31:0] */
    uint64_t wdata;         /*  8B  io_in_req_bits_wdata [63:0] */
    uint16_t user;          /*  2B  io_in_req_bits_user  [15:0] */
    uint8_t  wmask;         /*  1B  io_in_req_bits_wmask [7:0]  */
    uint8_t  cmd;           /*  1B  io_in_req_bits_cmd   [3:0]  */
    uint8_t  size_val;      /*  1B  io_in_req_bits_size  [2:0]  */

    /* Packed control bits */
    uint8_t  ctrl;          /*  1B  [0] req_valid
                                    [1] resp_ready
                                    [3:2] flush                  */

    /* Environment response control */
    uint8_t  env;           /*  1B  [0] mem_req_ready
                                    [1] mem_resp_valid
                                    [2] coh_req_ready
                                    [3] coh_resp_valid
                                    [4] mmio_req_ready
                                    [5] mmio_resp_valid          */

    /* Memory-side response  (io_out_mem) */
    uint64_t mem_rdata;     /*  8B  io_out_mem_resp_bits_rdata   */
    uint8_t  mem_cmd;       /*  1B  io_out_mem_resp_bits_cmd     */

    /* Coherence response    (io_out_coh) */
    uint64_t coh_rdata;     /*  8B  io_out_coh_resp_bits_rdata   */
    uint8_t  coh_cmd;       /*  1B  io_out_coh_resp_bits_cmd     */

    /* MMIO response         (io_mmio) */
    uint64_t mmio_rdata;    /*  8B  io_mmio_resp_bits_rdata      */
    uint8_t  mmio_cmd;      /*  1B  io_mmio_resp_bits_cmd        */
};
#pragma pack(pop)

static constexpr size_t BYTES_PER_CYCLE = sizeof(SeedCycle);  /* 46 */

/* ═══════════════════════════════════════════════════════════════════════
 * Command-line options
 * ═══════════════════════════════════════════════════════════════════════ */

struct Options {
    std::string seed_file;
    uint64_t    max_cycles   = 10000;
    uint64_t    reset_cycles = 10;
    std::string snap_save;
    std::string snap_load;
    std::string cov_out      = "coverage.dat";
    std::string vcd_file;
};

static void usage(const char *prog) {
    fprintf(stderr,
        "NutShell Cache EMU — seed-driven Verilator testbench for BMCFuzz\n\n"
        "Usage: %s --seed <file> [options]\n\n"
        "  --seed <file>         Binary seed file (required)\n"
        "  --cycles <N>          Simulation cycles after reset  [10000]\n"
        "  --reset-cycles <N>    Reset duration in cycles       [10]\n"
        "  --snap-save <file>    Save snapshot after simulation\n"
        "  --snap-load <file>    Restore snapshot before simulation\n"
        "  --cov-out <file>      Coverage output path           [coverage.dat]\n"
        "  --vcd <file>          VCD waveform dump\n"
        "  -h, --help            Show this message\n",
        prog);
}

static Options parse_opts(int argc, char **argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string a(argv[i]);
        auto next = [&]() -> const char * {
            if (i + 1 >= argc) { usage(argv[0]); std::exit(1); }
            return argv[++i];
        };
        if      (a == "--seed")         o.seed_file    = next();
        else if (a == "--cycles")       o.max_cycles   = std::stoull(next());
        else if (a == "--reset-cycles") o.reset_cycles = std::stoull(next());
        else if (a == "--snap-save")    o.snap_save    = next();
        else if (a == "--snap-load")    o.snap_load    = next();
        else if (a == "--cov-out")      o.cov_out      = next();
        else if (a == "--vcd")          o.vcd_file     = next();
        else if (a == "-h" || a == "--help") { usage(argv[0]); std::exit(0); }
    }
    if (o.seed_file.empty()) { usage(argv[0]); std::exit(1); }
    return o;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Helpers
 * ═══════════════════════════════════════════════════════════════════════ */

static std::vector<uint8_t> read_file(const std::string &path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        fprintf(stderr, "ERROR: cannot open %s\n", path.c_str());
        std::exit(1);
    }
    auto sz = static_cast<size_t>(f.tellg());
    f.seekg(0);
    std::vector<uint8_t> buf(sz);
    f.read(reinterpret_cast<char *>(buf.data()), static_cast<std::streamsize>(sz));
    return buf;
}

/*
 * Extract one cycle's input from the seed buffer.
 * Wraps around when (cycle * 46) exceeds seed length.
 */
static SeedCycle decode_cycle(const uint8_t *data, size_t len, uint64_t idx) {
    SeedCycle sc{};
    if (len == 0) return sc;
    size_t off = (idx * BYTES_PER_CYCLE) % len;
    size_t n   = std::min(BYTES_PER_CYCLE, len - off);
    std::memcpy(&sc, data + off, n);
    return sc;
}

/* Drive all DUT input ports from one SeedCycle. */
static void drive(VStandaloneCache *d, const SeedCycle &s) {
    /* CPU-side request */
    d->io_in_req_valid          = (s.ctrl >> 0) & 1;
    d->io_in_req_bits_addr      = s.addr;
    d->io_in_req_bits_size      = s.size_val & 0x7;
    d->io_in_req_bits_cmd       = s.cmd & 0xF;
    d->io_in_req_bits_wmask     = s.wmask;
    d->io_in_req_bits_wdata     = s.wdata;
    d->io_in_req_bits_user      = s.user;

    /* CPU-side response handshake */
    d->io_in_resp_ready         = (s.ctrl >> 1) & 1;

    /* Pipeline flush */
    d->io_flush                 = (s.ctrl >> 2) & 0x3;

    /* Memory-side environment */
    d->io_out_mem_req_ready       = (s.env >> 0) & 1;
    d->io_out_mem_resp_valid      = (s.env >> 1) & 1;
    d->io_out_mem_resp_bits_cmd   = s.mem_cmd & 0xF;
    d->io_out_mem_resp_bits_rdata = s.mem_rdata;

    /* Coherence environment */
    d->io_out_coh_req_ready       = (s.env >> 2) & 1;
    d->io_out_coh_resp_valid      = (s.env >> 3) & 1;
    d->io_out_coh_resp_bits_cmd   = s.coh_cmd & 0xF;
    d->io_out_coh_resp_bits_rdata = s.coh_rdata;

    /* MMIO environment */
    d->io_mmio_req_ready          = (s.env >> 4) & 1;
    d->io_mmio_resp_valid         = (s.env >> 5) & 1;
    d->io_mmio_resp_bits_cmd      = s.mmio_cmd & 0xF;
    d->io_mmio_resp_bits_rdata    = s.mmio_rdata;
}

/* VCD trace pointer (global to avoid #ifdef clutter in tick). */
#if VM_TRACE
static VerilatedVcdC *g_tfp = nullptr;
#endif

/* Advance one full clock cycle (negedge → posedge). */
static void tick(VStandaloneCache *d, uint64_t &t) {
    d->clock = 0;
    d->eval();
#if VM_TRACE
    if (g_tfp) g_tfp->dump(t);
#endif
    ++t;

    d->clock = 1;
    d->eval();
#if VM_TRACE
    if (g_tfp) g_tfp->dump(t);
#endif
    ++t;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Core simulation: reset + seed-driven run.
 * start_cycle: when > 0, skip reset and continue from this cycle (snapshot restore).
 * Returns (cycle, half_ticks) via out params; returns cycle count.
 * ═══════════════════════════════════════════════════════════════════════ */
static uint64_t run_simulation(
    VStandaloneCache *dut,
    VerilatedContext *ctx,
    const uint8_t *seed_data,
    size_t seed_len,
    uint64_t max_cycles,
    uint64_t reset_cycles,
    uint64_t start_cycle,
    uint64_t *out_half_ticks)
{
    uint64_t half_ticks = start_cycle * 2;  /* 2 ticks per cycle */
    uint64_t cycle      = start_cycle;

    if (start_cycle == 0) {
        dut->reset = 1;
        for (uint64_t i = 0; i < reset_cycles; ++i)
            tick(dut, half_ticks);
        dut->reset = 0;
    }

    uint64_t seed_idx = start_cycle;
    for (; cycle < max_cycles && !ctx->gotFinish(); ++cycle) {
        SeedCycle sc = decode_cycle(seed_data, seed_len, seed_idx++);
        drive(dut, sc);
        tick(dut, half_ticks);
    }
    if (out_half_ticks) *out_half_ticks = half_ticks;
    return cycle;
}

/* ═══════════════════════════════════════════════════════════════════════
 * libFuzzer harness — CACHE_FUZZ build only
 * ═══════════════════════════════════════════════════════════════════════ */
#ifdef CACHE_FUZZ
extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size) {
    static VerilatedContext ctx;
    static VStandaloneCache *dut = nullptr;

    if (!dut) {
        ctx.commandArgs(0, static_cast<char**>(nullptr));
        dut = new VStandaloneCache(&ctx);
    }

    /* Fuzz run: fixed 2000 cycles per input for speed. Seed wraps if short. */
    constexpr uint64_t FUZZ_CYCLES = 2000;
    constexpr uint64_t RESET_CYCLES = 10;

    run_simulation(dut, &ctx, Data, Size, FUZZ_CYCLES, RESET_CYCLES, 0, nullptr);
    return 0;
}

extern "C" int LLVMFuzzerInitialize(int *argc, char ***argv) {
    (void)argc;
    (void)argv;
    Verilated::randReset(42);  /* UR-03: reproducible seed */
    return 0;
}
#else

/* ═══════════════════════════════════════════════════════════════════════
 * Main simulation loop (non-fuzz)
 *
 *   1. (optional) restore snapshot → skip reset
 *   2. reset phase: hold reset for N cycles
 *   3. seed-driven simulation: decode + drive + tick for max_cycles
 *   4. (optional) save snapshot
 *   5. write coverage
 * ═══════════════════════════════════════════════════════════════════════ */

int main(int argc, char **argv) {
    auto ctx = std::make_unique<VerilatedContext>();
    ctx->commandArgs(argc, argv);
    Options opts = parse_opts(argc, argv);

    auto dut  = std::make_unique<VStandaloneCache>(ctx.get());
    auto seed = read_file(opts.seed_file);

    uint64_t half_ticks = 0;   /* VCD time (two per cycle) */
    uint64_t cycle      = 0;

    /* ── VCD setup ──────────────────────────────────────────────── */
#if VM_TRACE
    if (!opts.vcd_file.empty()) {
        ctx->traceEverOn(true);
        g_tfp = new VerilatedVcdC;
        dut->trace(g_tfp, 99);
        g_tfp->open(opts.vcd_file.c_str());
    }
#endif

    /* ── Snapshot restore ───────────────────────────────────────── */
#if VM_SAVABLE
    if (!opts.snap_load.empty()) {
        VerilatedRestore rs;
        rs.open(opts.snap_load.c_str());
        rs >> half_ticks;
        rs >> *dut;
        rs.close();
        cycle = half_ticks / 2;
        fprintf(stderr, "[snap] restored from %s (cycle %lu)\n",
                opts.snap_load.c_str(), static_cast<unsigned long>(cycle));
    }
#else
    if (!opts.snap_load.empty()) {
        fprintf(stderr, "ERROR: --snap-load not supported (build without CACHE_COV for savable)\n");
        std::exit(1);
    }
#endif

    /* ── Seed-driven simulation ─────────────────────────────────── */
    cycle = run_simulation(
        dut.get(), ctx.get(),
        seed.data(), seed.size(),
        opts.max_cycles, opts.reset_cycles,
        cycle,  /* start_cycle: 0 or restored */
        &half_ticks);

    /* ── Snapshot save ──────────────────────────────────────────── */
#if VM_SAVABLE
    if (!opts.snap_save.empty()) {
        VerilatedSave os;
        os.open(opts.snap_save.c_str());
        os << half_ticks;
        os << *dut;
        os.close();
        fprintf(stderr, "[snap] saved to %s (cycle %lu)\n",
                opts.snap_save.c_str(), static_cast<unsigned long>(cycle));
    }
#else
    if (!opts.snap_save.empty()) {
        fprintf(stderr, "ERROR: --snap-save not supported (build without CACHE_COV for savable)\n");
        std::exit(1);
    }
#endif

    /* ── Cleanup & coverage ─────────────────────────────────────── */
#if VM_TRACE
    if (g_tfp) { g_tfp->close(); delete g_tfp; g_tfp = nullptr; }
#endif

#if VM_COVERAGE
    VerilatedCov::write(opts.cov_out.c_str());
    fprintf(stderr, "[cov] written to %s\n", opts.cov_out.c_str());
#endif

    fprintf(stderr, "[sim] %lu cycles completed\n",
            static_cast<unsigned long>(cycle));
    dut->final();
    return 0;
}
#endif /* CACHE_FUZZ */
