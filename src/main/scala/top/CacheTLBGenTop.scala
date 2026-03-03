/**************************************************************************************
* Copyright (c) 2020 Institute of Computing Technology, CAS
* Copyright (c) 2020 University of Chinese Academy of Sciences
*
* NutShell is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*             http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND, EITHER
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT, MERCHANTABILITY OR
* FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package top

import chisel3._
import chisel3.stage._
import chisel3.util.experimental.BoringUtils
import nutcore._
import bus.simplebus._
import xfuzz.CoverPoint
import firrtl.transforms.NoCircuitDedupAnnotation

/**
 * Standalone wrapper combining EmbeddedTLB (DTLB) + Crossbar + DCache.
 * Used for deeper BMCFuzz/XFuzz coverage testing that exercises TLB PTW
 * state machine alongside the DCache 3-stage pipeline.
 *
 * Architecture:
 *   CPU(vaddr) ──► EmbeddedTLB ──► Crossbar 2:1 ──► DCache ──► out / mmio
 *                        │ PTW req (via PTERequestFilter) ──────────────┘ ↑
 *                        └─────────────────────────────────────────────── ┘
 *
 * BoringUtils design notes:
 *   TLBConfig.userBits = 0 (matches EmbeddedTLB.io.mem which is SimpleBusUC() with
 *   userBits=0).  This ensures all three Crossbar inputs share the same userBits=0,
 *   which is required by SimpleBusCrossbarNto1.
 *
 * Sources provided here (consumed as sinks inside EmbeddedTLB / EmbeddedTLBExec):
 *   CSRSATP, MOUFlushTLB, ISAMO, scInflight, lr, lr_addr
 *
 * Dummy sinks added here (satisfy sources from EmbeddedTLB / CacheStage3 that have
 * no consumer in standalone mode):
 *   DTLBFINISH, DTLBPF, DTLBAF, vmEnable, dtlb_paddr, lsuMMIO
 */
class StandaloneCacheWithTLB extends Module
    with HasNutCoreParameter with HasNutCoreConst {

  implicit val cacheConfig: CacheConfig = CacheConfig(
    ro = false, name = "dcache",
    totalSize = 8192, ways = 4, lineSize = 256
  )
  // userBits = 0 (default): keeps all SimpleBus ports compatible with
  // EmbeddedTLB.io.mem (always userBits=0) and SimpleBusCrossbarNto1.
  implicit val tlbConfig: TLBConfig = TLBConfig(name = "dtlb", totalEntry = 64)

  val io = IO(new Bundle {
    // CPU virtual-address input (userBits=0 matching TLBConfig)
    val in         = Flipped(new SimpleBusUC(addrBits = VAddrBits))

    // CSR/MMU: privilege mode / sum / mxr → TLB; loadPF / storePF / af ← TLB
    val csrMMU     = new MMUIO

    // BoringUtils sources — drive TLB sinks from fuzzer-controlled IO
    val satp       = Input(UInt(XLEN.W))
    val flushTLB   = Input(Bool())
    val isAMO      = Input(Bool())
    val scInflight = Input(Bool())
    val lr         = Input(Bool())
    val lrAddr     = Input(UInt(AddrBits.W))

    // Cache pipeline flush
    val flush      = Input(UInt(2.W))

    // Memory-side ports (driven by testbench environment model)
    val out        = new SimpleBusC
    val mmio       = new SimpleBusUC
  })

  // ─── BoringUtils sources (EmbeddedTLB / EmbeddedTLBExec will sink these) ────
  BoringUtils.addSource(io.satp,       "CSRSATP")
  BoringUtils.addSource(io.flushTLB,   "MOUFlushTLB")
  BoringUtils.addSource(io.isAMO,      "ISAMO")
  BoringUtils.addSource(io.scInflight, "scInflight")
  BoringUtils.addSource(io.lr,         "lr")
  BoringUtils.addSource(io.lrAddr,     "lr_addr")

  // ─── Crossbar 2:1: in(0) = translated CPU req, in(1) = PTW mem req ──────────
  val xbar = Module(new SimpleBusCrossbarNto1(2))

  // ─── EmbeddedTLB with built-in PTERequestFilter ──────────────────────────────
  // EmbeddedTLB.apply wires:
  //   tlb.io.in  <> io.in            (CPU virtual-address input)
  //   filter.io.out <> xbar.io.in(1) (PTW memory requests → crossbar port 1)
  val dtlb = EmbeddedTLB(
    in     = io.in,
    mem    = xbar.io.in(1),
    flush  = false.B,
    csrMMU = io.csrMMU,
    enable = true
  )
  xbar.io.in(0) <> dtlb.io.out

  // ─── DCache ──────────────────────────────────────────────────────────────────
  val cache = Module(new Cache)
  cache.io.in    <> xbar.io.out
  cache.io.flush := io.flush
  io.out         <> cache.io.out
  io.mmio        <> cache.io.mmio

  // Feed cache-empty back to TLB (used for page-fault response timing)
  dtlb.io.cacheEmpty := cache.io.empty

  // ─── Dummy BoringUtils sinks (sources inside DUT with no external consumer) ──
  // EmbeddedTLB sources consumed by LSU / IDU in the full system:
  val _dtlbFinish = WireInit(false.B)
  val _dtlbPF     = WireInit(false.B)
  val _dtlbAF     = WireInit(false.B)
  val _vmEnable   = WireInit(false.B)
  val _dtlbPaddr  = WireInit(0.U(64.W))
  BoringUtils.addSink(_dtlbFinish, "DTLBFINISH")
  BoringUtils.addSink(_dtlbPF,     "DTLBPF")
  BoringUtils.addSink(_dtlbAF,     "DTLBAF")
  BoringUtils.addSink(_vmEnable,   "vmEnable")
  BoringUtils.addSink(_dtlbPaddr,  "dtlb_paddr")

  // CacheStage3 adds source "lsuMMIO" for dcache; no LSU to consume it here.
  val _lsuMMIO = WireInit(false.B)
  BoringUtils.addSink(_lsuMMIO, "lsuMMIO")
}

object CacheTLBGenMain extends App {
  (new ChiselStage).execute(args ++ Array("-X", "sverilog"), Seq(
    NoCircuitDedupAnnotation,
    ChiselGeneratorAnnotation(() => new StandaloneCacheWithTLB)
  ) ++ CoverPoint.getTransforms(args)._2)
}
