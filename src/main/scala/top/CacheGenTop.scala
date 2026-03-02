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
import xfuzz.CoverPoint
import firrtl.transforms.NoCircuitDedupAnnotation

/**
 * Standalone wrapper for NutShell dcache, used for independent BMCFuzz testing.
 *
 * Configuration: CacheConfig(ro=false, name="dcache", userBits=16)
 * Parameters from DefaultSettings: XLEN=64, PAddrBits=32, EnableOutOfOrderExec=false
 *
 * IO ports (CacheIO):
 *   - in:    SimpleBusUC(userBits=16) — CPU-side request/response
 *   - flush: UInt(2.W)               — pipeline flush control
 *   - out:   SimpleBusC              — memory-side (mem + coh)
 *   - mmio:  SimpleBusUC             — MMIO bypass
 *   - empty: Bool                    — cache pipeline empty indicator
 */
class StandaloneCache extends Module
    with HasNutCoreParameter with HasNutCoreConst {
  implicit val cacheConfig: CacheConfig = CacheConfig(
    ro = false, name = "dcache", userBits = DCacheUserBundleWidth
  )
  val io = IO(new CacheIO)
  val cache = Module(new Cache)
  cache.io <> io

  // CacheStage3 calls BoringUtils.addSource(mmio, "lsuMMIO") for dcache.
  // In standalone mode there is no LSU to consume it, so we add a dummy sink
  // to satisfy the FIRRTL WiringTransform source/sink pairing requirement.
  val dummyLsuMMIO = WireInit(false.B)
  BoringUtils.addSink(dummyLsuMMIO, "lsuMMIO")
}

object CacheGenMain extends App {
  val args_sv = args ++ Array("-X", "sverilog")
  (new ChiselStage).execute(args_sv, Seq(
    NoCircuitDedupAnnotation,
    ChiselGeneratorAnnotation(() => new StandaloneCache)
  ) ++ CoverPoint.getTransforms(args)._2)
}
