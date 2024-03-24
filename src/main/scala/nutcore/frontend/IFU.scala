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

package nutcore

import chisel3._
import chisel3.util._
import chisel3.util.experimental.BoringUtils

import utils._
import bus.simplebus._
import top.Settings

trait HasResetVector {
  val resetVector = Settings.getLong("ResetVector")
}

class ICacheUserBundle extends NutCoreBundle {
    val pc = UInt(VAddrBits.W)
    val brIdx = UInt(4.W) // mark if an inst is predicted to branch
    val pnpc = UInt(VAddrBits.W)
    val instValid = UInt(4.W) // mark which part of this inst line is valid
}
// Note: update ICacheUserBundleWidth when change ICacheUserBundle

class IFU_embedded extends NutCoreModule with HasResetVector {
  val io = IO(new Bundle {
    val imem = new SimpleBusUC(userBits = 64, addrBits = VAddrBits)
    val out = Decoupled(new CtrlFlowIO)
    val redirect = Flipped(new RedirectIO)
    val flushVec = Output(UInt(4.W))
    val bpFlush = Output(Bool())
    val ipf = Input(Bool())
    val symmemIMemIF = new IMemIF
  })

  // pc
  val pc = RegInit(resetVector.U(32.W))
  // val pcUpdate = io.redirect.valid || io.imem.req.fire()
  val pcUpdate = io.redirect.valid || (io.symmemIMemIF.instructionReady && io.symmemIMemIF.fetchEnable)
  val snpc = pc + 4.U  // sequential next pc

  val bpu = Module(new BPU_embedded)

  // predicted next pc
  val pnpc = bpu.io.out.target
  val npc = Mux(io.redirect.valid, io.redirect.target, Mux(bpu.io.out.valid, pnpc, snpc))
  
  // bpu.io.in.pc.valid := io.imem.req.fire() // only predict when Icache accepts a request
  bpu.io.in.pc.valid := (io.symmemIMemIF.instructionReady && io.symmemIMemIF.fetchEnable)
  bpu.io.in.pc.bits := npc  // predict one cycle early
  bpu.io.flush := io.redirect.valid

  when (pcUpdate) { pc := npc }

  io.flushVec := Mux(io.redirect.valid, "b1111".U, 0.U)
  io.bpFlush := false.B

  io.imem := DontCare
  io.symmemIMemIF.address := pc
  io.symmemIMemIF.fetchEnable := io.out.ready
  // io.imem.req.bits.apply(addr = pc, size = "b10".U, cmd = SimpleBusCmd.read, wdata = 0.U, wmask = 0.U, user = Cat(pc, npc))
  // printf("[IFU] pc %x npc %x\n", pc, npc)
  // io.imem.req.valid := io.out.ready
  // io.imem.resp.ready := io.out.ready || io.flushVec(0)

  io.out.bits := DontCare
  // io.out.bits.instr := io.imem.resp.bits.rdata
  io.out.bits.instr := io.symmemIMemIF.instruction
  // io.imem.resp.bits.user.map{ case x =>
  //   io.out.bits.pc := x(2*VAddrBits-1, VAddrBits)
  //   io.out.bits.pnpc := x(VAddrBits-1, 0)
  // }
  io.out.bits.pc := pc
  io.out.bits.pnpc := npc
  // printf user, pc and pnpc
  // printf("[IFU] user %x\n", io.imem.resp.bits.user.get)
  // printf("[IFU] pc %x\n", io.out.bits.pc)
  // printf("[IFU] pnpc %x\n", io.out.bits.pnpc)
  // io.out.valid := io.imem.resp.valid && !io.flushVec(0)
  io.out.valid := io.symmemIMemIF.instructionReady && !io.flushVec(0)

  

  // Debug(io.imem.req.fire(), "[IFI] pc=%x user=%x redirect %x npc %x pc %x pnpc %x\n", io.imem.req.bits.addr, io.imem.req.bits.user.getOrElse(0.U), io.redirect.valid, npc, pc, bpu.io.out.target)
  // Debug(io.out.fire(), "[IFO] pc=%x user=%x inst=%x npc=%x ipf %x\n", io.out.bits.pc, io.imem.resp.bits.user.get, io.out.bits.instr, io.out.bits.pnpc, io.ipf)

  BoringUtils.addSource(BoolStopWatch(io.imem.req.valid, io.imem.resp.fire()), "perfCntCondMimemStall")
  BoringUtils.addSource(io.flushVec.orR, "perfCntCondMifuFlush")
}