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

import bus.simplebus._
import chisel3._
import chisel3.experimental.ChiselAnnotation
import chisel3.util._
import chisel3.util.experimental.BoringUtils
import difftest._
import firrtl.annotations.Annotation
import rfuzz.DoNotProfileModule
import utils._

class EXU(implicit val p: NutCoreConfig) extends NutCoreModule {
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new DecodeIO))
    val out = Decoupled(new CommitIO)
    val flush = Input(Bool())
    val dmem = new SimpleBusUC(addrBits = VAddrBits)
    val forward = new ForwardIO
    val memMMU = Flipped(new MemMMUIO)
  })

  val src1 = io.in.bits.data.src1(XLEN-1,0)
  val src2 = io.in.bits.data.src2(XLEN-1,0)

  val (fuType, fuOpType) = (io.in.bits.ctrl.fuType, io.in.bits.ctrl.fuOpType)

  val fuValids = Wire(Vec(FuType.num, Bool()))
  (0 until FuType.num).map (i => fuValids(i) := (fuType === i.U) && io.in.valid && !io.flush)

  val alu = Module(new ALU(hasBru = true))
  val aluOut = alu.access(valid = fuValids(FuType.alu), src1 = src1, src2 = src2, func = fuOpType)
  alu.io.cfIn := io.in.bits.cf
  alu.io.offset := io.in.bits.data.imm
  alu.io.out.ready := true.B

  def isBru(func: UInt) = func(4)

  val lsu = Module(new UnpipelinedLSU)
  val lsuTlbPF = WireInit(false.B)
  val lsuOut = lsu.access(valid = fuValids(FuType.lsu), src1 = src1, src2 = io.in.bits.data.imm, func = fuOpType, dtlbPF = lsuTlbPF)
  lsu.io.wdata := src2
  lsu.io.instr := io.in.bits.cf.instr
  io.out.bits.isMMIO := lsu.io.isMMIO
  io.dmem <> lsu.io.dmem
  lsu.io.out.ready := true.B

  val mdu = Module(new MDU)
  val mduOut = mdu.access(valid = fuValids(FuType.mdu), src1 = src1, src2 = src2, func = fuOpType)
  mdu.io.out.ready := true.B

  // val csr = if (Settings.get("MmodeOnly")) Module(new CSR_M) else Module(new CSR)
  val csr = Module(new CSR)
  val csrOut = csr.access(valid = fuValids(FuType.csr), src1 = src1, src2 = src2, func = fuOpType)
  csr.io.cfIn := io.in.bits.cf
  csr.io.cfIn.exceptionVec(loadAddrMisaligned) := lsu.io.loadAddrMisaligned
  csr.io.cfIn.exceptionVec(storeAddrMisaligned) := lsu.io.storeAddrMisaligned
  csr.io.cfIn.exceptionVec(loadAccessFault) := lsu.io.loadAccessFault
  csr.io.cfIn.exceptionVec(storeAccessFault) := lsu.io.storeAccessFault
  csr.io.cfIn.exceptionVec(loadPageFault) := lsu.io.in.valid && lsu.io.dtlbPF && LSUOpType.hasLoadFault(fuOpType)
  csr.io.cfIn.exceptionVec(storePageFault) := lsu.io.in.valid && lsu.io.dtlbPF && LSUOpType.hasStoreFault(fuOpType)
  csr.io.instrValid := io.in.valid && !io.flush
  csr.io.isBackendException := false.B
  io.out.bits.intrNO := csr.io.intrNO
  csr.io.isBackendException := false.B
  csr.io.out.ready := true.B
  csr.io.rfWenReal := io.in.bits.ctrl.rfWen && io.in.bits.ctrl.rfDest =/= 0.U

  csr.io.imemMMU <> io.memMMU.imem
  alu.io.iVmEnable := csr.io.vmEnable
  csr.io.dmemMMU <> io.memMMU.dmem
  csr.io.dmemExceptionAddr := lsu.io.vaddr

  // When the jump is illegal, we need to access CSR instead of other function units.
  csr.io.illegalJump.valid   := alu.io.jumpIsIllegal.valid || csr.io.xretIsIllegal.valid
  csr.io.illegalJump.bits    := Mux(alu.io.jumpIsIllegal.valid, alu.io.jumpIsIllegal.bits, csr.io.xretIsIllegal.bits)
  alu.io.jumpIsIllegal.ready := io.in.valid && !io.flush
  csr.io.xretIsIllegal.ready := io.in.valid && !io.flush
  when (alu.io.jumpIsIllegal.fire || csr.io.xretIsIllegal.fire) {
    fuValids.zipWithIndex.foreach{ case (v, index) =>
      v := (index == FuType.csr.litValue).B
    }
    when (csr.io.vmEnable) {
      csr.io.cfIn.exceptionVec(instrPageFault) := true.B
    }.otherwise {
      csr.io.cfIn.exceptionVec(instrAccessFault) := true.B
    }
  }

  // read requests to performance counters should be skipped.
  when (io.out.bits.decode.ctrl.rfWen && csr.io.isPerfRead) {
    io.out.bits.isMMIO := true.B
  }

  val mou = Module(new MOU)
  // mou does not write register
  mou.access(valid = fuValids(FuType.mou), src1 = src1, src2 = src2, func = fuOpType)
  mou.io.cfIn := io.in.bits.cf
  mou.io.out.ready := true.B

  io.out.bits.decode := DontCare
  (io.out.bits.decode.ctrl, io.in.bits.ctrl) match { case (o, i) =>
    val hasException = lsuTlbPF || lsu.io.dtlbAF ||
      lsu.io.loadAddrMisaligned || lsu.io.storeAddrMisaligned ||
      lsu.io.loadAccessFault || lsu.io.storeAccessFault
    o.rfWen := i.rfWen && (!hasException || !fuValids(FuType.lsu)) && !(csr.io.wenFix && fuValids(FuType.csr))
    o.rfDest := i.rfDest
    o.fuType := i.fuType
  }
  io.out.bits.decode.cf.pc := io.in.bits.cf.pc
  io.out.bits.decode.cf.instr := io.in.bits.cf.instr
  io.out.bits.decode.cf.runahead_checkpoint_id := io.in.bits.cf.runahead_checkpoint_id
  io.out.bits.decode.cf.isBranch := io.in.bits.cf.isBranch
  io.out.bits.isExit := csr.io.isExit
  io.out.bits.decode.cf.redirect :=
    Mux(mou.io.redirect.valid, mou.io.redirect,
      Mux(csr.io.redirect.valid, csr.io.redirect, alu.io.redirect))

  Debug(mou.io.redirect.valid || csr.io.redirect.valid || alu.io.redirect.valid, "[REDIRECT] mou %x csr %x alu %x \n", mou.io.redirect.valid, csr.io.redirect.valid, alu.io.redirect.valid)
  Debug(mou.io.redirect.valid || csr.io.redirect.valid || alu.io.redirect.valid, "[REDIRECT] flush: %d mou %x csr %x alu %x\n", io.flush, mou.io.redirect.target, csr.io.redirect.target, alu.io.redirect.target)

  // FIXME: should handle io.out.ready == false
  io.out.valid := io.in.valid && (MuxLookup(fuType, true.B, List(
    FuType.lsu -> lsu.io.out.valid,
    FuType.mdu -> mdu.io.out.valid
  )) || csr.io.illegalJump.valid)

  io.out.bits.commits(FuType.alu) := aluOut
  io.out.bits.commits(FuType.lsu) := lsuOut
  io.out.bits.commits(FuType.csr) := csrOut
  io.out.bits.commits(FuType.mdu) := mduOut
  io.out.bits.commits(FuType.mou) := 0.U

  io.in.ready := !io.in.valid || io.out.fire()

  io.forward.valid := io.in.valid
  io.forward.wb.rfWen := io.in.bits.ctrl.rfWen
  io.forward.wb.rfDest := io.in.bits.ctrl.rfDest
  io.forward.wb.rfData := Mux(alu.io.out.fire(), aluOut, lsuOut)
  io.forward.fuType := io.in.bits.ctrl.fuType

  val isBru = ALUOpType.isBru(fuOpType)
  BoringUtils.addSource(alu.io.out.fire() && !isBru, "perfCntCondMaluInstr")
  BoringUtils.addSource(alu.io.out.fire() && isBru, "perfCntCondMbruInstr")
  BoringUtils.addSource(lsu.io.out.fire(), "perfCntCondMlsuInstr")
  BoringUtils.addSource(mdu.io.out.fire(), "perfCntCondMmduInstr")
  BoringUtils.addSource(csr.io.out.fire(), "perfCntCondMcsrInstr")

  if (!p.FPGAPlatform) {
    val diffMod = Module(new EXUDiffWrapper)
    diffMod.io.in.valid := io.in.valid
    diffMod.io.in.bits := io.in.bits
    diffMod.io.flush := io.flush
  }
}

// This module is not profiled by the fuzz environment
class EXUDiffWrapper(implicit val p: NutCoreConfig) extends NutCoreModule {
  val io = IO(new Bundle {
    val in = Flipped(ValidIO(new DecodeIO))
    val flush = Input(Bool())
  })

  // We are using the real counters for DiffTest to avoid being overridden by instructions
  val cycleCnt = RegInit(0.U(64.W))
  cycleCnt := cycleCnt + 1.U
  val instrCnt = RegInit(0.U(64.W))
  val instrCommit = WireInit(false.B)
  when (instrCommit) {
    instrCnt := instrCnt + 1.U
  }
  BoringUtils.addSink(instrCommit, "perfCntCondMinstret")

  val nutcoretrap = io.in.bits.ctrl.isNutCoreTrap && io.in.valid && !io.flush
  BoringUtils.addSource(nutcoretrap, "nutcoretrap")

  val difftest = DifftestModule(new DiffTrapEvent)
  difftest.clock    := clock
  difftest.coreid   := 0.U // TODO: nutshell does not support coreid auto config
  difftest.hasTrap  := nutcoretrap
  difftest.code     := io.in.bits.data.src1
  difftest.pc       := io.in.bits.cf.pc
  difftest.cycleCnt := cycleCnt
  difftest.instrCnt := instrCnt
  difftest.hasWFI   := false.B

  val noProfileMod = this.toNamed
  chisel3.experimental.annotate(new ChiselAnnotation {
    override def toFirrtl: Annotation = DoNotProfileModule(noProfileMod)
  })

}
