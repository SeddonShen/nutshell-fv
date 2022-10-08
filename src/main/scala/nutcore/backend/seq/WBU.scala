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
import difftest._

class WBU(implicit val p: NutCoreConfig) extends NutCoreModule{
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new CommitIO))
    val wb = new WriteBackIO
    val redirect = new RedirectIO
  })

  io.wb.rfWen := io.in.bits.decode.ctrl.rfWen && io.in.valid
  io.wb.rfDest := io.in.bits.decode.ctrl.rfDest
  io.wb.rfData := io.in.bits.commits(io.in.bits.decode.ctrl.fuType)

  io.in.ready := true.B

  io.redirect := io.in.bits.decode.cf.redirect
  io.redirect.valid := io.in.bits.decode.cf.redirect.valid && io.in.valid

  // val runahead_redirect = DifftestModule(new DiffRunaheadRedirectEvent)
  // runahead_redirect.clock := clock
  // runahead_redirect.coreid := 0.U
  // runahead_redirect.valid := io.redirect.valid
  // runahead_redirect.pc := io.in.bits.decode.cf.pc // for debug only
  // runahead_redirect.target_pc := io.in.bits.decode.cf.redirect.target // for debug only
  // runahead_redirect.checkpoint_id := io.in.bits.decode.cf.runahead_checkpoint_id // make sure it is right

  // when(runahead_redirect.io.valid) {
  //   printf("DUT pc %x redirect to %x cpid %x\n", runahead_redirect.io.pc, runahead_redirect.io.target_pc, runahead_redirect.io.checkpoint_id)
  // }

  Debug(io.in.valid, "[COMMIT] pc = 0x%x inst %x wen %x wdst %x wdata %x mmio %x intrNO %x\n", io.in.bits.decode.cf.pc, io.in.bits.decode.cf.instr, io.wb.rfWen, io.wb.rfDest, io.wb.rfData, io.in.bits.isMMIO, io.in.bits.intrNO)

  val falseWire = WireInit(false.B) // make BoringUtils.addSource happy
  BoringUtils.addSource(io.in.valid, "perfCntCondMinstret")
  BoringUtils.addSource(falseWire, "perfCntCondMultiCommit")

  if (!p.FPGAPlatform) {
    val difftest_commit = DifftestModule(new DiffInstrCommit)
    difftest_commit.clock    := clock
    difftest_commit.coreid   := 0.U
    difftest_commit.index    := 0.U

    difftest_commit.valid    := RegNext(io.in.valid)
    difftest_commit.pc       := RegNext(SignExt(io.in.bits.decode.cf.pc, AddrBits))
    difftest_commit.instr    := RegNext(io.in.bits.decode.cf.instr)
    difftest_commit.skip     := RegNext(io.in.bits.isMMIO)
    difftest_commit.isRVC    := RegNext(io.in.bits.decode.cf.instr(1,0)=/="b11".U)
    difftest_commit.rfwen    := RegNext(io.wb.rfWen && io.wb.rfDest =/= 0.U) // && valid(ringBufferTail)(i) && commited(ringBufferTail)(i)
    difftest_commit.fpwen    := false.B
    difftest_commit.special  := 0.U
    difftest_commit.wdest    := RegNext(io.wb.rfDest)
    difftest_commit.wpdest   := RegNext(io.wb.rfDest)

    val difftest_wb = DifftestModule(new DiffIntWriteback)
    difftest_wb.clock := clock
    difftest_wb.coreid := 0.U
    difftest_wb.valid := RegNext(io.wb.rfWen && io.wb.rfDest =/= 0.U)
    difftest_wb.dest := RegNext(io.wb.rfDest)
    difftest_wb.data := RegNext(io.wb.rfData)

    // val runahead_commit = DifftestModule(new DiffRunaheadCommitEvent)
    // runahead_commit.clock := clock
    // runahead_commit.coreid := 0.U
    // runahead_commit.index := 0.U
    // runahead_commit.valid := RegNext(io.in.valid && io.in.bits.decode.cf.isBranch)
    // runahead_commit.pc    := RegNext(SignExt(io.in.bits.decode.cf.pc, AddrBits))
  } else {
    BoringUtils.addSource(io.in.valid, "ilaWBUvalid")
    BoringUtils.addSource(io.in.bits.decode.cf.pc, "ilaWBUpc")
    BoringUtils.addSource(io.wb.rfWen, "ilaWBUrfWen")
    BoringUtils.addSource(io.wb.rfDest, "ilaWBUrfDest")
    BoringUtils.addSource(io.wb.rfData, "ilaWBUrfData")
  }
}
