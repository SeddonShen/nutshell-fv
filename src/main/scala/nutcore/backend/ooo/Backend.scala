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

trait HasBackendConst{
  // val multiIssue = true
  val robSize = 16
  val robWidth = 2
  val robInstCapacity = robSize * robWidth
  val checkpointSize = 4 // register map checkpoint size
  val brTagWidth = log2Up(checkpointSize)
  val prfAddrWidth = log2Up(robSize) + log2Up(robWidth) // physical rf addr width

  val DispatchWidth = 2
  val CommitWidth = 2
  val RetireWidth = 2

  val enableCheckpoint = true
}

class Backend_inorder(implicit val p: NutCoreConfig) extends NutCoreModule {
  val io = IO(new Bundle {
    val in = Vec(2, Flipped(Decoupled(new DecodeIO)))
    val flush = Input(UInt(2.W))
    val dmem = new SimpleBusUC(addrBits = VAddrBits)
    val memMMU = Flipped(new MemMMUIO)
    val redirect = new RedirectIO
    val symmemDMemIF = new DMemIF
  })

  val isu  = Module(new ISU)
  val exu  = Module(new EXU)
  val wbu  = Module(new WBU)

  io.symmemDMemIF <> exu.io.symmemDMemIF
  
  PipelineConnect(isu.io.out, exu.io.in, exu.io.out.fire(), io.flush(0))
  PipelineConnect(exu.io.out, wbu.io.in, true.B, io.flush(1))

  isu.io.in <> io.in
  
  isu.io.flush := io.flush(0)
  exu.io.flush := io.flush(1)

  isu.io.wb <> wbu.io.wb
  io.redirect <> wbu.io.redirect
  // forward
  isu.io.forward <> exu.io.forward  

  io.memMMU.imem <> exu.io.memMMU.imem
  io.memMMU.dmem <> exu.io.memMMU.dmem
  io.dmem <> exu.io.dmem
}