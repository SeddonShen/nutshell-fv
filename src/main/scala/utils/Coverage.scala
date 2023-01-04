/**************************************************************************************
* Copyright (c) 2022-2023 Institute of Computing Technology, CAS
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

package utils

import chisel3._
import chisel3.experimental.ChiselAnnotation
import chisel3.util._
import difftest._
import firrtl.annotations.Annotation
import nutcore.{FuType, HasInstrType}
import rfuzz.DoNotProfileModule

abstract class Coverage extends Module {
  def n_cover: Int
  lazy val out = Wire(Valid(UInt(log2Ceil(n_cover).W)))

  def createDiffMod[T <: DiffCoverage](coverType: => T): Unit = {
    val difftest = DifftestModule(coverType)
    difftest.clock   := clock
    difftest.coreid  := 0.U
    difftest.valid   := out.valid
    difftest.address := out.bits
    difftest.covered := true.B
  }

  val noProfileMod = this.toNamed
   chisel3.experimental.annotate(new ChiselAnnotation {
     override def toFirrtl: Annotation = DoNotProfileModule(noProfileMod)
   })
}

class CoverInstr(decodeTable: Seq[BitPat]) extends Coverage {
  val in = IO(Flipped(ValidIO(UInt(32.W))))
  def cover(v: Bool, i: UInt): Unit = {
    in.valid := v
    in.bits  := i
  }

  val numInstrTypes = decodeTable.length
  val instrMatchVec = decodeTable.map(_ === in.bits)

  override def n_cover: Int = numInstrTypes
  class DiffInstrCover extends DiffCoverage("icover", n_cover)
  createDiffMod(new DiffInstrCover)

  out.valid := in.valid && VecInit(instrMatchVec).asUInt.orR
  out.bits  := OHToUInt(instrMatchVec)
}

class CoverInstrImm(decodeTable: Seq[(BitPat, List[UInt])]) extends Coverage with HasInstrType {
  val in = IO(Flipped(ValidIO(UInt(32.W))))
  def cover(v: Bool, i: UInt): Unit = {
    in.valid := v
    in.bits  := i
  }

  val instrMatchVec = decodeTable.map(_._1 === in.bits)
  val instrImmCoverMatch = decodeTable.map(_._2.map(_.litValue)).map(t => {
    if (t.head == InstrI.litValue.toInt) {
      if (t.tail.head == FuType.csr.litValue.toInt) (1 << 12, in.bits(31, 20))
      else (1 << 2, Cat(in.bits(31), in.bits(20)))
    }
    else if (t.head == InstrS.litValue.toInt)
      (1 << 2, Cat(in.bits(31), in.bits(7)))
    else if (t.head == InstrB.litValue.toInt)
      (1 << 1, in.bits(31))
    else if (t.head == InstrU.litValue.toInt)
      (1 << 1, in.bits(31))
    else if (t.head == InstrJ.litValue.toInt)
      (1 << 1, in.bits(31))
    else if (t.head == InstrSA.litValue.toInt)
      (1 << 2, Cat(in.bits(31), in.bits(7)))
    else (1 << 0, 0.U)
  })
  val numInstrImmCover = instrImmCoverMatch.map(_._1).sum
  val baseAddress = instrImmCoverMatch.indices.map(i => instrImmCoverMatch.map(_._1).take(i).sum)
  val baseAddressU = PriorityMux(instrMatchVec, baseAddress.map(_.U(log2Ceil(numInstrImmCover).W)))
  val offsetU = PriorityMux(instrMatchVec, instrImmCoverMatch.map(_._2))
  val coverAddress = baseAddressU + offsetU

  override def n_cover: Int = numInstrImmCover
  class DiffInstrImmCover extends DiffCoverage("instr_imm_cover", n_cover)
  createDiffMod(new DiffInstrImmCover)

  out.valid := in.valid && VecInit(instrMatchVec).asUInt.orR
  out.bits  := coverAddress
}