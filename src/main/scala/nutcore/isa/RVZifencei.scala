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

object RVZifenceiInstr extends HasInstrType {
  // The unused fields in the FENCE.I instruction, imm[11:0], rs1, and rd,
  // are reserved for finer-grain fences in future extensions. For forward
  // compatibility, base implementations shall ignore these fields, and
  // standard software shall zero these fields.
  def FENCEI = BitPat("b????????????_?????_001_?????_0001111")

  val table = Array(
    FENCEI -> List(InstrB, FuType.mou, MOUOpType.fencei)
  )
}
