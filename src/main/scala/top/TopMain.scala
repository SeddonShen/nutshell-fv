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
import device.AXI4VGA
import difftest.DifftestModule
import difuzz.ControlRegisterCoverage
import firrtl.stage.RunFirrtlTransformAnnotation
import nutcore.NutCoreConfig
import rfuzz.{NoDedupTransform, ProfilingTransform, SplitMuxConditions}
import sic.{FsmCoverage, LineCoverage, ReadyValidCoverage, ToggleCoverage}
import sim.SimTop
import system.NutShell
import xfuzz.{ControlRegisterCoverTransform, CoverPoint, CoverPointTransform, DontTouchClockAndResetTransform, MuxCoverTransform}
import firrtl.options.Dependency
import firrtl.transforms.NoCircuitDedupAnnotation

class Top extends Module {
  val io = IO(new Bundle{})
  val nutshell = Module(new NutShell()(NutCoreConfig()))
  val vga = Module(new AXI4VGA)

  nutshell.io := DontCare
  vga.io := DontCare
  dontTouch(nutshell.io)
  dontTouch(vga.io)
}

object TopMain extends App {
  def parseArgs(info: String, args: Array[String]): String = {
    var target = ""
    for (arg <- args) {
      if (arg.startsWith(info + "=")) {
        target = arg
      }
    }
    if (target == "") "" else target.substring(info.length() + 1)
  }

  val board = parseArgs("BOARD", args)
  val core = parseArgs("CORE", args)

  val s = (board match {
    case "sim" => Nil
    case "pynq" => PynqSettings()
    case "axu3cg" => Axu3cgSettings()
    case "PXIe" => PXIeSettings()
  }) ++ (core match {
    case "inorder" => InOrderSettings()
    case "ooo" => OOOSettings()
    case "embedded" => EmbededSettings()
  })
  s.foreach {
    Settings.settings += _
  } // add and overwrite DefaultSettings
  println("====== Settings = (" + board + ", " + core + ") ======")
  Settings.settings.toList.sortBy(_._1)(Ordering.String).foreach {
    case (f, v: Long) =>
      println(f + " = 0x" + v.toHexString)
    case (f, v) =>
      println(f + " = " + v)
  }

  val args_systemverilog = args ++ Array("-X", "sverilog")
  if (board == "sim") {
    (new ChiselStage).execute(args_systemverilog, Seq(
      NoCircuitDedupAnnotation,
      ChiselGeneratorAnnotation(() => new SimTop),
    ) ++ CoverPoint.getTransforms(args)._2)
  } else {
    (new ChiselStage).execute(args_systemverilog, Seq(
      ChiselGeneratorAnnotation(() => new Top)
    ) ++ CoverPoint.getTransforms(args)._2)
  }

  // DifftestModule.finish("nutshell")
}
