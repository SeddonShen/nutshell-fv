package formal

import chisel3._
import chisel3.stage._
import chiseltest._
import chiseltest.formal._
import org.scalatest.flatspec.AnyFlatSpec

import nutcore.{NutCore, NutCoreConfig}
import sim.SimTop
import top._

class NutCoreFormalSpec extends AnyFlatSpec with Formal with ChiselScalatestTester {
  behavior of "NutCoreFormal"
  it should "pass" in {
    // config
    // val s = (FormalSettings()) ++ (InOrderSettings())
    // val s = (FormalSettings()) ++ (InOrderSettings())
    val s = (OOOSettings())
    s.foreach { Settings.settings += _ }
    Settings.settings.toList.sortBy(_._1)(Ordering.String).foreach {
      case (f, v: Long) =>
        println(f + " = 0x" + v.toHexString)
      case (f, v) =>
        println(f + " = " + v)
    }
    // (new ChiselStage).execute(args, Seq(
    //   ChiselGeneratorAnnotation(() => new SimTop))
    // )
    // (new chisel3.stage.ChiselStage)
    //   .emitSystemVerilog(new NutCore()(NutCoreConfig()), Array("--target-dir", "test_run_dir/Elaborate_ooo"))
    // verify
    verify(new NutCore()(NutCoreConfig()), Seq(BoundedCheck(15), BtormcEngineAnnotation))
  }
}
