package formal

import chisel3._
import chiseltest._
import chiseltest.formal._
import org.scalatest.flatspec.AnyFlatSpec

import nutcore.{NutCore, NutCoreConfig}
import top._

class NutCoreFormalSpec extends AnyFlatSpec with Formal with ChiselScalatestTester {
  behavior of "NutCoreFormal"
  it should "pass" in {
    // config
    val s = (FormalSettings()) ++ (InOrderSettings()) ++ Map("Formal" -> true, "RVFI" -> true)
    s.foreach { Settings.settings += _ }
    Settings.settings.toList.sortBy(_._1)(Ordering.String).foreach {
      case (f, v: Long) =>
        println(f + " = 0x" + v.toHexString)
      case (f, v) =>
        println(f + " = " + v)
    }

    // (new chisel3.stage.ChiselStage).emitSystemVerilog(new NutCore()(NutCoreConfig()), Array("--target-dir", "test_run_dir/Elaborate_SpecCore_Verilog"))
    // verify
    verify(new NutCore()(NutCoreConfig()), Seq(BoundedCheck(12), BtormcEngineAnnotation))
  }
}
