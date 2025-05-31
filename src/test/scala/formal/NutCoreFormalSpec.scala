package formal

import chisel3._
import chisel3.stage.{ChiselCli, ChiselGeneratorAnnotation, ChiselStage}
import firrtl.transforms.formal.DontAssertSubmoduleAssumptionsAnnotation
import chiseltest._
import chiseltest.formal._
import org.scalatest.flatspec.AnyFlatSpec

import nutcore.{NutCore, NutCoreConfig}
import top._

class NutCoreFormalSpec extends AnyFlatSpec with Formal with ChiselScalatestTester {
  behavior of "NutCoreFormal"
  it should "pass" in {
    // config
    val s = (FormalSettings()) ++ (InOrderSettings()) ++ Map("Formal" -> false, "RVFI" -> true)
    s.foreach { Settings.settings += _ }
    Settings.settings.toList.sortBy(_._1)(Ordering.String).foreach {
      case (f, v: Long) =>
        println(f + " = 0x" + v.toHexString)
      case (f, v) =>
        println(f + " = " + v)
    }

    // (new chisel3.stage.ChiselStage).emitSystemVerilog(new NutCore()(NutCoreConfig()), Array("--target-dir", "test_run_dir/Elaborate_SpecCore_Verilog"))
    // verify
    // chiseltest.formal.verify methods:
    // verify(new NutCore()(NutCoreConfig()), Seq(BoundedCheck(12), BtormcEngineAnnotation))
    // ChiselAnnotations
    (new ChiselStage).execute(
      Array("--target-dir", "test_run_dir/Elaborate_chirvformal_SystemVerilog", "-X", "sverilog"),
      Seq(
        DontAssertSubmoduleAssumptionsAnnotation,
        ChiselGeneratorAnnotation(() => new NutCore()(NutCoreConfig()))
      )
    )
  }
}
