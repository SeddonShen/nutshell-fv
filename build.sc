/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

// This file is adapted from OpenXiangShan/XiangShan/build.sc
import os.Path
import mill._
import scalalib._
import publish._
// import coursier.maven.MavenRepository
import $file.difftest.build
import $file.difftest.instrumentation.instrumentation.build

object ivys {
  val sv = "2.12.16"
  val chisel3 = ivy"edu.berkeley.cs::chisel3:3.5.6"
  val chisel3Plugin = ivy"edu.berkeley.cs:::chisel3-plugin:3.5.6"
  val chiseltest = ivy"edu.berkeley.cs::chiseltest:0.5.4"
  val chiselCirct = ivy"com.sifive::chisel-circt:0.4.0"
  val scalatest = ivy"org.scalatest::scalatest:3.2.2"
  val macroParadise = ivy"org.scalamacros:::paradise:2.1.1"
}

trait NSModule extends ScalaModule with PublishModule {

  // override this to use chisel from source
  def chiselOpt: Option[PublishModule] = None

  override def scalaVersion = ivys.sv

  override def compileIvyDeps = Agg(ivys.macroParadise)

  override def scalacPluginIvyDeps = Agg(ivys.macroParadise, ivys.chisel3Plugin)

  override def scalacOptions = Seq("-Xsource:2.11")

  override def ivyDeps = (if(chiselOpt.isEmpty) Agg(ivys.chisel3) else Agg.empty[Dep]) ++ Agg(ivys.chiselCirct)

  override def moduleDeps = Seq() ++ chiselOpt

  def publishVersion = "0.0.1"

  // TODO: fix this
  def pomSettings = PomSettings(
    description = "NutShell",
    organization = "",
    url = "https://github.com/poemonsense/NutShell",
    licenses = Seq(),
    versionControl = VersionControl.github("poemonsense", "NutShell"),
    developers = Seq.empty
  )
}

object difftestDep extends difftest.build.CommonDiffTest {

  object fuzz extends difftest.instrumentation.instrumentation.build.CommonRFuzz {
    def sourceRoot = T.sources { T.workspace / "difftest" / "instrumentation" / "instrumentation" / "src" }

    def allSources = T { sourceRoot().flatMap(p => os.walk(p.path)).map(PathRef(_)) }
  }

  override def fuzzModule: PublishModule = fuzz

  override def millSourcePath = os.pwd / "difftest"
}

trait CommonNutShell extends NSModule with SbtModule { m =>
  // module deps
  def difftestModule: PublishModule

  override def millSourcePath = os.pwd

  override def forkArgs = Seq("-Xmx1G", "-Xss256m")

  override def ivyDeps = super.ivyDeps() ++ Seq(ivys.chiseltest)

  override def moduleDeps = super.moduleDeps ++ Seq(
    difftestModule
  )

  object test extends Tests with TestModule.ScalaTest {

    override def forkArgs = m.forkArgs

    override def ivyDeps = super.ivyDeps() ++ Agg(
      ivys.scalatest
    )
  }

}

object NutShell extends CommonNutShell {
  override def difftestModule = difftestDep
}
