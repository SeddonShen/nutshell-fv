package nutcore

import chisel3._
import chisel3.util._
import chisel3.util.experimental.BoringUtils

class RVFIIO extends Bundle {
  val valid = Output(Bool())
  val order = Output(UInt(64.W))
  val insn = Output(UInt(64.W))
  val trap = Output(Bool())
  val halt = Output(Bool())
  val intr = Output(Bool())
  val mode = Output(UInt(2.W))
  val ixl = Output(UInt(2.W))
  val rs1_addr = Output(UInt(5.W))
  val rs2_addr = Output(UInt(5.W))
  val rs1_rdata = Output(UInt(64.W))
  val rs2_rdata = Output(UInt(64.W))
  val rd_addr = Output(UInt(5.W))
  val rd_wdata = Output(UInt(64.W))
  val pc_rdata = Output(UInt(64.W))
  val pc_wdata = Output(UInt(64.W))
  val mem_addr = Output(UInt(64.W))
  val mem_rmask = Output(UInt(8.W))
  val mem_wmask = Output(UInt(8.W))
  val mem_rdata = Output(UInt(64.W))
  val mem_wdata = Output(UInt(64.W))
}

class RVFIMem extends Bundle {
  val addr_real = Output(UInt(64.W))
  val rmask = Output(UInt(8.W))
  val wmask = Output(UInt(8.W))
  val rdata = Output(UInt(64.W))
  val wdata = Output(UInt(64.W))
}