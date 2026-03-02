module FormalTop;
(* gclk *) wire glb_clk;
wire clock;
wire reset;
// io.in (SimpleBusUC, userBits=16) — CPU-side request/response
wire        io_in_req_ready;
wire        io_in_req_valid;
wire [31:0] io_in_req_bits_addr;
wire [2:0]  io_in_req_bits_size;
wire [3:0]  io_in_req_bits_cmd;
wire [7:0]  io_in_req_bits_wmask;
wire [63:0] io_in_req_bits_wdata;
wire [15:0] io_in_req_bits_user;
wire        io_in_resp_ready;
wire        io_in_resp_valid;
wire [3:0]  io_in_resp_bits_cmd;
wire [63:0] io_in_resp_bits_rdata;
wire [15:0] io_in_resp_bits_user;
// io.flush — pipeline flush control
wire [1:0]  io_flush;
// io.out.mem (SimpleBusUC, userBits=0) — memory-side request/response
wire        io_out_mem_req_ready;
wire        io_out_mem_req_valid;
wire [31:0] io_out_mem_req_bits_addr;
wire [2:0]  io_out_mem_req_bits_size;
wire [3:0]  io_out_mem_req_bits_cmd;
wire [7:0]  io_out_mem_req_bits_wmask;
wire [63:0] io_out_mem_req_bits_wdata;
wire        io_out_mem_resp_ready;
wire        io_out_mem_resp_valid;
wire [3:0]  io_out_mem_resp_bits_cmd;
wire [63:0] io_out_mem_resp_bits_rdata;
// io.out.coh (SimpleBusUC, userBits=0) — coherence channel
wire        io_out_coh_req_ready;
wire        io_out_coh_req_valid;
wire [31:0] io_out_coh_req_bits_addr;
wire [2:0]  io_out_coh_req_bits_size;
wire [3:0]  io_out_coh_req_bits_cmd;
wire [7:0]  io_out_coh_req_bits_wmask;
wire [63:0] io_out_coh_req_bits_wdata;
wire        io_out_coh_resp_ready;
wire        io_out_coh_resp_valid;
wire [3:0]  io_out_coh_resp_bits_cmd;
wire [63:0] io_out_coh_resp_bits_rdata;
// io.mmio (SimpleBusUC, userBits=0) — MMIO bypass
wire        io_mmio_req_ready;
wire        io_mmio_req_valid;
wire [31:0] io_mmio_req_bits_addr;
wire [2:0]  io_mmio_req_bits_size;
wire [3:0]  io_mmio_req_bits_cmd;
wire [7:0]  io_mmio_req_bits_wmask;
wire [63:0] io_mmio_req_bits_wdata;
wire        io_mmio_resp_ready;
wire        io_mmio_resp_valid;
wire [3:0]  io_mmio_resp_bits_cmd;
wire [63:0] io_mmio_resp_bits_rdata;
// io.empty — cache pipeline empty indicator
wire        io_empty;

reg reg_reset = 1'b1;
always @(posedge glb_clk) begin
  if (reg_reset) reg_reset <= 1'b0;
end

assign clock = glb_clk;
assign reset = reg_reset;

StandaloneCache dut(
  .clock,
  .reset,
  .io_in_req_ready,
  .io_in_req_valid,
  .io_in_req_bits_addr,
  .io_in_req_bits_size,
  .io_in_req_bits_cmd,
  .io_in_req_bits_wmask,
  .io_in_req_bits_wdata,
  .io_in_req_bits_user,
  .io_in_resp_ready,
  .io_in_resp_valid,
  .io_in_resp_bits_cmd,
  .io_in_resp_bits_rdata,
  .io_in_resp_bits_user,
  .io_flush,
  .io_out_mem_req_ready,
  .io_out_mem_req_valid,
  .io_out_mem_req_bits_addr,
  .io_out_mem_req_bits_size,
  .io_out_mem_req_bits_cmd,
  .io_out_mem_req_bits_wmask,
  .io_out_mem_req_bits_wdata,
  .io_out_mem_resp_ready,
  .io_out_mem_resp_valid,
  .io_out_mem_resp_bits_cmd,
  .io_out_mem_resp_bits_rdata,
  .io_out_coh_req_ready,
  .io_out_coh_req_valid,
  .io_out_coh_req_bits_addr,
  .io_out_coh_req_bits_size,
  .io_out_coh_req_bits_cmd,
  .io_out_coh_req_bits_wmask,
  .io_out_coh_req_bits_wdata,
  .io_out_coh_resp_ready,
  .io_out_coh_resp_valid,
  .io_out_coh_resp_bits_cmd,
  .io_out_coh_resp_bits_rdata,
  .io_mmio_req_ready,
  .io_mmio_req_valid,
  .io_mmio_req_bits_addr,
  .io_mmio_req_bits_size,
  .io_mmio_req_bits_cmd,
  .io_mmio_req_bits_wmask,
  .io_mmio_req_bits_wdata,
  .io_mmio_resp_ready,
  .io_mmio_resp_valid,
  .io_mmio_resp_bits_cmd,
  .io_mmio_resp_bits_rdata,
  .io_empty
);
endmodule
