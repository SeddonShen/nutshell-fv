/**
 * FormalTop — Cache formal verification wrapper with FormalMemModel and FormalMMIOModel.
 *
 * Connects StandaloneCache to:
 * - FormalMemModel: backing store for io.out.mem (read/write consistency)
 * - FormalMMIOModel: MMIO responses for io.mmio
 * - Coherence: no probe (coh_resp_valid=0), coh_req_ready=1
 */
module FormalTop;
(* gclk *) wire glb_clk;
wire clock;
wire reset;

// io.in — CPU-side (unconstrained inputs for BMC)
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
wire [1:0]  io_flush;

// io.out.mem — driven by FormalMemModel
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

// io.out.coh — no probe, accept cache output
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

// io.mmio — driven by FormalMMIOModel
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

wire        io_empty;

reg reg_reset = 1'b1;
always @(posedge glb_clk) begin
  if (reg_reset) reg_reset <= 1'b0;
end

assign clock = glb_clk;
assign reset = reg_reset;

// Coherence: no probe. Cache drives req_ready (output) and resp_* (output).
// We drive req_* (input: no requests) and resp_ready (input: we accept).
assign io_out_coh_req_valid = 1'b0;
assign io_out_coh_req_bits_addr = 32'b0;
assign io_out_coh_req_bits_size = 3'b0;
assign io_out_coh_req_bits_cmd = 4'b0;
assign io_out_coh_req_bits_wmask = 8'b0;
assign io_out_coh_req_bits_wdata = 64'b0;
assign io_out_coh_resp_ready = 1'b1;

FormalMemModel #(.ADDR_BITS(16), .LINE_BEATS(32)) mem_model (
  .clock(clock),
  .reset(reset),
  .req_ready(io_out_mem_req_ready),
  .req_valid(io_out_mem_req_valid),
  .req_bits_addr(io_out_mem_req_bits_addr),
  .req_bits_size(io_out_mem_req_bits_size),
  .req_bits_cmd(io_out_mem_req_bits_cmd),
  .req_bits_wmask(io_out_mem_req_bits_wmask),
  .req_bits_wdata(io_out_mem_req_bits_wdata),
  .resp_ready(io_out_mem_resp_ready),
  .resp_valid(io_out_mem_resp_valid),
  .resp_bits_cmd(io_out_mem_resp_bits_cmd),
  .resp_bits_rdata(io_out_mem_resp_bits_rdata)
);

FormalMMIOModel mmio_model (
  .clock(clock),
  .reset(reset),
  .req_ready(io_mmio_req_ready),
  .req_valid(io_mmio_req_valid),
  .req_bits_addr(io_mmio_req_bits_addr),
  .req_bits_size(io_mmio_req_bits_size),
  .req_bits_cmd(io_mmio_req_bits_cmd),
  .req_bits_wmask(io_mmio_req_bits_wmask),
  .req_bits_wdata(io_mmio_req_bits_wdata),
  .resp_ready(io_mmio_resp_ready),
  .resp_valid(io_mmio_resp_valid),
  .resp_bits_cmd(io_mmio_resp_bits_cmd),
  .resp_bits_rdata(io_mmio_resp_bits_rdata)
);

StandaloneCache dut(
  .clock(clock),
  .reset(reset),
  .io_in_req_ready(io_in_req_ready),
  .io_in_req_valid(io_in_req_valid),
  .io_in_req_bits_addr(io_in_req_bits_addr),
  .io_in_req_bits_size(io_in_req_bits_size),
  .io_in_req_bits_cmd(io_in_req_bits_cmd),
  .io_in_req_bits_wmask(io_in_req_bits_wmask),
  .io_in_req_bits_wdata(io_in_req_bits_wdata),
  .io_in_req_bits_user(io_in_req_bits_user),
  .io_in_resp_ready(io_in_resp_ready),
  .io_in_resp_valid(io_in_resp_valid),
  .io_in_resp_bits_cmd(io_in_resp_bits_cmd),
  .io_in_resp_bits_rdata(io_in_resp_bits_rdata),
  .io_in_resp_bits_user(io_in_resp_bits_user),
  .io_flush(io_flush),
  .io_out_mem_req_ready(io_out_mem_req_ready),
  .io_out_mem_req_valid(io_out_mem_req_valid),
  .io_out_mem_req_bits_addr(io_out_mem_req_bits_addr),
  .io_out_mem_req_bits_size(io_out_mem_req_bits_size),
  .io_out_mem_req_bits_cmd(io_out_mem_req_bits_cmd),
  .io_out_mem_req_bits_wmask(io_out_mem_req_bits_wmask),
  .io_out_mem_req_bits_wdata(io_out_mem_req_bits_wdata),
  .io_out_mem_resp_ready(io_out_mem_resp_ready),
  .io_out_mem_resp_valid(io_out_mem_resp_valid),
  .io_out_mem_resp_bits_cmd(io_out_mem_resp_bits_cmd),
  .io_out_mem_resp_bits_rdata(io_out_mem_resp_bits_rdata),
  .io_out_coh_req_ready(io_out_coh_req_ready),
  .io_out_coh_req_valid(io_out_coh_req_valid),
  .io_out_coh_req_bits_addr(io_out_coh_req_bits_addr),
  .io_out_coh_req_bits_size(io_out_coh_req_bits_size),
  .io_out_coh_req_bits_cmd(io_out_coh_req_bits_cmd),
  .io_out_coh_req_bits_wmask(io_out_coh_req_bits_wmask),
  .io_out_coh_req_bits_wdata(io_out_coh_req_bits_wdata),
  .io_out_coh_resp_ready(io_out_coh_resp_ready),
  .io_out_coh_resp_valid(io_out_coh_resp_valid),
  .io_out_coh_resp_bits_cmd(io_out_coh_resp_bits_cmd),
  .io_out_coh_resp_bits_rdata(io_out_coh_resp_bits_rdata),
  .io_mmio_req_ready(io_mmio_req_ready),
  .io_mmio_req_valid(io_mmio_req_valid),
  .io_mmio_req_bits_addr(io_mmio_req_bits_addr),
  .io_mmio_req_bits_size(io_mmio_req_bits_size),
  .io_mmio_req_bits_cmd(io_mmio_req_bits_cmd),
  .io_mmio_req_bits_wmask(io_mmio_req_bits_wmask),
  .io_mmio_req_bits_wdata(io_mmio_req_bits_wdata),
  .io_mmio_resp_ready(io_mmio_resp_ready),
  .io_mmio_resp_valid(io_mmio_resp_valid),
  .io_mmio_resp_bits_cmd(io_mmio_resp_bits_cmd),
  .io_mmio_resp_bits_rdata(io_mmio_resp_bits_rdata),
  .io_empty(io_empty)
);
endmodule
