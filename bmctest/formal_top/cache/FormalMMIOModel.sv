/**
 * FormalMMIOModel — SimpleBus-compliant MMIO model for Cache formal verification.
 *
 * Handles MMIO requests (addr in 0x40000000..0x7FFFFFFF). Simplified:
 * - Accepts mmio_req, returns mmio_resp after one cycle
 * - rdata = 0 (or arbitrary; MMIO semantics are device-specific)
 * - Ensures proper handshake, eliminates unconstrained env
 */
module FormalMMIOModel (
  input         clock,
  input         reset,

  output        req_ready,
  input         req_valid,
  input  [31:0] req_bits_addr,
  input  [2:0]  req_bits_size,
  input  [3:0]  req_bits_cmd,
  input  [7:0]  req_bits_wmask,
  input  [63:0] req_bits_wdata,

  input         resp_ready,
  output        resp_valid,
  output [3:0]  resp_bits_cmd,
  output [63:0] resp_bits_rdata
);

  localparam [3:0] CMD_READ_LAST  = 4'b0110;
  localparam [3:0] CMD_WRITE_RESP = 4'b0101;

  reg pending_resp;
  reg is_read;
  assign req_ready = !pending_resp && !reset;
  assign resp_valid = pending_resp;
  assign resp_bits_rdata = 64'b0;
  assign resp_bits_cmd = is_read ? CMD_READ_LAST : CMD_WRITE_RESP;

  wire req_is_read = (req_bits_cmd[0] == 0) && (req_bits_cmd[3] == 0);

  always @(posedge clock) begin
    if (reset) begin
      pending_resp <= 0;
      is_read <= 0;
    end else begin
      if (resp_ready && pending_resp) pending_resp <= 0;
      if (req_valid && req_ready) begin
        pending_resp <= 1;
        is_read <= req_is_read;
      end
    end
  end
endmodule
