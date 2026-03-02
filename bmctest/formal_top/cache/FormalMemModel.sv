/**
 * FormalMemModel — SimpleBus-compliant memory model for Cache formal verification.
 *
 * Simulates a backing store for Cache's io.out.mem channel. Ensures:
 * - Read responses return data consistent with prior writes
 * - Burst transactions (readBurst, writeBurst/writeLast) are handled correctly
 * - Eliminates false positives from unconstrained environment
 *
 * SimpleBus cmd: read=0x0, write=0x1, readBurst=0x2, writeBurst=0x3, writeLast=0x7
 *               readLast=0x6, writeResp=0x5
 */
module FormalMemModel #(
  parameter ADDR_BITS = 16,   // log2 of 64-bit words (16 -> 512KB)
  parameter LINE_BEATS = 32   // 256B line -> 32 words
) (
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

  localparam [ADDR_BITS-1:0] BEAT_MASK = LINE_BEATS - 1;  // for cache-line wrap-around

  localparam [3:0] CMD_READ        = 4'b0000;
  localparam [3:0] CMD_WRITE       = 4'b0001;
  localparam [3:0] CMD_READ_BURST  = 4'b0010;
  localparam [3:0] CMD_WRITE_BURST = 4'b0011;
  localparam [3:0] CMD_WRITE_LAST  = 4'b0111;
  localparam [3:0] CMD_READ_LAST   = 4'b0110;
  localparam [3:0] CMD_WRITE_RESP  = 4'b0101;

  reg [63:0] mem [0:(1<<ADDR_BITS)-1];

  wire [ADDR_BITS-1:0] word_addr = req_bits_addr[ADDR_BITS+2:3];
  wire req_is_read  = (req_bits_cmd == CMD_READ) || (req_bits_cmd == CMD_READ_BURST);
  wire req_is_write = (req_bits_cmd == CMD_WRITE) || (req_bits_cmd == CMD_WRITE_BURST) || (req_bits_cmd == CMD_WRITE_LAST);

  // Merge wmask into existing word
  reg [63:0] merged;
  integer i;
  always @(*) begin
    merged = mem[word_addr];
    for (i = 0; i < 8; i = i + 1)
      if (req_bits_wmask[i]) merged[i*8 +: 8] = req_bits_wdata[i*8 +: 8];
  end

  // State
  reg [5:0] read_beat_cnt;
  reg [31:0] read_base_addr;
  reg [5:0] write_beat_cnt;
  reg [31:0] write_base_addr;
  reg sending_read_resp;
  reg sending_write_resp;
  reg in_read_burst;
  reg in_write_burst;

  reg [3:0] resp_cmd;
  reg [63:0] resp_rdata;
  assign resp_bits_cmd = resp_cmd;
  assign resp_bits_rdata = resp_rdata;

  wire idle = !in_read_burst && !in_write_burst && !sending_read_resp && !sending_write_resp;
  // During write burst, only accept the next write-burst beat (not reads).
  // This prevents in_read_burst and in_write_burst from being set simultaneously.
  wire req_cmd_is_write_burst_beat = (req_bits_cmd == CMD_WRITE_BURST || req_bits_cmd == CMD_WRITE_LAST);
  assign req_ready = (idle || (in_write_burst && !sending_write_resp && req_cmd_is_write_burst_beat)) && !reset;
  assign resp_valid = sending_read_resp || sending_write_resp;

  always @(posedge clock) begin
    if (reset) begin
      read_beat_cnt <= 0;
      read_base_addr <= 0;
      write_beat_cnt <= 0;
      write_base_addr <= 0;
      sending_read_resp <= 0;
      sending_write_resp <= 0;
      in_read_burst <= 0;
      in_write_burst <= 0;
      resp_cmd <= 0;
      resp_rdata <= 0;
    end else begin
      // Consume resp when ready
      if (resp_ready && sending_read_resp) begin
        if (read_beat_cnt == LINE_BEATS - 1) begin
          sending_read_resp <= 0;
          in_read_burst <= 0;
        end else begin
          read_beat_cnt <= read_beat_cnt + 1;
          // Wrap within the cache line: advance word index modulo LINE_BEATS,
          // keeping the line-base address unchanged.
          resp_rdata <= mem[(read_base_addr[ADDR_BITS+2:3] & ~BEAT_MASK) |
                            ((read_base_addr[ADDR_BITS+2:3] +
                              {{(ADDR_BITS-6){1'b0}}, read_beat_cnt} + 1'b1) & BEAT_MASK)];
          resp_cmd <= (read_beat_cnt + 1 == LINE_BEATS - 1) ? CMD_READ_LAST : 0;
        end
      end
      if (resp_ready && sending_write_resp) sending_write_resp <= 0;

      // Accept req (idle or next write burst beat)
      if (req_valid && req_ready) begin
        if (in_write_burst && (req_bits_cmd == CMD_WRITE_BURST || req_bits_cmd == CMD_WRITE_LAST)) begin
          mem[req_bits_addr[ADDR_BITS+2:3]] <= merged;
          if (req_bits_cmd == CMD_WRITE_LAST) begin
            resp_cmd <= CMD_WRITE_RESP;
            sending_write_resp <= 1;
            in_write_burst <= 0;
          end else
            write_beat_cnt <= write_beat_cnt + 1;
        end else if (req_is_read) begin
          if (req_bits_cmd == CMD_READ) begin
            resp_rdata <= mem[word_addr];
            resp_cmd <= CMD_READ_LAST;
            sending_read_resp <= 1;
          end else begin
            read_base_addr <= {req_bits_addr[31:3], 3'b0};
            read_beat_cnt <= 0;
            resp_rdata <= mem[req_bits_addr[ADDR_BITS+2:3]];
            resp_cmd <= (LINE_BEATS == 1) ? CMD_READ_LAST : 0;
            sending_read_resp <= 1;
            in_read_burst <= 1;
          end
        end else if (req_is_write) begin
          mem[word_addr] <= merged;
          if (req_bits_cmd == CMD_WRITE) begin
            resp_cmd <= CMD_WRITE_RESP;
            sending_write_resp <= 1;
          end else if (req_bits_cmd == CMD_WRITE_LAST) begin
            resp_cmd <= CMD_WRITE_RESP;
            sending_write_resp <= 1;
            in_write_burst <= 0;
          end else begin
            write_base_addr <= {req_bits_addr[31:3], 3'b0};
            write_beat_cnt <= 1;
            in_write_burst <= 1;
          end
        end
      end
    end
  end
endmodule
