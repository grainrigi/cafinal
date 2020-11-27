`include "define.v"

// write-noallocate cache
// multiword (4-words)
`ifdef CACHE_ASYNC
module m_cache 
(
  input  wire         i_clk,   // clock
  input  wire [`ADDR] i_addr,  // operation address
  input  wire         i_we,    // write enable
  input  wire [31:0]  i_data,  // write data
  output wire [127:0] o_data,  // read data
  input  wire         i_bwe,   // write enable (cache install)
  input  wire [127:0] i_bdata, // write data (cache install)
  output wire         o_hit    // cache hit
);
  localparam INDEX_WIDTH = 8;
  localparam NUM_ENTRIES = 2 ** INDEX_WIDTH;
  localparam TAG_WIDTH   = `ADDR_WIDTH - INDEX_WIDTH - 4;
  localparam EWIDTH = 1 + TAG_WIDTH + 128;
  reg  [EWIDTH-1:0] cm_ram[0:NUM_ENTRIES-1]; // 256 entries

  wire [TAG_WIDTH-1:0]   w_itag   = i_addr[`ADDR_WIDTH-1:`ADDR_WIDTH-TAG_WIDTH];  // tag
  wire [INDEX_WIDTH-1:0] w_index  = i_addr[INDEX_WIDTH+3:4];                      // entry index
  wire [1:0]             w_bindex = i_addr[3:2];                                  // block index

  wire [EWIDTH-1:0]    w_entry = cm_ram[w_index];
  wire [TAG_WIDTH-1:0] w_etag  = w_entry[EWIDTH-2 -: TAG_WIDTH];
  wire                 w_hit   = w_entry[EWIDTH-1] && (w_etag == w_itag);
  assign o_data = w_entry[127:0];
  assign o_hit  = w_hit;

  always @(posedge i_clk) begin
    if (i_we && w_hit) begin
      cm_ram[w_index][w_bindex*32 +: 32] <= i_data;
    end else if (i_bwe) begin
      cm_ram[w_index] <= {1'b1, w_itag, i_bdata};
    end
  end
endmodule
`else
module m_cache 
(
  input  wire         i_clk,   // clock
  input  wire [`ADDR] i_addr,  // operation address
  input  wire         i_we,    // write enable
  input  wire [31:0]  i_data,  // write data
  output reg  [127:0] o_data,  // read data
  input  wire         i_bwe,   // write enable (cache install)
  input  wire [127:0] i_bdata, // write data (cache install)
  output wire         o_hit    // cache hit
);
  localparam INDEX_WIDTH = 12;
  localparam NUM_ENTRIES = 2 ** INDEX_WIDTH;
  localparam TAG_WIDTH   = `ADDR_WIDTH - INDEX_WIDTH - 4;
  localparam EWIDTH = 1 + TAG_WIDTH + 128;
  reg  [TAG_WIDTH:0] meta_ram[0:NUM_ENTRIES-1]; // 256 entries
  reg  [127:0]       data_ram[0:NUM_ENTRIES-1]; // 256 entries

  wire [TAG_WIDTH-1:0]   w_itag   = i_addr[`ADDR_WIDTH-1:`ADDR_WIDTH-TAG_WIDTH];  // tag
  wire [INDEX_WIDTH-1:0] w_index  = i_addr[INDEX_WIDTH+3:4];                      // entry index
  wire [1:0]             w_bindex = i_addr[3:2];                                  // block index

  wire [TAG_WIDTH:0]   w_meta = meta_ram[w_index];
  wire [TAG_WIDTH-1:0] w_etag = w_meta[TAG_WIDTH-1:0];
  wire                 w_hit  = w_meta[TAG_WIDTH] && (w_etag == w_itag);
  assign o_hit = w_hit;

  wire [127:0] w_block = (i_bwe) ? i_bdata : data_ram[w_index];

  always @(posedge i_clk) begin
    // first, write the latest data
    if (i_we && w_hit) begin
      data_ram[w_index][32*w_bindex +: 32] = i_data;
    end else if (i_bwe) begin
      meta_ram[w_index] <= {1'b1, w_itag};
      data_ram[w_index] = i_bdata;
    end
    // then read cache data into o_data
    o_data = data_ram[w_index];
  end

  integer i;
  initial begin
    for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
      meta_ram[i] = 0;
    end
  end
endmodule
`endif