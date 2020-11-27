`include "define.v"

// write-noallocate cache
// multiword (4-words)
`ifdef CACHE_ASYNC
module m_cache 
(
  input  wire         i_clk,   // clock
  input  wire [`ADDR] i_addr,  // operation address
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
module m_cache_store #(
  parameter INDEX_WIDTH = 8
)
(
  input  wire        i_clk,
  input  wire [INDEX_WIDTH-1:0] i_rindex,
  input  wire [INDEX_WIDTH-1:0] i_windex,
  input  wire        i_we,
  input  wire [31:0] i_data,
  output reg  [31:0] o_data
);
  localparam NUM_ENTRIES = 2 ** INDEX_WIDTH;
  
  reg [31:0] cm_ram[0:NUM_ENTRIES-1];
  always @(posedge i_clk) if (i_we) cm_ram[i_windex] <= i_data;
  always @(posedge i_clk) o_data <= cm_ram[i_rindex];
endmodule

module m_cache 
(
  input  wire         i_clk,   // clock
  input  wire [`ADDR] i_raddr, // read address
  input  wire [`ADDR] i_waddr, // write address
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
  reg  [TAG_WIDTH:0] meta_ram[0:NUM_ENTRIES-1]; // 256 entries

  wire [TAG_WIDTH-1:0]   w_ritag   = i_raddr[`ADDR_WIDTH-1:`ADDR_WIDTH-TAG_WIDTH];  // tag
  wire [INDEX_WIDTH-1:0] w_rindex  = i_raddr[INDEX_WIDTH+3:4];                      // entry index
  wire [1:0]             w_rbindex = i_raddr[3:2];                                  // block index
  wire [TAG_WIDTH-1:0]   w_witag   = i_waddr[`ADDR_WIDTH-1:`ADDR_WIDTH-TAG_WIDTH];  // tag
  wire [INDEX_WIDTH-1:0] w_windex  = i_waddr[INDEX_WIDTH+3:4];                      // entry index

  reg  [`ADDR]           r_waddr;
  reg  [31:0]            r_wdata;
  reg                    r_we;
  wire [TAG_WIDTH-1:0]   w_dwitag   = r_waddr[`ADDR_WIDTH-1:`ADDR_WIDTH-TAG_WIDTH];
  wire [INDEX_WIDTH-1:0] w_dwindex  = r_waddr[INDEX_WIDTH+3:4];
  wire [1:0]             w_dwbindex = r_waddr[3:2];

  reg  [TAG_WIDTH:0]   r_meta;
  wire [TAG_WIDTH-1:0] w_etag = r_meta[TAG_WIDTH-1:0];
  wire                 w_hit  = r_meta[TAG_WIDTH] && (w_etag == w_ritag);
  assign o_hit = w_hit;

  genvar g;
  generate
  for (g = 0; g < 4; g = g + 1) begin: data_ram
    m_cache_store #(
      .INDEX_WIDTH(INDEX_WIDTH)
    ) store (
      .i_clk(i_clk),
      .i_rindex(w_rindex),
      .i_windex((i_bwe) ? w_windex : w_dwindex),
      .i_we(((w_dwbindex == g) && r_we && w_hit) || i_bwe),
      .i_data((i_bwe) ? i_bdata[g*32 +: 32] : r_wdata),
      .o_data(o_data[g*32 +: 32])
    );
  end
  endgenerate

  always @(posedge i_clk) if (i_bwe) meta_ram[w_windex] <= {1'b1, w_witag};
  always @(posedge i_clk) begin
     r_meta <= meta_ram[w_windex];
     r_waddr <= i_waddr;
     r_wdata <= i_data;
     r_we <= i_we;
  end
  
  integer i;
  initial begin
    for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
      meta_ram[i] = 0;
    end
  end
endmodule
`endif