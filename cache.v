`include "define.v"

// write-noallocate cache
// multiword (4-words)
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
  input  wire [`EADDR] i_raddr, // read address
  input  wire [`EADDR] i_waddr, // write address
  input  wire         i_we,    // write enable
  input  wire [31:0]  i_data,  // write data
  output reg  [127:0] o_data,  // read data
  input  wire         i_bwe,   // write enable (cache install)
  input  wire [127:0] i_bdata, // write data (cache install)
  output wire         o_hit,   // cache hit (read)
  output wire [1:0]   o_bindex
);
  localparam INDEX_WIDTH = 8;
  localparam NUM_ENTRIES = 2 ** INDEX_WIDTH;
  localparam TAG_WIDTH   = `EADDR_WIDTH - INDEX_WIDTH - 4;
  localparam EWIDTH = 1 + TAG_WIDTH + 128;
  reg  [TAG_WIDTH:0] meta_ram[0:NUM_ENTRIES-1]; // 256 entries

  // read no delay (for metadata reading)
  wire [TAG_WIDTH-1:0]   w_ritag   = i_raddr[`EADDR_WIDTH-1:`EADDR_WIDTH-TAG_WIDTH];  // tag
  wire [INDEX_WIDTH-1:0] w_rindex  = i_raddr[INDEX_WIDTH+3:4];                      // entry index
  wire [1:0]             w_rbindex = i_raddr[3:2];                                  // block index

  // write no delay (for cache install)
  wire [TAG_WIDTH-1:0]   w_witag   = i_waddr[`EADDR_WIDTH-1:`EADDR_WIDTH-TAG_WIDTH];  // tag
  wire [INDEX_WIDTH-1:0] w_windex  = i_waddr[INDEX_WIDTH+3:4];                      // entry index

  // read delay (for hit judge)
  reg  [TAG_WIDTH:0]     r_rmeta;
  reg  [`EADDR]           r_raddr;
  wire [TAG_WIDTH-1:0]   w_dritag  = r_raddr[`EADDR_WIDTH-1:`EADDR_WIDTH-TAG_WIDTH];
  wire [TAG_WIDTH-1:0]   w_dretag   = r_rmeta[TAG_WIDTH-1:0];
  wire                   w_drhit    = r_rmeta[TAG_WIDTH] && (w_dretag == w_dritag);
  wire [1:0]             w_drbindex = r_raddr[3:2];
  assign o_hit = w_drhit;
  assign o_bindex = w_drbindex;

  // write delay (for sw)
  reg  [`EADDR]           r_waddr;
  reg  [31:0]            r_wdata;
  reg                    r_we;
  reg                    r_iwe;
  wire [TAG_WIDTH-1:0]   w_dwitag   = r_waddr[`EADDR_WIDTH-1:`EADDR_WIDTH-TAG_WIDTH];
  wire [INDEX_WIDTH-1:0] w_dwindex  = r_waddr[INDEX_WIDTH+3:4];
  wire [1:0]             w_dwbindex = r_waddr[3:2];

  reg  [TAG_WIDTH:0]   r_wmeta;
  wire [TAG_WIDTH-1:0] w_dwetag = r_wmeta[TAG_WIDTH-1:0];
  wire                 w_dwhit  = r_wmeta[TAG_WIDTH] && (w_dwetag == w_dwitag);

  // simultaneous read and install handling
  wire w_read_installing = (i_bwe) && (w_witag == w_ritag) && (w_windex == w_rindex);
  reg  r_read_installing;
  reg [127:0] r_bdata;

  integer i;
  always @(*) begin
    for (i = 0; i < 4; i = i + 1) begin
      o_data[i*32 +: 32] = (r_read_installing && w_drbindex == i) ? r_bdata[i*32 +: 32] : w_drdata[i*32 +: 32];
    end
  end

  wire [127:0] w_drdata;

  genvar g;
  generate
  for (g = 0; g < 4; g = g + 1) begin: data_ram
    m_cache_store #(
      .INDEX_WIDTH(INDEX_WIDTH)
    ) store (
      .i_clk(i_clk),
      .i_rindex(w_rindex),
      .i_windex((i_bwe) ? w_windex : w_dwindex),
      .i_we(((w_dwbindex == g) && r_we && w_dwhit) || i_bwe),
      .i_data((i_bwe) ? i_bdata[g*32 +: 32] : r_wdata),
      .o_data(w_drdata[g*32 +: 32])
    );
  end
  endgenerate

  always @(posedge i_clk) if (i_bwe) meta_ram[w_windex] <= {1'b1, w_witag};
  always @(posedge i_clk) begin
     r_wmeta <= meta_ram[w_windex];
     r_rmeta <= meta_ram[w_rindex];
     r_raddr <= i_raddr;
     r_waddr <= i_waddr;
     r_wdata <= i_data;
     r_we <= i_we;
     r_read_installing <= w_read_installing;
     r_bdata <= i_bdata;
  end
  
  initial begin
    for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
      meta_ram[i] = 0;
    end
  end
endmodule