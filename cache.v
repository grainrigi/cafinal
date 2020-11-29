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

module m_cache #(
  parameter INDEX_WIDTH = 8
)(
  input  wire          i_clk,   // clock
  input  wire [`DADDR] i_addr,  // 32bit-data operation address (read/write)
  output wire          o_hit,   // 
  input  wire          i_we,    // write enable
  output wire          o_we,    // write enable
  input  wire [31:0]   i_data,  // write data
  output reg  [127:0]  o_data,  // read data
  output reg           o_rhit,  // read cache hit (o_data is valid or not)
  output wire [1:0]    o_bindex,// read data block index
  input  wire          i_ie,    // install enable
  input  wire [`DADDR] i_iaddr, // install address (4 lsb is ignored)
  input  wire [127:0]  i_idata  // write data (cache install)
);
  localparam NUM_ENTRIES = 2 ** INDEX_WIDTH;
  localparam TAG_WIDTH   = `DADDR_WIDTH - INDEX_WIDTH - 4;
  localparam EWIDTH = 1 + TAG_WIDTH + 128;
  reg  [TAG_WIDTH:0] meta_ram[0:NUM_ENTRIES-1]; // 256 entries

  // 32bit-data operation data
  wire [TAG_WIDTH-1:0]   w_itag   = i_addr[`DADDR_WIDTH-1:`DADDR_WIDTH-TAG_WIDTH];  // tag
  wire [INDEX_WIDTH-1:0] w_index  = i_addr[INDEX_WIDTH+3:4];                      // entry index
  wire [1:0]             w_bindex = i_addr[3:2];                                  // block index
  wire [TAG_WIDTH:0]     w_meta   = meta_ram[w_index];
  wire [TAG_WIDTH-1:0]   w_etag   = w_meta[TAG_WIDTH-1:0];
  wire                   w_valid  = w_meta[TAG_WIDTH];
  wire                   w_hit    = w_valid && (w_etag == w_itag);

  always @(posedge i_clk) o_rhit <= w_hit;
  assign o_hit = w_hit;

  // delay write
  reg                    r_we;
  reg  [`DADDR]          r_daddr;
  reg  [`DADDR]          r_ddata;
  wire [INDEX_WIDTH-1:0] w_dindex  = r_daddr[INDEX_WIDTH+3:4];
  wire [1:0]             w_dbindex = r_daddr[3:2];
  assign o_we = r_we;

  // cache install
  wire [TAG_WIDTH-1:0]   w_iitag   = i_iaddr[`DADDR_WIDTH-1:`DADDR_WIDTH-TAG_WIDTH];  // tag
  wire [INDEX_WIDTH-1:0] w_iindex  = i_iaddr[INDEX_WIDTH+3:4];                      // entry index

  // simultaneous read and install handling
  assign o_bindex = w_dbindex;
  wire w_read_installing = (i_ie) && (w_iitag == w_itag) && (w_iindex == w_index);
  reg  r_read_installing;
  reg [31:0] r_installed_data;

  integer i;
  always @(*) begin
    for (i = 0; i < 4; i = i + 1) begin
      o_data[i*32 +: 32] = (r_read_installing && w_dbindex == i) ? r_installed_data : w_drdata[i*32 +: 32];
    end
  end

  wire [127:0] w_drdata;

  genvar g;
  generate
  for (g = 0; g < 4; g = g + 1) begin: data_ram
    wire this_we = (w_dbindex == g) && r_we && o_rhit;
    m_cache_store #(
      .INDEX_WIDTH(INDEX_WIDTH)
    ) store (
      .i_clk(i_clk),
      .i_rindex(w_index),
      .i_windex((i_ie) ? w_iindex : w_dindex),
      .i_we(this_we || i_ie),
      .i_data((i_ie) ? i_idata[g*32 +: 32] : r_ddata),
      .o_data(w_drdata[g*32 +: 32])
    );
  end
  endgenerate

  always @(posedge i_clk) if (i_ie) meta_ram[w_iindex] <= {1'b1, w_iitag};
  always @(posedge i_clk) begin
     if (i_we && i_ie) begin
       $write("simultaneous write and install is not permitted.\n");
       #1000 $finish();
     end
     r_read_installing <= w_read_installing;
     r_we    <= i_we;
     r_daddr <= i_addr;
     r_ddata <= i_data;
     r_installed_data <= i_idata[w_bindex*32 +: 32];
  end
  
  initial begin
    for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
      meta_ram[i] = 0;
    end
  end
endmodule