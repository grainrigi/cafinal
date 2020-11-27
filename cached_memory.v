`include "define.v"
`include "cache.v"
`include "common/sync_fifo.v"

`define PSEUDO_DRAM
`ifdef PSEUDO_DRAM
module m_mw_memory #(
          parameter APP_ADDR_WIDTH  = 28,
          parameter APP_CMD_WIDTH   = 3,
          parameter APP_DATA_WIDTH  = 128,
          parameter APP_MASK_WIDTH  = 16
) (
  input  wire                      i_clk,
  input  wire                      i_ren,
  input  wire                      i_wen,
  input  wire [APP_ADDR_WIDTH-2:0] i_addr,
  input  wire [APP_DATA_WIDTH-1:0] i_data,
  input  wire [APP_MASK_WIDTH-1:0] i_mask,
  input  wire                      i_busy,
  output wire                      o_init_calib_complete,
  output wire [APP_DATA_WIDTH-1:0] o_data,
  output wire                      o_data_valid,
  output wire                      o_busy
);
  reg  [APP_DATA_WIDTH-1:0] cm_ram[0:2**(APP_ADDR_WIDTH-4)];

  wire [APP_ADDR_WIDTH-5:0] w_addr = i_addr[APP_ADDR_WIDTH-2:3];

  // write
  integer i;
  always @(posedge i_clk) begin
    if (i_wen) begin
      for (i = 0; i < APP_MASK_WIDTH; i++) begin
        if (!i_mask[i]) cm_ram[w_addr][i*8 +: 8] <= i_data[i*8 +: 8];
      end
    end
  end

  // read
  reg [APP_ADDR_WIDTH-2:0] r_raddr;       // reading address
  reg [APP_DATA_WIDTH-1:0] r_rdata;       // read data
  reg                      r_valid;       // r_rdata is valid or not
  reg [3:0]                r_count = 0;   // read wait count
  reg                      r_reading = 0; // is waiting (decrementing r_count)
  reg                      r_rst = 1;     // reset (for fifo)
  initial #100 r_rst <= 0;

  wire                      w_fifo_full;
  wire                      w_fifo_empty;
  wire                      w_fifo_available = !w_fifo_empty;
  wire [APP_ADDR_WIDTH-2:0] w_fifo_data;
  SyncFIFO #(
             .DATA_WIDTH(APP_ADDR_WIDTH-1),
             .ADDR_WIDTH(3))
  sfifo(
        .clk(i_clk),
        .i_rst(r_rst),
        .i_wen(i_ren && !w_fifo_full),
        .i_data(i_addr),
        .i_ren(w_fifo_available && r_count == 0),
        .o_data(w_fifo_data),
        .o_empty(w_fifo_empty),
        .o_full(w_fifo_full));

  always @(posedge i_clk) begin
    if (r_count != 0) begin
      r_count <= r_count - 1;
      r_valid <= 0;
    end else begin
      if (r_reading) begin
        r_rdata <= cm_ram[r_raddr[APP_ADDR_WIDTH-2:3]];
      end
      if (w_fifo_available) begin
        r_raddr <= w_fifo_data;
        r_count <= 15;
      end
      r_reading <= (w_fifo_available) ? 1 : 0;
      r_valid <= (r_reading) ? 1 : 0;
    end
  end

  assign o_data = r_rdata;
  assign o_data_valid = r_valid;

  assign o_init_calib_complete = r_rst;
  assign o_busy = r_rst || w_fifo_full;
endmodule
`endif

module m_cached_memory #(
                    parameter DDR3_DQ_WIDTH   = 16,
                    parameter DDR3_DQS_WIDTH  = 2,
                    parameter DDR3_ADDR_WIDTH = 14,
                    parameter DDR3_BA_WIDTH   = 3,
                    parameter DDR3_DM_WIDTH   = 2,
                    parameter APP_ADDR_WIDTH  = 28,
                    parameter APP_CMD_WIDTH   = 3,
                    parameter APP_DATA_WIDTH  = 128,
                    parameter APP_MASK_WIDTH  = 16)
    (
`ifndef PSEUDO_DRAM
     // sys_clk: input clock (166.67MHz),
     // ref_clk: reference clock (200MHz),
     // sys_rst: reset (active-high)
     input  wire                         sys_clk,
     input  wire                         ref_clk,
     input  wire                         sys_rst,
     // dram interface signals
     inout  wire [DDR3_DQ_WIDTH-1 : 0]   ddr3_dq,
     inout  wire [DDR3_DQS_WIDTH-1 : 0]  ddr3_dqs_n,
     inout  wire [DDR3_DQS_WIDTH-1 : 0]  ddr3_dqs_p,
     output wire [DDR3_ADDR_WIDTH-1 : 0] ddr3_addr,
     output wire [DDR3_BA_WIDTH-1 : 0]   ddr3_ba,
     output wire                         ddr3_ras_n,
     output wire                         ddr3_cas_n,
     output wire                         ddr3_we_n,
     output wire                         ddr3_reset_n,
     output wire [0:0]                   ddr3_ck_p,
     output wire [0:0]                   ddr3_ck_n,
     output wire [0:0]                   ddr3_cke,
     output wire [0:0]                   ddr3_cs_n,
     output wire [DDR3_DM_WIDTH-1 : 0]   ddr3_dm,
     output wire [0:0]                   ddr3_odt,
     // output clock and reset (active-high) signals for user design
     output wire                         o_clk,
     output wire                         o_rst,
`else
     input  wire                         i_clk,
`endif
     // user design interface signals
     input  wire                         i_dmem_init_done,
     input  wire [3:0]                   i_dmem_init_wen,
     input  wire [31:0]                  i_dmem_init_addr,
     input  wire [31:0]                  i_dmem_init_data,
     input  wire                         i_dmem_ren,
     input  wire [3:0]                   i_dmem_wen,
     input  wire [31:0]                  i_dmem_addr,
     input  wire [31:0]                  i_dmem_data,
     output wire [31:0]                  o_dmem_data,
     output wire                         o_dmem_stall);

    localparam STATE_DRAM_CALIB     = 3'b000;
    localparam STATE_DRAM_IDLE      = 3'b001;
    localparam STATE_DRAM_WRITE     = 3'b010;
    localparam STATE_DRAM_READ      = 3'b011;
    localparam STATE_DRAM_READ_WAIT = 3'b100;

    wire                        clk;
    wire                        rst;

    wire                        dram_ren;
    wire                        dram_wen;
    wire [APP_ADDR_WIDTH-2 : 0] dram_addr;
    wire [2:0]                  dram_addr_column_offset;
    reg  [APP_DATA_WIDTH-1 : 0] dram_din;
    reg  [APP_MASK_WIDTH-1 : 0] dram_mask;
    wire                        dram_init_calib_complete;
    wire [APP_DATA_WIDTH-1 : 0] dram_dout;
    wire                        dram_dout_valid;
    wire                        dram_busy;

    wire                        user_design_busy;

    reg  [APP_DATA_WIDTH-1:0]   dram_dout_reg;

    wire                        dmem_ren;
    wire [3:0]                  dmem_wen;
    wire [31:0]                 dmem_addr;
    wire [31:0]                 dmem_din;
    reg  [31:0]                 dmem_dout;
    wire                        dmem_stall;

    reg  [3:0]                  dmem_wen_reg;
    reg  [31:0]                 dmem_addr_reg;
    reg  [31:0]                 dmem_din_reg;

    reg  [2:0]                  state = STATE_DRAM_CALIB;

    integer i;

`ifndef PSEUDO_DRAM
    assign o_clk = clk;
    assign o_rst = rst;
`else
    assign clk = i_clk;
`endif
    assign o_dmem_data = dmem_dout;
    assign o_dmem_stall = dmem_stall;

    assign dmem_ren   = (i_dmem_init_done)? i_dmem_ren  : 0;
    assign dmem_wen   = (i_dmem_init_done)? i_dmem_wen  : i_dmem_init_wen;
    assign dmem_addr  = (i_dmem_init_done)? i_dmem_addr : i_dmem_init_addr;
    assign dmem_din   = (i_dmem_init_done)? i_dmem_data : i_dmem_init_data;

    always @(*) begin
        dmem_dout = 0;
        for (i = 0; i < 4; i = i + 1) begin // 4: APP_DATA_WIDTH/32
            if (dram_addr_column_offset[2:1] == i) begin
                dmem_dout = dram_dout_reg[i*32 +: 32];
            end
        end
    end

    assign dmem_stall = (state != STATE_DRAM_IDLE);

    assign dram_ren = ((state == STATE_DRAM_READ) && !dram_busy);
    assign dram_wen = ((state == STATE_DRAM_WRITE) && !dram_busy);
    assign dram_addr = {dmem_addr_reg[APP_ADDR_WIDTH-1 : 4], 3'b000};
    assign dram_addr_column_offset = dmem_addr_reg[3:1];

    always @(*) begin
        dram_din = 0;
        for (i = 0; i < 4; i = i + 1) begin // 4: APP_DATA_WIDTH/32
            if (dram_addr_column_offset[2:1] == i) begin
                dram_din[i*32 +: 32] = dmem_din_reg;
            end
        end
    end

    always @(*) begin
        dram_mask = {(APP_MASK_WIDTH){1'b1}};
        for (i = 0; i < APP_MASK_WIDTH; i = i + 4) begin // 4: 32/8
            if ({dram_addr_column_offset, 1'b0} == i) begin
                dram_mask[i +: 4] = (~dmem_wen_reg);
            end
        end
    end

    // in this implementation, user design is stalled when dram is accessed;
    // thus, when data are available, user design can always accept them
    assign user_design_busy = 1'b0;

`ifndef PSEUDO_DRAM
    DRAM #(
           .DDR3_DQ_WIDTH(DDR3_DQ_WIDTH),
           .DDR3_DQS_WIDTH(DDR3_DQS_WIDTH),
           .DDR3_ADDR_WIDTH(DDR3_ADDR_WIDTH),
           .DDR3_BA_WIDTH(DDR3_BA_WIDTH),
           .DDR3_DM_WIDTH(DDR3_DM_WIDTH),
           .APP_ADDR_WIDTH(APP_ADDR_WIDTH),
           .APP_CMD_WIDTH(APP_CMD_WIDTH),
           .APP_DATA_WIDTH(APP_DATA_WIDTH),
           .APP_MASK_WIDTH(APP_MASK_WIDTH))
    dram (
          // input clock (166.67MHz),
          // reference clock (200MHz),
          // reset (active-high)
          .sys_clk(sys_clk),
          .ref_clk(ref_clk),
          .sys_rst(sys_rst),
          // dram interface signals
          .ddr3_dq(ddr3_dq),
          .ddr3_dqs_n(ddr3_dqs_n),
          .ddr3_dqs_p(ddr3_dqs_p),
          .ddr3_addr(ddr3_addr),
          .ddr3_ba(ddr3_ba),
          .ddr3_ras_n(ddr3_ras_n),
          .ddr3_cas_n(ddr3_cas_n),
          .ddr3_we_n(ddr3_we_n),
          .ddr3_reset_n(ddr3_reset_n),
          .ddr3_ck_p(ddr3_ck_p),
          .ddr3_ck_n(ddr3_ck_n),
          .ddr3_cke(ddr3_cke),
          .ddr3_cs_n(ddr3_cs_n),
          .ddr3_dm(ddr3_dm),
          .ddr3_odt(ddr3_odt),
          // output clock and reset (active-high) signals for user design
          .o_clk(clk),
          .o_rst(rst),
`else
  m_mw_memory #(
           .APP_ADDR_WIDTH(APP_ADDR_WIDTH),
           .APP_CMD_WIDTH(APP_CMD_WIDTH),
           .APP_DATA_WIDTH(APP_DATA_WIDTH),
           .APP_MASK_WIDTH(APP_MASK_WIDTH))
  dram (
          .i_clk(i_clk),
`endif
          // user design interface signals
          .i_ren(dram_ren),
          .i_wen(dram_wen),
          .i_addr(dram_addr),
          .i_data(dram_din),
          .i_mask(dram_mask),
          .i_busy(user_design_busy),
          .o_init_calib_complete(dram_init_calib_complete),
          .o_data(dram_dout),
          .o_data_valid(dram_dout_valid),
          .o_busy(dram_busy));

    always @(posedge clk) begin
        if (rst) begin
            state <= STATE_DRAM_CALIB;
            dmem_wen_reg <= 0;
            dmem_addr_reg <= 0;
            dmem_din_reg <= 0;

            dram_dout_reg <= 0;
        end else begin
            case (state)
                STATE_DRAM_CALIB: begin
                    if (dram_init_calib_complete) begin
                        state <= STATE_DRAM_IDLE;
                    end
                end
                STATE_DRAM_IDLE: begin
                    dmem_wen_reg <= dmem_wen;
                    dmem_addr_reg <= dmem_addr;
                    dmem_din_reg <= dmem_din;
                    if (dmem_wen != 0) begin
                        state <= STATE_DRAM_WRITE;
                    end else if (dmem_ren) begin
                        state <= STATE_DRAM_READ;
                    end
                end
                STATE_DRAM_WRITE: begin
                    if (!dram_busy) begin
                        state <= STATE_DRAM_IDLE;
                    end
                end
                STATE_DRAM_READ: begin
                    if (!dram_busy) begin
                        state <= STATE_DRAM_READ_WAIT;
                    end
                end
                default: begin // STATE_DRAM_READ_WAIT
                    dram_dout_reg <= dram_dout;
                    if (dram_dout_valid) begin
                        state <= STATE_DRAM_IDLE;
                    end
                end
            endcase
        end
    end
endmodule