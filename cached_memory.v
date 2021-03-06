`include "define.v"
`include "cache.v"

`ifdef PSEUDO_DRAM
`include "common/sync_fifo.v"
module m_mw_memory #(
          parameter APP_ADDR_WIDTH  = 28,
          parameter APP_CMD_WIDTH   = 3,
          parameter APP_DATA_WIDTH  = 128,
          parameter APP_MASK_WIDTH  = 16
) (
  output wire                      o_rst,
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
      for (i = 0; i < APP_MASK_WIDTH; i = i + 1) begin
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
  assign o_rst = r_rst;

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
      r_rdata <= 128'h12345678ABCDEF0123456789ABCDEF01;
    end else begin
      if (r_reading) begin
        r_rdata <= cm_ram[r_raddr[APP_ADDR_WIDTH-2:3]];
        r_valid <= 1;
      end else if (!i_busy) begin
        r_rdata <= 128'h12345678ABCDEF0123456789ABCDEF01;
        r_valid <= 0;
      end
      if (w_fifo_available) begin
        r_raddr <= w_fifo_data;
        r_count <= 15;
      end
      r_reading <= (w_fifo_available) ? 1 : 0;
    end
  end

  assign o_data = r_rdata;
  assign o_data_valid = r_valid;

  assign o_init_calib_complete = !r_rst;
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
     input  wire [`DADDR]                i_dmem_init_addr,
     input  wire [31:0]                  i_dmem_init_data,
     input  wire                         i_dmem_ren,
     input  wire [3:0]                   i_dmem_wen,
     input  wire [`DADDR]                i_dmem_addr,
     input  wire [31:0]                  i_dmem_data,
     output wire [31:0]                  o_dmem_data,
     output wire                         o_dmem_stall);

    localparam INDEX_WIDTH             = 9;

    localparam TASK_WAIT_CALIB         = 4'b0000;
    localparam TASK_IDLE               = 4'b0001;
    localparam TASK_CACHE_READ         = 4'b0010;
    localparam TASK_WRITE_THROUGH      = 4'b0011;
    localparam TASK_WRITE_ISSUE_STALL  = 4'b0100;
    localparam TASK_READ_ISSUE_STALL   = 4'b0101;
    localparam TASK_READ_WAIT          = 4'b0110;
    localparam TASK_READ_ISSUE         = 4'b0111;
    localparam TASK_WRITE_ISSUE        = 4'b1000;
    localparam TASK_COMPLETE_READ      = 4'b1001;
    localparam TASK_SAVE_DRAM_RESULT   = 4'b1010;
    localparam TASK_CACHE_READ_STALL   = 4'b1011;

    localparam READ_NONE               = 3'b000;
    localparam READ_GET                = 3'b010;
    localparam READ_GET_THEN_PREFETCH  = 3'b011;
    localparam READ_PREFETCH           = 3'b100;
    localparam READ_PREFETCH_THEN_GET  = 3'b101;

    wire                        clk;
    wire                        rst;

    wire                        dram_ren;
    wire                        dram_wen;
    wire [`DADDR]               dram_addr_org;
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

    reg  [2:0]                  read_state;
    wire                        read_stall;
    wire                        read_get_valid;
    wire                        read_prefetch_valid;
    wire                        read_get_prefetch_coalesce;
    wire                        prefetch_issuable;
    wire                        prefetch_issuing;
    wire [`DADDR]               prefetch_addr;
    wire                        prefetch_installable;
    wire                        prefetch_installing;
    wire [`DADDR]               prefetch_install_addr;
    wire [APP_DATA_WIDTH-1:0]   prefetch_install_data;

    wire                        cache_read_hit;
    wire [APP_DATA_WIDTH-1:0]   cache_dout;
    wire [1:0]                  cache_bindex;
    wire                        cache_install;
    wire [`DADDR]               cache_install_addr;
    wire [APP_DATA_WIDTH-1:0]   cache_install_data;
    wire                        cache_write_now;

    wire [APP_DATA_WIDTH-1:0]   dmem_raw_data;

    wire                        dmem_ren;
    wire [3:0]                  dmem_wen;
    wire [`DADDR]               dmem_addr;
    wire [31:0]                 dmem_din;
    reg  [31:0]                 dmem_dout;
    wire                        dmem_stall;
    wire [1:0]                  dmem_bindex;

    reg                         dmem_ren_reg;
    reg  [3:0]                  dmem_wen_reg;
    reg  [`DADDR]               dmem_addr_reg;
    reg  [31:0]                 dmem_din_reg;
    reg                         dmem_use_dramout;

    reg  [3:0]                  prev_task = 0;

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

    assign dmem_raw_data = C_TASK_COMPLETE_READ ? dram_dout_reg : cache_dout;
    assign dmem_bindex   = C_TASK_COMPLETE_READ ? dmem_addr_reg[3:2] : cache_bindex;

    always @(*) begin
        dmem_dout = 0;
        for (i = 0; i < 4; i = i + 1) begin // 4: APP_DATA_WIDTH/32
            if (dmem_bindex == i) begin
                dmem_dout = dmem_raw_data[i*32 +: 32];
            end
        end
    end

    assign dmem_stall = (
       !dram_init_calib_complete
    || (prev_task == TASK_CACHE_READ && !cache_read_hit)
    || (prev_task == TASK_READ_ISSUE)
    || (prev_task == TASK_READ_ISSUE_STALL)
    || (prev_task == TASK_READ_WAIT)
    || (!dram_busy && (prev_task == TASK_WRITE_THROUGH || prev_task == TASK_WRITE_ISSUE_STALL) && (dmem_ren || dmem_wen))
    || (dram_busy && (prev_task == TASK_WRITE_THROUGH || prev_task == TASK_WRITE_ISSUE_STALL))
    );

    assign dram_ren = C_TASK_READ_ISSUE || prefetch_issuing;
    assign dram_wen = C_TASK_WRITE_ISSUE;
    assign dram_addr_org = (C_TASK_READ_ISSUE || C_TASK_WRITE_ISSUE) ? dmem_addr_reg : prefetch_addr;
    assign dram_addr = {4'd0, dram_addr_org[`DADDR_WIDTH-1 : 4], 3'b000};
    assign dram_addr_column_offset = dram_addr_org[3:1];

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

    // prefetcher
    m_prefetcher #(
      .APP_ADDR_WIDTH(APP_ADDR_WIDTH),
      .APP_ADDR_WIDTH(APP_DATA_WIDTH),
      .INDEX_WIDTH(INDEX_WIDTH)
    ) prefetcher (
      .i_clk(clk),
      .i_rst(rst),
      .i_issuable(prefetch_issuable),
      .o_issuing(prefetch_issuing),
      .o_issue_addr(prefetch_addr),
      .i_notify_read(prev_task == TASK_CACHE_READ),
      .i_notify_write(C_TASK_WRITE_ISSUE),
      .i_notify_addr(dmem_addr_reg),
      .i_notify_data(dmem_din_reg),
      .i_notify_hit(cache_read_hit),
      .o_yield_read(read_get_prefetch_coalesce),
      .i_data_valid(read_prefetch_valid),
      .i_data(dram_dout),
      .i_installable(prefetch_installable),
      .o_installing(prefetch_installing),
      .o_install_addr(prefetch_install_addr),
      .o_install_data(prefetch_install_data)
    );

    assign read_get_valid             = dram_dout_valid && (read_state == READ_GET || read_state == READ_GET_THEN_PREFETCH);
    assign read_prefetch_valid        = dram_dout_valid && (read_state == READ_PREFETCH || read_state == READ_PREFETCH_THEN_GET);

    assign prefetch_issuable    = !dram_busy && !((prev_task == C_TASK_WAIT_CALIB) || C_TASK_READ_ISSUE || C_TASK_WRITE_ISSUE);
    assign prefetch_installable = !(C_TASK_COMPLETE_READ || cache_write_now || C_TASK_WRITE_ISSUE);

    // cache
    assign cache_install = C_TASK_COMPLETE_READ || prefetch_installing;
    assign cache_install_addr = (C_TASK_COMPLETE_READ) ? dmem_addr_reg : prefetch_install_addr;
    assign cache_install_data = (C_TASK_COMPLETE_READ) ? dram_dout_reg : prefetch_install_data;

    m_cache #(
      .INDEX_WIDTH(INDEX_WIDTH)
    ) cache (
      .i_clk(clk),
      .i_addr(dmem_addr),
      .i_we(dmem_wen != 0),
      .o_we(cache_write_now),
      .i_data(dmem_din),
      .o_data(cache_dout),
      .o_rhit(cache_read_hit),
      .o_bindex(cache_bindex),
      .i_ie(cache_install),
      .i_iaddr(cache_install_addr),
      .i_idata(cache_install_data)
    );

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
          .o_rst(rst),
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
    
    wire C_TASK_WAIT_CALIB         = prev_task == TASK_WAIT_CALIB && !dram_init_calib_complete;
    wire w_write_issue_next        = prev_task == TASK_WRITE_THROUGH || prev_task == TASK_WRITE_ISSUE_STALL;
    wire C_TASK_WRITE_ISSUE_STALL  = dram_busy && w_write_issue_next;
    wire w_read_miss               = (prev_task == TASK_CACHE_READ) && !cache_read_hit;
    wire w_read_issue_next         = (w_read_miss && !read_get_prefetch_coalesce) || prev_task == TASK_READ_ISSUE_STALL;
    wire C_TASK_READ_ISSUE_STALL   = dram_busy && w_read_issue_next;
    wire C_TASK_READ_WAIT          = prev_task == TASK_READ_ISSUE || read_get_prefetch_coalesce || (!read_get_valid && prev_task == TASK_READ_WAIT);
    wire C_TASK_READ_ISSUE         = !dram_busy && w_read_issue_next;
    wire C_TASK_WRITE_ISSUE        = !dram_busy && w_write_issue_next; // this bit and C_TASK_WRITE_THROUGH would assert simultaneously
    wire C_TASK_COMPLETE_READ      = prev_task == TASK_SAVE_DRAM_RESULT;
    wire C_TASK_SAVE_DRAM_RESULT   = read_get_valid && prev_task == TASK_READ_WAIT;
    wire w_other                   = !(
         C_TASK_WAIT_CALIB
      || C_TASK_WRITE_ISSUE_STALL
      || C_TASK_READ_ISSUE_STALL
      || C_TASK_READ_WAIT
      || C_TASK_READ_ISSUE
      || C_TASK_COMPLETE_READ
      || C_TASK_SAVE_DRAM_RESULT
    );
    wire C_TASK_CACHE_READ         = w_other && dmem_ren;
    wire C_TASK_WRITE_THROUGH      = w_other && dmem_wen;

    always @(posedge clk) if (rst) begin
      prev_task <= TASK_WAIT_CALIB;
      dmem_wen_reg <= 0;
      dmem_addr_reg <= 0;
      dmem_din_reg <= 0;
      dram_dout_reg <= 0;

      read_state <= 0;
    end else begin
      // task update
           if (C_TASK_WAIT_CALIB         ) prev_task <= TASK_WAIT_CALIB;
      else if (C_TASK_CACHE_READ         ) prev_task <= TASK_CACHE_READ;
      else if (C_TASK_WRITE_THROUGH      ) prev_task <= TASK_WRITE_THROUGH;
      else if (C_TASK_WRITE_ISSUE_STALL  ) prev_task <= TASK_WRITE_ISSUE_STALL;
      else if (C_TASK_READ_ISSUE_STALL   ) prev_task <= TASK_READ_ISSUE_STALL;
      else if (C_TASK_READ_WAIT          ) prev_task <= TASK_READ_WAIT;
      else if (C_TASK_READ_ISSUE         ) prev_task <= TASK_READ_ISSUE;
      // else if (C_TASK_WRITE_ISSUE        ) prev_task <= TASK_WRITE_ISSUE;
      else if (C_TASK_COMPLETE_READ      ) prev_task <= TASK_COMPLETE_READ;
      else if (C_TASK_SAVE_DRAM_RESULT   ) prev_task <= TASK_SAVE_DRAM_RESULT;
      else                                 prev_task <= TASK_IDLE;
      if (C_TASK_CACHE_READ || C_TASK_WRITE_THROUGH) begin
         dmem_addr_reg <= dmem_addr;
         dmem_wen_reg <= dmem_wen;
         dmem_din_reg <= dmem_din;
      end
      if (C_TASK_SAVE_DRAM_RESULT) begin
        dram_dout_reg <= dram_dout;
      end
      // read mediator
      if (dram_dout_valid) begin
        case (read_state)
          default: // READ_GET, READ_PREFETCH
            read_state <= (prefetch_issuing) ? READ_PREFETCH : (C_TASK_READ_ISSUE) ? READ_GET : READ_NONE;
          READ_GET_THEN_PREFETCH:
            read_state <= READ_PREFETCH;
          READ_PREFETCH_THEN_GET:
            read_state <= READ_GET;
        endcase
      end else if (prefetch_issuing) begin
        if (read_state == READ_NONE)
          read_state <= READ_PREFETCH;
        else // READ_GET
          read_state <= READ_GET_THEN_PREFETCH;
      end else if (C_TASK_READ_ISSUE) begin
        if (read_state == READ_NONE)
          read_state <= READ_GET;
        else // READ_PREFETCH
          read_state <= READ_PREFETCH_THEN_GET;
      end else if (read_get_prefetch_coalesce) begin
        // read_state must be READ_PREFETCH
        read_state <= READ_GET;
      end
    end
endmodule

module m_prefetcher #(
          parameter APP_ADDR_WIDTH  = 28,
          parameter APP_DATA_WIDTH  = 128,
          parameter INDEX_WIDTH     = 8
) (
  input  wire                      i_clk,
  input  wire                      i_rst,

  input  wire                      i_issuable,     // read issue on next posedge is permitted 
  output wire                      o_issuing,      // issue address is valid
  output wire [`DADDR]             o_issue_addr,   // issue address (held until the prefething of the address completes)

  input  wire                      i_notify_read,  // read was issued to cache in the previous posedge
  input  wire                      i_notify_write, // write is being issued from processor
  input  wire [`DADDR]             i_notify_addr, // notify address
  input  wire [31:0]               i_notify_data,  // notify data (write)
  input  wire                      i_notify_hit,   // the block of i_notify_raddr exists in cache or not
  output wire                      o_yield_read,   // 

  input  wire                      i_data_valid,   // provided data is valid
  input  wire [APP_DATA_WIDTH-1:0] i_data,         // dram read result

  input  wire                      i_installable,  // cache install on next posedge is permitted
  output wire                      o_installing,   // install addr/data is valid
  output wire [`DADDR]             o_install_addr, // install addr
  output wire [APP_DATA_WIDTH-1:0] o_install_data  // install data
);
  localparam TAG_WIDTH   = `DADDR_WIDTH - INDEX_WIDTH - 4;

  localparam TASK_WAIT_NEXT_ISSUE = 3'b000;
  localparam TASK_ISSUE           = 3'b001;
  localparam TASK_READ_WAIT       = 3'b011;
  localparam TASK_SAVE_RESULT     = 3'b111;
  localparam TASK_INSTALL         = 3'b100;
  localparam TASK_INSTALL_WAIT    = 3'b101;
  localparam TASK_WAIT_TAG_CHANGE = 3'b110;

  reg [3:0]                prev_task = 0;
  reg                      r_disable_current = 0;
  reg [APP_DATA_WIDTH-1:0] r_data;
  reg [31:0]               r_addr = 0;
  reg [31:0]               r_addr_next = 0;
  reg [TAG_WIDTH-1:0]      r_current_tag = 0;
  reg                      r_tag_changed = 0;
  reg [3:0]                r_mask = 0;
  wire [27:0]              w_notify_addr_norm;
  wire [TAG_WIDTH-1:0]     w_notify_addr_tag;
  wire [1:0]               w_notify_addr_bindex;
  wire [27:0]              w_addr_norm;
  wire [TAG_WIDTH-1:0]     w_addr_tag;
  wire                     w_accessing_prefetchee;
  wire                     w_tag_mismatch;

  assign w_notify_addr_norm   = i_notify_addr[`DADDR_WIDTH-1:4];
  assign w_notify_addr_tag    = i_notify_addr[`DADDR_WIDTH-1 -: TAG_WIDTH];
  assign w_notify_addr_bindex = i_notify_addr[3:2];
  
  assign w_addr_norm = r_addr[`DADDR_WIDTH-1:4];
  assign w_addr_tag  = r_addr[`DADDR_WIDTH-1 -: TAG_WIDTH];
  assign w_accessing_prefetchee = w_notify_addr_norm == w_addr_norm;
  assign w_tag_mismatch         = w_notify_addr_tag != w_addr_tag;
  assign o_yield_read   = i_notify_read && !i_notify_hit && w_accessing_prefetchee && C_TASK_READ_WAIT;

  wire C_TASK_WAIT_TAG_CHANGE = (prev_task == TASK_WAIT_TAG_CHANGE || prev_task == TASK_INSTALL) && (!r_tag_changed && r_current_tag != w_addr_tag);
  wire C_TASK_WAIT_NEXT_ISSUE = !C_TASK_WAIT_TAG_CHANGE && (prev_task == TASK_WAIT_TAG_CHANGE || prev_task == TASK_WAIT_NEXT_ISSUE || prev_task == TASK_INSTALL) && !i_issuable;
  wire C_TASK_ISSUE           = !C_TASK_WAIT_TAG_CHANGE && (prev_task == TASK_WAIT_TAG_CHANGE || prev_task == TASK_WAIT_NEXT_ISSUE || prev_task == TASK_INSTALL) && i_issuable;
  wire C_TASK_READ_WAIT       = (prev_task == TASK_ISSUE || prev_task == TASK_READ_WAIT) && !i_data_valid;
  wire C_TASK_SAVE_RESULT     = (prev_task == TASK_READ_WAIT) && i_data_valid;
  wire C_TASK_INSTALL         = (prev_task == TASK_SAVE_RESULT || prev_task == TASK_INSTALL_WAIT) && i_installable;
  wire C_TASK_INSTALL_WAIT    = (prev_task == TASK_SAVE_RESULT || prev_task == TASK_INSTALL_WAIT) && !i_installable;

  wire w_end_of_fetch         = C_TASK_INSTALL || o_yield_read;

  assign o_issuing      = C_TASK_ISSUE;
  assign o_issue_addr   = r_addr;
  assign o_installing   = C_TASK_INSTALL && !r_disable_current;
  assign o_install_addr = r_addr;
  assign o_install_data = r_data;

  integer i;

  always @(posedge i_clk) if (i_rst) begin
    prev_task <= TASK_WAIT_NEXT_ISSUE;
    r_data <= 0;
    r_addr <= 0;
    r_addr_next <= 32'b10000;
    r_current_tag <= 0;
    r_tag_changed = 0;
    r_mask <= 0;
  end else begin
         if (o_yield_read)           prev_task <= TASK_INSTALL;
    else if (C_TASK_WAIT_NEXT_ISSUE) prev_task <= TASK_WAIT_NEXT_ISSUE;
    else if (C_TASK_ISSUE          ) prev_task <= TASK_ISSUE;
    else if (C_TASK_READ_WAIT      ) prev_task <= TASK_READ_WAIT;
    else if (C_TASK_SAVE_RESULT    ) prev_task <= TASK_SAVE_RESULT;
    else if (C_TASK_INSTALL        ) prev_task <= TASK_INSTALL;
    else if (C_TASK_INSTALL_WAIT   ) prev_task <= TASK_INSTALL_WAIT;
    else if (C_TASK_WAIT_TAG_CHANGE) prev_task <= TASK_WAIT_TAG_CHANGE;

    if (C_TASK_SAVE_RESULT) begin
`ifdef SAVE_SW_MISS
      for (i = 0; i < 4; i = i + 1) begin
        if (!r_mask[i]) r_data[i*32 +: 32] <= i_data[i*32 +: 32];
      end
`else
      r_data <= i_data;
`endif
    end

`ifdef SAVE_SW_MISS
    if (i_notify_write && w_accessing_prefetchee) begin
      r_data[w_notify_addr_bindex*32 +: 32] <= i_notify_data;
      r_mask[w_notify_addr_bindex]          <= 1;
    end else if (w_end_of_fetch) begin
      r_mask      <= 0;
    end
`else
    if (i_notify_write && w_accessing_prefetchee) begin
      r_disable_current <= 1;
    end
`endif

    if (w_end_of_fetch) begin
      r_addr      <= r_addr_next;
    end

    if (w_end_of_fetch) begin
      r_addr_next <= r_addr_next + 5'b10000;
      r_disable_current <= 0;
    end else if (i_notify_read && w_tag_mismatch) begin
      r_disable_current <= 1;
      r_addr_next <= i_notify_addr + 5'b10000;
    end

    if (i_notify_read || i_notify_write) begin
      r_current_tag <= w_notify_addr_tag;
      r_tag_changed <= w_notify_addr_tag != r_current_tag;
    end

  end

endmodule