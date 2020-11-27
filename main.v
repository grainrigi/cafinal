/********************************************************************************************/
/* Sample Verilog HDL Code for CSC.T363 Computer Architecture          Arch Lab. TOKYO TECH */
/* This code is based on the code from https://github.com/thiemchu/dram-arty-a7             */
/********************************************************************************************/
`include "config.vh"
/********************************************************************************************/
`timescale 1ns/100ps
`default_nettype none
`define HALT {6'h4, 5'd0, 5'd0, 16'hffff} // L1: beq  $0, $0, L1

/********************************************************************************************/
`define RX_COUNT 49
module m_UartRx (w_clk, w_rxd, w_dout, r_en);
   input  wire       w_clk, w_rxd;
   output wire [7:0] w_dout;
   output reg        r_en = 0;

   reg [2:0] r_detect_cnt = 0; /* to detect the start bit */
   always @(posedge w_clk) r_detect_cnt <= (w_rxd) ? 0 : r_detect_cnt + 1;
   wire w_detected = (r_detect_cnt>2);

   reg       r_busy = 0;
   reg [3:0] r_cnt  = 0;
   reg [7:0] r_wait = 0;
   always@(posedge w_clk) r_wait <= (r_busy==0) ? 0 : (r_wait>=`RX_COUNT) ? 0 : r_wait + 1;

   reg [8:0] r_data = 0;
   always@(posedge w_clk) begin
      if (r_busy==0) begin
         {r_data, r_cnt, r_en} <= 0;
         if(w_detected) r_busy <= 1;
      end
      else if (r_wait>= `RX_COUNT) begin
         r_cnt  <= r_cnt + 1;
         r_data <= {w_rxd, r_data[8:1]};
         if (r_cnt==8) begin r_en <= 1; r_busy <= 0; end
      end
   end
   assign w_dout = r_data[7:0];
endmodule

/********************************************************************************************/
module m_memory_d (w_clk, w_raddr, w_waddr, w_we, w_din, r_dout);
   input  wire w_clk, w_we;
   input  wire [ 8:0] w_raddr, w_waddr; // read address & write address
   input  wire [31:0] w_din;
   output wire [31:0] r_dout;
   reg [31:0] cm_ram [0:511]; // 512 word (512 x 32bit) memory
   always @(posedge w_clk) if (w_we) cm_ram[w_waddr] <= w_din; // write port
   always @(posedge w_clk) r_dout <= cm_ram[w_raddr];                            // read  port
endmodule

/********************************************************************************************/
module main #(
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
    // input clock (100MHz), reset (active-low) ports
    input  wire                         clk_in,
    input  wire                         rstx_in,
    // dram interface ports
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
    input  wire                         uart_rxd,
    output wire                         uart_txd
    );

   wire        clk;            // system clock
   wire        rst;            // 
   wire        clk_166_67_mhz; //
   wire        clk_200_mhz;    // 
   wire        locked;         // clk_wiz locked
   wire [31:0] I_DATA, I_ADDR, D_IN, D_OUT, D_ADDR;
   wire [3:0]  D_WE;
   wire        D_OE, D_STALL;
   assign uart_txd = 1;
   
   /****************************************************************************************/   
   clk_wiz_1 dram_clkgen (.clk_in1(clk_in), .resetn(rstx_in), .clk_out1(clk_166_67_mhz), 
			  .clk_out2(clk_200_mhz), .locked(locked));

   /***** instruction memory and program loader *****/
   /****************************************************************************************/
   wire [7:0] w_uartd; // uart data
   wire w_en;          // uart data enable
   m_UartRx m_UartRx0(clk, uart_rxd, w_uartd, w_en);

   reg        initdone = 0;
   reg [2:0]  r_cnt  = 0;
   reg [31:0] r_wcnt = 0; // word counter
   reg [31:0] r_data = 0;
   always @(posedge clk) if(w_en) r_data <= {w_uartd, r_data[31:8]};
   always @(posedge clk) r_cnt <= (r_cnt==4) ? 0 : (w_en) ? r_cnt + 1 : r_cnt;
   always @(posedge clk) if(r_cnt==4) r_wcnt <= r_wcnt + 1;
   always @(posedge clk) if(r_wcnt==512) initdone <= 1;

   wire [31:0] I_IN;
   m_memory_d m_imem (clk, I_ADDR[10:2], r_wcnt, (!initdone & r_cnt==4), r_data, I_IN);
   /****************************************************************************************/
   reg r_rstx = 0; // reset_x signal for processor core
   always @(posedge clk) r_rstx <= (!rst & initdone);

//   m_proc09 p(.CLK(clk), .RST_X(initdone), .STALL(D_STALL),
   MIPSCORE p(.CLK(clk), .RST_X(initdone), .STALL(D_STALL), 
              .I_ADDR(I_ADDR), .I_IN(I_IN),
              .D_ADDR(D_ADDR), .D_IN(D_IN), .D_OE(D_OE), .D_WE(D_WE), .D_OUT(D_OUT));

   reg r_halt = 0;
   always @(posedge clk) if(I_IN==`HALT) r_halt <= 1;

   reg [31:0] r_rout = 0;
   always @(posedge clk) if(D_ADDR==0 & D_WE) r_rout <= D_OUT;
             
   reg [31:0] r_clk_cnt = 0; // counter to measure the elapsed clock cycles
   always @(posedge clk) if(r_rstx & ~r_halt) r_clk_cnt <= r_clk_cnt + 1;
   vio_0 vio_00(.clk(clk), .probe_in0(I_ADDR), .probe_in1(r_rout), .probe_in2(r_clk_cnt));
   
   /***** DRAM *****/
   /****************************************************************************************/   
   reg dram_rst_sync1;
   reg dram_rst_sync2;

   wire        dmem_init_done;
   wire [3:0]  dmem_init_wen;
   wire [31:0] dmem_init_addr;
   wire [31:0] dmem_init_din;
   wire        dmem_ren;
   wire [3:0]  dmem_wen;
   wire [31:0] dmem_addr;
   wire [31:0] dmem_din;
   wire [31:0] dmem_dout;
   wire        dmem_stall;
   wire        dram_rstx_async = rstx_in & locked;
   wire        dram_rst = dram_rst_sync2;

   always @(posedge clk_166_67_mhz or negedge dram_rstx_async) begin
      if (!dram_rstx_async) begin
         dram_rst_sync1 <= 1'b1;
         dram_rst_sync2 <= 1'b1;
      end else begin
         dram_rst_sync1 <= 1'b0;
         dram_rst_sync2 <= dram_rst_sync1;
      end
   end

   assign dmem_wen = D_WE;
   assign dmem_addr = {D_ADDR[31:2], 2'b00};  // dmem_addr must be 4-byte aligned
   assign dmem_din = D_OUT;
   assign dmem_ren = D_OE;
   assign D_IN    = dmem_dout;
   assign D_STALL = dmem_stall;
   assign dmem_init_done = 1; //initdone;
   assign dmem_init_wen  = 0; //{4{initwe}};
   assign dmem_init_addr = 0; //initaddr;
   assign dmem_init_din  = 0; //initdata;

   DataMemory #(
                .DDR3_DQ_WIDTH(DDR3_DQ_WIDTH),
                .DDR3_DQS_WIDTH(DDR3_DQS_WIDTH),
                .DDR3_ADDR_WIDTH(DDR3_ADDR_WIDTH),
                .DDR3_BA_WIDTH(DDR3_BA_WIDTH),
                .DDR3_DM_WIDTH(DDR3_DM_WIDTH),
                .APP_ADDR_WIDTH(APP_ADDR_WIDTH),
                .APP_CMD_WIDTH(APP_CMD_WIDTH),
                .APP_DATA_WIDTH(APP_DATA_WIDTH),
                .APP_MASK_WIDTH(APP_MASK_WIDTH))
   dmem (
         // input clock (166.67MHz),
         // reference clock (200MHz),
         // reset (active-high)
         .sys_clk(clk_166_67_mhz),
         .ref_clk(clk_200_mhz),
         .sys_rst(dram_rst),
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
         .o_clk(clk),    ///// 83.333MHz -> 40MHz 
         .o_rst(rst),
         // user design interface signals
         .i_dmem_init_done(dmem_init_done),
         .i_dmem_init_wen(dmem_init_wen),
         .i_dmem_init_addr(dmem_init_addr),
         .i_dmem_init_data(dmem_init_din),
         .i_dmem_ren(dmem_ren),
         .i_dmem_wen(dmem_wen),
         .i_dmem_addr(dmem_addr),
         .i_dmem_data(dmem_din),
         .o_dmem_data(dmem_dout),
         .o_dmem_stall(dmem_stall));
endmodule
/********************************************************************************************/
