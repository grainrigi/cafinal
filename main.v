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

/************************************************************************************/
module m_regfile (w_clk, w_rr1, w_rr2, w_wr, w_we, w_wdata, w_rdata1, w_rdata2);
   input  wire        w_clk;
   input  wire [4:0]  w_rr1, w_rr2, w_wr;
   input  wire [31:0] w_wdata;
   input  wire        w_we;
   output wire [31:0] w_rdata1, w_rdata2;
   
   reg [31:0] r[0:31];
   assign w_rdata1 = (w_rr1==0) ? 0 : r[w_rr1];
   assign w_rdata2 = (w_rr2==0) ? 0 : r[w_rr2];
   always @(posedge w_clk) if(w_we) r[w_wr] <= w_wdata;
endmodule

/********************************************************************************************/
module m_amemory_d (w_clk, w_raddr, w_waddr, w_we, w_din, w_dout);
   input  wire w_clk, w_we;
   input  wire [ 8:0] w_raddr, w_waddr; // read address & write address
   input  wire [31:0] w_din;
   output wire [31:0] w_dout;
   reg [31:0] cm_ram [0:511]; // 512 word (512 x 32bit) memory
   always @(posedge w_clk) if (w_we) cm_ram[w_waddr] <= w_din; // write port
   assign w_dout = cm_ram[w_raddr];                            // read  port
endmodule

/********************************************************************************************/
module m_proc09 (CLK, RST_X, STALL, I_ADDR, I_IN, D_ADDR, D_IN, D_OUT, D_OE, D_WE, IO_IN);
   input  wire CLK, RST_X, STALL;
   input  wire [31:0] I_IN;
   output wire [31:0] I_ADDR;
   output wire [31:0] D_ADDR;
   output wire  [3:0] D_WE;
   output wire        D_OE;
   output wire [31:0] D_OUT;
   input  wire [31:0] D_IN, IO_IN;

   wire w_clk = CLK;
   wire w_stall = STALL;

   reg  [31:0] r_pc = 0;
   wire [31:0] w_ir = I_IN;
   wire [31:0] w_rrs, w_rrt;
   wire [31:0] w_rrt2, w_rslt, w_ldd, w_rslt2;
   wire  [5:0] w_op    = w_ir[31:26];
   wire  [4:0] w_rs    = w_ir[25:21];
   wire  [4:0] w_rt    = w_ir[20:16];
   wire  [4:0] w_rd    = w_ir[15:11];
   wire [15:0] w_imm   = w_ir[15: 0];
   wire  [5:0] w_funct = w_ir[ 5: 0];

   assign I_ADDR  = r_pc;
   assign D_ADDR  = w_rslt;
   assign D_WE    = (w_insn_sw & (state==0)) ? 4'b1111 : 0; // write enable for store insn
   assign D_OUT   = w_rrt;                                  // write data   for store insn
   assign D_OE    = w_insn_lw & (state==0);                 // read enable  for load  insn
   
   wire w_insn_add  = (w_op==0 && w_funct==6'h20);
   wire w_insn_sllv = (w_op==0 && w_funct==6'h4);
   wire w_insn_srlv = (w_op==0 && w_funct==6'h6);
   wire w_insn_addi = (w_op==6'h8);
   wire w_insn_lw   = (w_op==6'h23);
   wire w_insn_sw   = (w_op==6'h2b);
   wire w_insn_beq  = (w_op==6'h4);
   wire w_insn_bne  = (w_op==6'h5);

   reg state = 0;
   always @(posedge w_clk) state <= (!RST_X) ? 0 : (!w_stall) ? state + 1 : state;
   
   wire [31:0] w_pc4 = r_pc + 4;
   wire [31:0] w_imm32 = {{16{w_imm[15]}}, w_imm};
   wire [31:0] w_tpc = w_pc4 + {w_imm32[29:0], 2'h0};
   wire w_taken = (w_insn_beq && w_rrs==w_rrt2) || (w_insn_bne && w_rrs!=w_rrt2);

   always @(posedge w_clk) 
     r_pc <= (!RST_X) ? 0 : (w_stall || state==0) ? r_pc : (w_taken) ? w_tpc : w_pc4;
   
   wire  [4:0] w_rd2 = (w_insn_add | w_insn_sllv | w_insn_srlv) ? w_rd : w_rt;
   wire w_we = w_insn_add | w_insn_addi | w_insn_sllv | w_insn_srlv | w_insn_lw;

   m_regfile m_regs (w_clk, w_rs, w_rt, w_rd2, w_we && (w_stall==0) && state, 
                     w_rslt2, w_rrs, w_rrt);

   assign w_rrt2 = (w_insn_addi | w_insn_lw | w_insn_sw) ? w_imm32 : w_rrt;

   assign w_rslt = (w_insn_sllv) ? w_rrs << w_rrt2[4:0] :
		   (w_insn_srlv) ? w_rrs >> w_rrt2[4:0] : w_rrs + w_rrt2;

   assign w_rslt2 = (w_insn_lw) ? D_IN : w_rslt;
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
   m_amemory_d m_imem (clk, I_ADDR[10:2], r_wcnt, (!initdone & r_cnt==4), r_data, I_IN);
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