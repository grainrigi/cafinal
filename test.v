`timescale 1ns/100ps
`default_nettype none
`define HALT {6'h4, 5'd0, 5'd0, 16'hffff} // L1: beq  $0, $0, L1

`include "cached_memory.v"

`include "mipscore2.v"

`define DRAM_STATE_IDLE 0
`define DRAM_STATE_READWAIT 1

module m_memory_d (w_clk, w_raddr, w_waddr, w_we, w_din, r_dout);
   input  wire w_clk, w_we;
   input  wire [`IADDR] w_raddr, w_waddr; // read address & write address
   input  wire [31:0]   w_din;
   output reg  [31:0]   r_dout;
   reg [31:0] cm_ram [0:(2**`IADDR_WIDTH)-1]; // 512 word (512 x 32bit) memory
   always @(posedge w_clk) if (w_we) cm_ram[w_waddr] <= w_din; // write port
   always @(posedge w_clk) r_dout <= cm_ram[w_raddr];          // read  port

   integer i; initial for(i=0; i<100; i=i+1) cm_ram[i]=0; /* init by zero */
`include "app/program_c2.txt"
endmodule

module m_top #(
              parameter DDR3_DQ_WIDTH   = 16,
              parameter DDR3_DQS_WIDTH  = 2,
              parameter DDR3_ADDR_WIDTH = 14,
              parameter DDR3_BA_WIDTH   = 3,
              parameter DDR3_DM_WIDTH   = 2,
              parameter APP_ADDR_WIDTH  = 28,
              parameter APP_CMD_WIDTH   = 3,
              parameter APP_DATA_WIDTH  = 128,
              parameter APP_MASK_WIDTH  = 16)
 (); 
   reg r_clk=0; initial forever #50 r_clk = ~r_clk;
   reg r_rst=0;
   wire [`HADDR] w_pc;
   wire [`EADDR] w_daddr;
   wire [31:0] w_ir, w_rdata, w_wdata;
   wire [3:0] w_we;
   wire w_oe;
   wire w_stall;

`ifndef CONTEST_V
   initial $dumpfile("main.vcd");
   initial $dumpvars(0, m_top);
`endif
   initial r_rst <= #200 1;

   m_memory_d m_imem (
     .w_clk(r_clk),
     .w_raddr(w_pc[`ISLICE]),
     .w_waddr(w_pc[`ISLICE]),
     .w_we(1'b0),
     .w_din(32'd0),
     .r_dout(w_ir)
   );

   MIPSCORE p (
     .CLK(r_clk),
     .RST_X(r_rst),
     .STALL(w_stall),
     .I_ADDR(w_pc),
     .I_IN(w_ir),
     .D_ADDR(w_daddr),
     .D_IN(w_rdata),
     .D_OUT(w_wdata),
     .D_OE(w_oe),
     .D_WE(w_we)
   );
  reg [31:0] r_counter = 0;
  always@(posedge r_clk) r_counter <= r_counter + 1;
`ifdef CONTEST_V
  always@(posedge r_clk) #50 if(!p.Wb_STALL && p.MeWb_rd2 != 0) $write("%x\n", p.WbRSLT);
`elsif CONTEST_VD
  always@(posedge r_clk) #50 if(!p.Wb_STALL && p.MeWb_rd2 != 0) $write("%x %x %2x %x\n", r_counter, p.MeWb_pc, p.MeWb_rd2, p.WbRSLT);
`else
   initial $write("time: r_pc     w_ir     w_rrs    w_rrt2   r_rslt2  r_led\n");
   always@(posedge r_clk) $write("%4d: %x %x %x %x %x %x\n", $time / 100,
                         w_pc, w_ir, p.Id1RRS, p.Id1RRT, p.WbRSLT, p.MeWb_rd2);
                         // p.pc, p.IfId_ir, p.IdRRS, p.IdRRT, p.MaWb_rslt, p.MaWb_dst);
`endif
   always@(posedge r_clk) if(w_ir==`HALT) $finish();

   // dram
   wire          dmem_init_done;
   wire [3:0]    dmem_init_wen;
   wire [`EADDR] dmem_init_addr;
   wire [31:0]   dmem_init_din;
   wire          dmem_ren;
   wire [3:0]    dmem_wen;
   wire [31:0]   dmem_addr;
   wire [31:0]   dmem_din;
   wire [31:0]   dmem_dout;
   wire          dmem_stall;

   assign dmem_wen = w_we;
   assign dmem_addr = { w_daddr[31:2], 2'b00 };
   assign dmem_din = w_wdata;
   assign dmem_ren = w_oe;
   assign w_rdata  = dmem_dout;
   assign w_stall  = dmem_stall;
   assign dmem_init_done = 1; //initdone;
   assign dmem_init_wen  = 0; //{4{initwe}};
   assign dmem_init_addr = 0; //initaddr;
   assign dmem_init_din  = 0; //initdata;

   m_cached_memory #(
                .APP_ADDR_WIDTH(APP_ADDR_WIDTH),
                .APP_CMD_WIDTH(APP_CMD_WIDTH),
                .APP_DATA_WIDTH(APP_DATA_WIDTH),
                .APP_MASK_WIDTH(APP_MASK_WIDTH))
   dmem (
         .i_clk(r_clk),
         // user design interface signals
         .i_dmem_init_done(dmem_init_done),
         .i_dmem_init_wen(dmem_init_wen),
         .i_dmem_init_addr(dmem_init_addr[`DSLICE]),
         .i_dmem_init_data(dmem_init_din),
         .i_dmem_ren(dmem_ren),
         .i_dmem_wen(dmem_wen),
         .i_dmem_addr(dmem_addr[`DSLICE]),
         .i_dmem_data(dmem_din),
         .o_dmem_data(dmem_dout),
         .o_dmem_stall(dmem_stall));
endmodule