`timescale 1ns/100ps
`default_nettype none
`define HALT {6'h4, 5'd0, 5'd0, 16'hffff} // L1: beq  $0, $0, L1

`include "mipscore.v"

`define DRAM_STATE_IDLE 0
`define DRAM_STATE_READWAIT 1

`ifdef NODELAY
module m_pseudo_dram(
  input wire i_clk,
  input wire [`ADDR] i_addr,
  input wire [31:0] i_data,
  output wire [31:0] o_data,
  input wire [3:0] i_we,
  input wire i_oe,
  output wire o_stall
);
  reg [31:0] r_read;
  wire [31:0] w_read;
  assign o_data = r_read;
  always@(posedge i_clk) if (i_oe) r_read <= w_read;
  m_amemory_d_clean m_ram (
    .w_clk(i_clk),
    .w_raddr(i_addr[12:2]),
    .w_waddr(i_addr[12:2]),
    .w_we(i_we != 0),
    .w_din(i_data),
    .w_dout(w_read)
  );
  assign o_stall = 0;
endmodule
`else
module m_pseudo_dram(
  input wire i_clk,
  input wire [`ADDR] i_addr,
  input wire [31:0] i_data,
  output wire [31:0] o_data,
  input wire [3:0] i_we,
  input wire i_oe,
  output wire o_stall
);
  // read delay
  reg         r_state = `DRAM_STATE_IDLE;
  reg [3:0]   r_wait = 0;
  reg [10:0] r_addr = 0;
  reg [31:0]  r_read = 0;

  always @(posedge i_clk) begin
    case (r_state)
      `DRAM_STATE_IDLE: begin
        if (i_oe) begin
          r_addr <= i_addr[12:2];
          r_wait <= 15;
          r_state <= `DRAM_STATE_READWAIT;
        end
      end
      `DRAM_STATE_READWAIT: begin
        if (r_wait > 0) r_wait <= r_wait - 1;
        else begin
          r_read <= w_read;
          r_state <= `DRAM_STATE_IDLE;
        end
      end
    endcase
  end

  // in/out
  assign o_data = r_read;
  assign o_stall = r_state != `DRAM_STATE_IDLE;

  // memory
  wire [31:0]  w_read;

  m_amemory_d_clean m_ram (
    .w_clk(i_clk),
    .w_raddr(r_addr),
    .w_waddr(i_addr[12:2]),
    .w_we(i_we != 0),
    .w_din(i_data),
    .w_dout(w_read)
  );
endmodule
`endif

module m_amemory_d_clean (w_clk, w_raddr, w_waddr, w_we, w_din, w_dout);
   input  wire w_clk, w_we;
   input  wire [10:0] w_raddr, w_waddr; // read address & write address
   input  wire [31:0] w_din;
   output wire [31:0] w_dout;
   reg [31:0] cm_ram [0:2047]; // 2048 word (2048 x 32bit) memory
   always @(posedge w_clk) if (w_we) cm_ram[w_waddr] <= w_din; // write port
   assign w_dout = cm_ram[w_raddr];                            // read  port
endmodule

module m_amemory_d (w_clk, w_raddr, w_waddr, w_we, w_din, w_dout);
   input  wire w_clk, w_we;
   input  wire [ 8:0] w_raddr, w_waddr; // read address & write address
   input  wire [31:0] w_din;
   output wire [31:0] w_dout;
   reg [31:0] cm_ram [0:511]; // 512 word (512 x 32bit) memory
   always @(posedge w_clk) if (w_we) cm_ram[w_waddr] <= w_din; // write port
   assign w_dout = cm_ram[w_raddr];                            // read  port

   integer i; initial for(i=0; i<100; i=i+1) cm_ram[i]=0; /* init by zero */
`include "app/program_contest.txt"
endmodule

module m_top (); 
   reg r_clk=0; initial forever #50 r_clk = ~r_clk;
   reg r_rst=0;
   wire [`ADDR] w_pc, w_daddr;
   wire [31:0] w_ir, w_rdata, w_wdata;
   wire [3:0] w_we;
   wire w_oe;
   wire w_stall;

`ifndef CONTEST_V
   initial $dumpfile("main.vcd");
   initial $dumpvars(0, m_top);
`endif
   initial r_rst <= #200 1;

   m_amemory_d m_imem (
     .w_clk(r_clk),
     .w_raddr(w_pc[10:2]),
     .w_waddr(w_pc[10:2]),
     .w_we(1'b0),
     .w_din(32'd0),
     .w_dout(w_ir)
   );
   m_pseudo_dram m_dmem (
     .i_clk(r_clk),
     .i_addr(w_daddr),
     .i_data(w_wdata),
     .o_data(w_rdata),
     .i_we(w_we),
     .i_oe(w_oe),
     .o_stall(w_stall)
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
  always@(posedge r_clk) if(p.MeWb_rd2 != 0 && p.WbRSLT != 0) $write("%x\n", p.WbRSLT);
  // always@(posedge r_clk) if(p.MeWb_rd2 != 0 && p.WbRSLT != 0) $write("%x %x %2x %x\n", r_counter, p.MeWb_pc, p.MeWb_rd2, p.WbRSLT);
`else
   initial $write("time: r_pc     w_ir     w_rrs    w_rrt2   r_rslt2  r_led\n");
   always@(posedge r_clk) $write("%4d: %x %x %x %x %x %x\n", $time / 100,
                         // w_pc, w_ir, p.Id1RRS, p.Id1RRT, p.WbRSLT, p.MeWb_rd2);
                         p.pc, p.IfId_ir, p.IdRRS, p.IdRRT, p.MaWb_rslt, p.MaWb_dst);
`endif
   always@(posedge r_clk) if(w_ir==`HALT) $finish();
endmodule