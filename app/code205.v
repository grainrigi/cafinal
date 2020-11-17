/**************************************************************************/
/* code205.v       For CSC.T363 Computer Architecture, Archlab TOKYO TECH */
/**************************************************************************/
`timescale 1ns/100ps
`default_nettype none

/**************************************************************************/
`define HALT {6'h4, 5'd0, 5'd0, 16'hffff} // L1: beq  $0, $0, L1
/**************************************************************************/

/***** top module for simulation *****/

module m_top (); 
   reg r_clk=0; initial forever #50 r_clk = ~r_clk;
   wire [31:0] w_led;

   m_proc07 p (r_clk, w_led);

   integer fp;
   integer i;
   initial begin
      fp = $fopen("memory_image.bin", "wb");
      for(i=0; i<512; i=i+1) begin
         $fwrite(fp, "%u", p.m_imem.cm_ram[i]);
      end
      $fclose(fp);

      $write("memory_image.bin was generated.\n");

      $finish();
   end
endmodule

/*
module m_top (); 
   reg r_clk=0; initial forever #50 r_clk = ~r_clk;
   wire [31:0] w_led;

   reg [31:0] r_cnt = 0;
   always@(posedge r_clk)  r_cnt <= r_cnt + 1;
    
   m_proc07 p (r_clk, w_led);

   initial $write("clock : r_pc     w_ir     w_rrs    w_rrt   w_rrt2   r_rslt2  r_led\n");
   always@(posedge r_clk) begin
       $write("%6d: %x %x (%d %d)%x %x %x %x %x :%x %x %x ", r_cnt,
              p.r_pc, p.w_ir, p.w_rs, p.w_rt, 
              p.w_rrs, p.w_rrt, p.w_rrt2, p.w_rslt2, w_led, p.m_regs.r[20], p.m_regs.r[21],p.m_regs.r[22]);
       if(p.w_insn_add)  $write("add");
       if(p.w_insn_sllv) $write("sllv");
       if(p.w_insn_srlv) $write("srlv");
       if(p.w_insn_addi) $write("addi");
       if(p.w_insn_lw)   $write("lw");
       if(p.w_insn_sw)   $write("sw");
       if(p.w_insn_beq)  $write("beq");
       if(p.w_insn_bne)  $write("bne");
       $write("\n");
   end

   always@(posedge r_clk) if(p.w_ir==`HALT) $finish();
endmodule
*/

/**************************************************************************/
module m_amemory (w_clk, w_addr, w_we, w_din, w_dout);
   input  wire w_clk, w_we;
   input  wire [10:0] w_addr;
   input  wire [31:0] w_din;
   output wire [31:0] w_dout;
   reg [31:0] 	      cm_ram [0:2047]; // 4K word (2048 x 32bit) memory
   always @(posedge w_clk) if (w_we) cm_ram[w_addr] <= w_din;
   assign #20 w_dout = cm_ram[w_addr];

   integer i; initial for(i=0; i<100; i=i+1) cm_ram[i]=0; /* init by zero */
`include "program.txt"
endmodule

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
   
   initial r[1] = 1;
   initial r[2] = 2;
endmodule

module m_proc07 (w_clk, r_led);
   input  wire w_clk;
   output reg [31:0] r_led;
   initial r_led = 0;
   
   reg  [31:0] r_pc = 0;
   wire [31:0] w_ir, w_rrs, w_rrt;
   wire [31:0] w_rrt2, w_rslt, w_ldd, w_rslt2;
   wire  [5:0] w_op    = w_ir[31:26];
   wire  [4:0] w_rs    = w_ir[25:21];
   wire  [4:0] w_rt    = w_ir[20:16];
   wire  [4:0] w_rd    = w_ir[15:11];
   wire [15:0] w_imm   = w_ir[15: 0];
   wire  [5:0] w_funct = w_ir[ 5: 0];

   wire w_insn_add  = (w_op==0 && w_funct==6'h20);
   wire w_insn_sllv = (w_op==0 && w_funct==6'h4);
   wire w_insn_srlv = (w_op==0 && w_funct==6'h6);
   wire w_insn_addi = (w_op==6'h8);
   wire w_insn_lw   = (w_op==6'h23);
   wire w_insn_sw   = (w_op==6'h2b);
   wire w_insn_beq  = (w_op==6'h4);
   wire w_insn_bne  = (w_op==6'h5);
   
   wire [31:0] w_pc4 = r_pc + 4;
   wire [31:0] w_imm32 = {{16{w_imm[15]}}, w_imm};
   wire [31:0] w_tpc = w_pc4 + {w_imm32[29:0], 2'h0};
   wire w_taken = (w_insn_beq && w_rrs==w_rrt2) || (w_insn_bne && w_rrs!=w_rrt2);
   always @(posedge w_clk) r_pc <= #3 (w_taken) ? w_tpc : w_pc4;
   
   m_amemory m_imem (w_clk, r_pc[12:2], 1'd0, 32'd0, w_ir);

   wire  [4:0] w_rd2 = (w_insn_add | w_insn_sllv | w_insn_srlv) ? w_rd : w_rt;
   wire w_we = w_insn_add | w_insn_addi | w_insn_sllv | w_insn_srlv | w_insn_lw;

   m_regfile m_regs (w_clk, w_rs, w_rt, w_rd2, w_we, w_rslt2, w_rrs, w_rrt);

   assign w_rrt2 = (w_insn_addi | w_insn_lw | w_insn_sw) ? w_imm32 : w_rrt;

   assign #10 w_rslt = (w_insn_sllv) ? w_rrs << w_rrt2[4:0] :
		       (w_insn_srlv) ? w_rrs >> w_rrt2[4:0] : w_rrs + w_rrt2;

   m_amemory m_dmem (w_clk, w_rslt[12:2], w_insn_sw, w_rrt, w_ldd);
   
   assign w_rslt2 = (w_insn_lw) ? w_ldd : w_rslt;

   always @(posedge w_clk) r_led <= (w_we && w_rd2==30) ? w_rslt2 : r_led;
endmodule

