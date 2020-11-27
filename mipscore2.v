`include "define.v"

`include "predictor.v"

`define OP_NOBRANCH 6'b111011
`define ADD 6'h20
`define ADDI 6'h8
`define BEQ 6'h4
`define BNE 6'h5
`define SLLV 6'h4
`define SRLV 6'h6
`define LW 6'h23
`define SW 6'h2b

module MIPSCORE2 (
  input wire CLK,
  input wire RST_X,
  input wire STALL,
  output wire [`ADDR] I_ADDR,
  input wire [31:0] I_IN,
  output wire [`ADDR] D_ADDR,
  input wire [31:0] D_IN,
  output wire [31:0] D_OUT,
  output wire D_OE,
  output wire [3:0] D_WE
);
  wire WBEXFW_INTERLOCK; // パイプラインをインターロック(停止)するかどうか (IF,ID,EXが停止、MEM,WBは稼働する)

  wire If_STALL  = STALL || WBEXFW_INTERLOCK;
  wire Id_STALL  = STALL || WBEXFW_INTERLOCK;
  wire Id1_STALL = STALL || WBEXFW_INTERLOCK;
  wire Id2_STALL = STALL || WBEXFW_INTERLOCK;
  wire Ex_STALL  = STALL || WBEXFW_INTERLOCK;
  wire Me_STALL  = STALL;
  wire Wb_STALL  = STALL;

  reg [`ADDR] IfId_pc;         // IF-ID pipeline reg: program counter
  reg [`ADDR] IfId_pc4;        // IF-ID pipeline reg: program counter + 4
  reg         IfId_pr;         // IF-ID pipeline reg: is branch predicted (may invalidated on prediction failure)
  reg [31:0]  IfId_ir;         // IF-ID pipeline reg: instruction

  reg [`ADDR] IdId1_pc;        // ID-ID1 pipeline reg: program counter
  reg [`ADDR] IdId1_pc4;       // ID-ID1 pipeline reg: program counter + 4
  reg         IdId1_pr;        // ID-ID1 pipeline reg: is branch predicted (may invalidated on prediction failure)
  reg         IdId1_add;       // ID-ID1 pipeline reg: add
  reg         IdId1_addi;      // ID-ID1 pipeline reg: addi
`ifdef ENABLE_SHIFT
  reg         IdId1_sllv;      // ID-ID1 pipeline reg: sllv
  reg         IdId1_srlv;      // ID-ID1 pipeline reg: srlv
`endif
  reg         IdId1_beq;       // ID-ID1 pipeline reg: beq
  reg         IdId1_bne;       // ID-ID1 pipeline reg: bne
  reg         IdId1_sw;        // ID-ID1 pipeline reg: sw
  reg         IdId1_lw;        // ID-ID1 pipeline reg: lw
  reg [4:0]   IdId1_rs;        // ID-ID1 pipeline reg: RS
  reg [4:0]   IdId1_rt;        // ID-ID1 pipeline reg: RT
  reg [4:0]   IdId1_rd2;       // ID-ID1 pipeline reg: RD
  reg [31:0]  IdId1_imm32;     // ID-ID1 pipeline reg: IMM32
  reg [`ADDR] IdId1_tpc;       // ID-ID1 pipeline reg: taken pc (only valid for beq/bne)

  reg [`ADDR] Id1Id2_pc;       // ID1-ID2 pipeline reg: program counter
  reg [`ADDR] Id1Id2_pc4;      // ID1-ID2 pipeline reg: program counter + 4
  reg         Id1Id2_pr;       // ID1-ID2 pipeline reg: is branch predicted (may invalidated on prediction failure)
  reg         Id1Id2_add;      // ID1-ID2 pipeline reg: add
  reg         Id1Id2_addi;     // ID1-ID2 pipeline reg: addi
`ifdef ENABLE_SHIFT
  reg         Id1Id2_sllv;     // ID1-ID2 pipeline reg: sllv
  reg         Id1Id2_srlv;     // ID1-ID2 pipeline reg: srlv
`endif
  reg         Id1Id2_beq;      // ID1-ID2 pipeline reg: beq
  reg         Id1Id2_bne;      // ID1-ID2 pipeline reg: bne
  reg         Id1Id2_sw;       // ID1-ID2 pipeline reg: sw
  reg         Id1Id2_lw;       // ID1-ID2 pipeline reg: lw
  reg [4:0]   Id1Id2_rs;       // ID1-ID2 pipeline reg: RS
  reg [4:0]   Id1Id2_rt;       // ID1-ID2 pipeline reg: RT
  reg [4:0]   Id1Id2_rd2;      // ID1-ID2 pipeline reg: RD
  reg [31:0]  Id1Id2_imm32;    // ID1-ID2 pipeline reg: IMM32
  reg [`ADDR] Id1Id2_tpc;      // ID1-ID2 pipeline reg: taken pc (only valid for beq/bne)
  reg [31:0]  Id1Id2_rrs;      // ID1-ID2 pipeline reg: value of RS
  reg [31:0]  Id1Id2_rrt;      // ID1-ID2 pipeline reg: value of RT

  reg [`ADDR] Id2Ex_pc;        // ID2-EX pipeline reg: program counter
  reg [`ADDR] Id2Ex_pc4;       // ID2-EX pipeline reg: program counter + 4
  reg         Id2Ex_pr;        // ID2-EX pipeline reg: is branch predicted (may invalidated on prediction failure)
  reg         Id2Ex_add;       // ID2-EX pipeline reg: add
  reg         Id2Ex_addi;      // ID2-EX pipeline reg: addi
`ifdef ENABLE_SHIFT
  reg         Id2Ex_sllv;      // ID2-EX pipeline reg: sllv
  reg         Id2Ex_srlv;      // ID2-EX pipeline reg: srlv
`endif
  reg         Id2Ex_beq;       // ID2-EX pipeline reg: beq
  reg         Id2Ex_bne;       // ID2-EX pipeline reg: bne
  reg         Id2Ex_sw;        // ID2-EX pipeline reg: sw
  reg         Id2Ex_lw;        // ID2-EX pipeline reg: lw
  reg [4:0]   Id2Ex_rs;        // ID2-EX pipeline reg: RS
  reg [4:0]   Id2Ex_rt;        // ID2-EX pipeline reg: RT
  reg [4:0]   Id2Ex_rd2;       // ID2-EX pipeline reg: RD
  reg [31:0]  Id2Ex_imm32;     // ID2-EX pipeline reg: IMM32
  reg [`ADDR] Id2Ex_tpc;       // ID2-EX pipeline reg: taken pc (only valid for beq/bne)
  reg [31:0]  Id2Ex_rrs;       // ID2-EX pipeline reg: value of RS
  reg [31:0]  Id2Ex_rrt;       // ID2-EX pipeline reg: value of RT
  reg [31:0]  Id2Ex_rrt2;      // ID2-EX pipeline reg: value of second operand
  reg         Id2Ex_rsfwme;    // ID2-EX pipeline reg: forwarding necessity of RS (EX -> ME)
  reg         Id2Ex_rsfwwb;    // ID2-EX pipeline reg: forwarding necessity of RS (EX -> WB)
  reg         Id2Ex_rtfwme;    // ID2-EX pipeline reg: forwarding necessity of RT (EX -> ME)
  reg         Id2Ex_rtfwwb;    // ID2-EX pipeline reg: forwarding necessity of RT (EX -> WB)

  reg [`ADDR] ExMe_pc;         // EX-ME pipeline reg: program counter (just for debugging)
  reg         ExMe_sw;         // EX-ME pipeline reg: sw
  reg         ExMe_lw;         // EX-ME pipeline reg: lw
  reg [4:0]   ExMe_rs;         // EX-ME pipeline reg: RS
  reg [4:0]   ExMe_rt;         // EX-ME pipeline reg: RT
  reg [4:0]   ExMe_rd2;        // EX-ME pipeline reg: destination of writeback
  reg [31:0]  ExMe_rrt;        // EX-ME pipeline reg: value of RT (for sw)
  reg [31:0]  ExMe_rslt;       // EX-ME pipeline reg: calculation result (result of arithmetic op or store address)

  reg [`ADDR] MeWb_pc;         // ME-WB pipeline reg: program counter (just for debugging)
  reg         MeWb_lw;         // ME-WB pipeline reg: lw
  reg [4:0]   MeWb_rd2;        // ME-WB pipeline reg: destination of writeback
  reg [31:0]  MeWb_rslt;       // ME-WB pipeline reg: calculation result (result of arithmetic op or store address)

  /**************************** IF stage **********************************/
  wire         ExTAKEN;    // (EX) branch is taken or not
  wire         PR_E;       // (IF) prediction: cache hit
  wire         PR_ISBR;    // (IF) prediction: cache value (only valid when PR_E asserts)
  wire [`ADDR]  PR_ADDR;    // (IF) prediction: branch dest
  reg  [`ADDR]  IfPC;       // (IF) program counter (the instruction on this address will be fetched to IfId_ir on next clock)
  wire [`ADDR]  IfPC4;      // (IF) program counter + 4
  wire [`ADDR]  IfNPC;      // (IF) next program counter (the value of IfPC on next clock)

  assign I_ADDR = IfPC;
  assign IfPC4 = IfPC + 4;
  assign IfNPC = (ExPR_FAIL) ? ((ExTAKEN) ? Id2Ex_tpc : Id2Ex_pc4) :
                 (PR_E) ? (PR_ISBR ? PR_ADDR : IfPC4) : IfPC4;
  
  // program counter update
  always @(posedge CLK) begin
    if (!RST_X) IfPC <= #3 0;
    else if (!If_STALL) begin
      if (ExPR_FAIL) IfPC <= #3 (ExTAKEN) ? Id2Ex_tpc : Id2Ex_pc4;
      else if (PR_E && PR_ISBR) IfPC <= #3 PR_ADDR;
      else IfPC <= #3 IfPC4;
    end
  end

  always @(posedge CLK) begin
    if (!RST_X) {IfId_pc, IfId_pc4, IfId_pr, IfId_ir} <= #3 0;
    else if (!If_STALL) begin
      IfId_pc <= #3 IfPC;
      IfId_pc4 <= #3 IfPC4;
      IfId_pr  <= #3 PR_E && PR_ISBR; // 分岐をどっちに予測したか(分岐予測が提供されればその結果、提供されなければ必ずnot takenと予測)
      IfId_ir  <= #3 I_IN;
    end
  end

  /**************************** ID stage (Instruction Decode) ***********************************/
  wire [5:0]  IdOP    = IfId_ir[31:26];             // (ID) OP

  wire        IdADD   = IdOP == 0 && IdFUNCT == `ADD;
  wire        IdADDI  = IdOP == `ADDI;
`ifdef ENABLE_SHIFT
  wire        IdSLLV  = IdOP == 0 && IdFUNCT == `SLLV;
  wire        IdSRLV  = IdOP == 0 && IdFUNCT == `SRLV;
`endif
  wire        IdBEQ   = IdOP == `BEQ;
  wire        IdBNE   = IdOP == `BNE;
  wire        IdLW    = IdOP == `LW;
  wire        IdSW    = IdOP == `SW;

  wire [4:0]  IdRS    = IfId_ir[25:21];             // (ID) source reg 1
  wire [4:0]  IdRT    = IfId_ir[20:16];             // (ID) source reg 2
  wire [4:0]  IdRD    = IfId_ir[15:11];             // (ID) destination reg (only for R format)
  wire [4:0]  IdRD2;                                // (ID) real destination reg
  wire [5:0]  IdFUNCT = IfId_ir[5:0];               // (ID) funct (only for R format)
  wire [15:0] IdIMM   = IfId_ir[15:0];              // (ID) immediate
  wire [31:0] IdIMM32 = {{16{IdIMM[15]}}, IdIMM};   // (ID) immediate (SignExt32)
  assign IdRD2 = (IdBEQ || IdBNE || IdSW) ? 0 : (IdOP != 0) ? IdRT : IdRD;

  
  wire [10:0] IdTPC = IfId_pc4 + {IdIMM32[29:0], 2'b0};
  wire        IdWE  = IdOP>6'h27;

  always @(posedge CLK) begin
    if (!RST_X) {IdId1_pc, IdId1_pc4, IdId1_pr,
`ifdef ENABLE_SHIFT
      IdId1_sllv, IdId1_srlv, 
`endif
      IdId1_add, IdId1_addi, IdId1_beq, IdId1_bne, IdId1_lw, IdId1_sw,
      IdId1_rs, IdId1_rt, IdId1_rd2, IdId1_imm32,
      IdId1_tpc} <= #3 0;
    else if (!Id_STALL) begin
      IdId1_pc    <= #3 IfId_pc;
      IdId1_pc4   <= #3 IfId_pc4;
      IdId1_pr    <= #3 IfId_pr;

      IdId1_add   <= #3 IdADD;
      IdId1_addi  <= #3 IdADDI;
`ifdef ENABLE_SHIFT
      IdId1_sllv  <= #3 IdSLLV;
      IdId1_srlv  <= #3 IdSRLV;
`endif
      IdId1_beq   <= #3 !r_pr_fail && IdBEQ;
      IdId1_bne   <= #3 !r_pr_fail && IdBNE;
      IdId1_lw    <= #3 !r_pr_fail && IdLW;
      IdId1_sw    <= #3 !r_pr_fail && IdSW;

      IdId1_rs    <= #3 IdRS;
      IdId1_rt    <= #3 IdRT;
      IdId1_rd2   <= #3 r_pr_fail ? 0 : IdRD2;
      IdId1_imm32 <= #3 IdIMM32;

      IdId1_tpc   <= #3 IdTPC;
    end
  end 
  
  /**************************** ID1 stage (Register Fetch) ***********************************/
  wire [31:0] Id1RRS;      // (ID1) value of RS
  wire [31:0] Id1RRT;      // (ID1) value of RT
  wire [31:0] WbRSLT;      // (ME) value to writeback (memory read result in ME or calc result in EX)
  wire [31:0] Id1RRS_FW;   // (ID1) value of RS
  wire [31:0] Id1RRT_FW;   // (ID1) value of RS

  FORWARDING_SINGLE id1frs(.SRC(IdId1_rs), .DIN0(Id1RRS), .DOUT(Id1RRS_FW), .DST1(MeWb_rd2), .DIN1(WbRSLT));
  FORWARDING_SINGLE id1frt(.SRC(IdId1_rt), .DIN0(Id1RRT), .DOUT(Id1RRT_FW), .DST1(MeWb_rd2), .DIN1(WbRSLT));
  
  GPR gpr(.CLK(CLK), .REGNUM0(IdId1_rs), .REGNUM1(IdId1_rt), .DOUT0(Id1RRS), .DOUT1(Id1RRT),
          .REGNUM2(MeWb_rd2), .DIN0(WbRSLT), .WE0(!Wb_STALL));
  always @(posedge CLK) begin
    if (!RST_X) {Id1Id2_pc, Id1Id2_pc4, Id1Id2_pr,
`ifdef ENABLE_SHIFT
      Id1Id2_sllv, Id1Id2_srlv,
`endif
      Id1Id2_add, Id1Id2_addi, Id1Id2_beq, Id1Id2_bne, Id1Id2_lw, Id1Id2_sw,
      Id1Id2_rs, Id1Id2_rt, Id1Id2_rd2, Id1Id2_imm32,
      Id1Id2_tpc,
      Id1Id2_rrs, Id1Id2_rrt} <= #3 0;
    else if (!Id1_STALL) begin
      Id1Id2_pc    <= #3 IdId1_pc;
      Id1Id2_pc4   <= #3 IdId1_pc4;
      Id1Id2_pr    <= #3 IdId1_pr;

      Id1Id2_add   <= #3 IdId1_add;
      Id1Id2_addi  <= #3 IdId1_addi;
`ifdef ENABLE_SHIFT
      Id1Id2_sllv  <= #3 IdId1_sllv;
      Id1Id2_srlv  <= #3 IdId1_srlv;
`endif
      Id1Id2_beq   <= #3 !r_pr_fail && IdId1_beq;
      Id1Id2_bne   <= #3 !r_pr_fail && IdId1_bne;
      Id1Id2_lw    <= #3 !r_pr_fail && IdId1_lw;
      Id1Id2_sw    <= #3 !r_pr_fail && IdId1_sw;

      Id1Id2_rs    <= #3 IdId1_rs;
      Id1Id2_rt    <= #3 IdId1_rt;
      Id1Id2_rd2   <= #3 r_pr_fail ? 0 : IdId1_rd2;
      Id1Id2_imm32 <= #3 IdId1_imm32;

      Id1Id2_tpc   <= #3 IdId1_tpc;

      Id1Id2_rrs   <= #3 Id1RRS_FW;
      Id1Id2_rrt   <= #3 Id1RRT_FW;
    end
  end  

  /**************************** ID2 stage (Data Fowarding) ***********************************/
  wire [31:0] Id2RRS_FW;             // forwarded RS
  wire [31:0] Id2RRT_FW;             // forwarded RT
  wire [31:0] Id2RRS_INTERLOCK_FW;   // forwarded RS (retry)
  wire [31:0] Id2RRT_INTERLOCK_FW;   // forwarded RT (retry)

  wire Id2RRT2_IS_IMM = Id1Id2_addi || Id1Id2_sw || Id1Id2_lw;
  wire ExRRT2_IS_IMM = Id2Ex_addi || Id2Ex_sw || Id2Ex_lw;

  FORWARDING id2fwrs (.SRC(Id1Id2_rs), .DIN0(Id1Id2_rrs), .DOUT(Id2RRS_FW),
                      .DST1(ExMe_rd2), .DIN1(ExMe_rslt), .DST2(MeWb_rd2), .DIN2(WbRSLT));
  FORWARDING id2fwrt (.SRC(Id1Id2_rt), .DIN0(Id1Id2_rrt), .DOUT(Id2RRT_FW),
                      .DST1(ExMe_rd2), .DIN1(ExMe_rslt), .DST2(MeWb_rd2), .DIN2(WbRSLT));
  FORWARDING id2fwrs_interlock (.SRC(Id2Ex_rs), .DIN0(Id2Ex_rrs), .DOUT(Id2RRS_INTERLOCK_FW),
                                .DST1(ExMe_rd2), .DIN1(ExMe_rslt), .DST2(MeWb_rd2), .DIN2(WbRSLT));
  FORWARDING id2fwrt_interlock (.SRC(Id2Ex_rt), .DIN0(Id2Ex_rrt), .DOUT(Id2RRT_INTERLOCK_FW),
                                .DST1(ExMe_rd2), .DIN1(ExMe_rslt), .DST2(MeWb_rd2), .DIN2(WbRSLT));

  always @(posedge CLK) begin
    if (!RST_X) {Id2Ex_pc, Id2Ex_pc4, Id2Ex_pr,
`ifdef ENABLE_SHIFT
      Id2Ex_sllv, Id2Ex_srlv,
`endif
      Id2Ex_add, Id2Ex_addi, Id2Ex_beq, Id2Ex_bne, Id2Ex_lw, Id2Ex_sw,
      Id2Ex_rs, Id2Ex_rt, Id2Ex_rd2,
      Id2Ex_tpc,
      Id2Ex_rrs, Id2Ex_rrt, Id2Ex_rrt2,
      Id2Ex_rsfwme, Id2Ex_rsfwwb, Id2Ex_rtfwme, Id2Ex_rtfwwb} <= #3 0;
    else if (!Id2_STALL) begin
      Id2Ex_pc     <= #3 Id1Id2_pc;
      Id2Ex_pc4    <= #3 Id1Id2_pc4;
      Id2Ex_pr     <= #3 Id1Id2_pr;

      Id2Ex_add    <= #3 Id1Id2_add;
      Id2Ex_addi   <= #3 Id1Id2_addi;
`ifdef ENABLE_SHIFT
      Id2Ex_sllv   <= #3 Id1Id2_sllv;
      Id2Ex_srlv   <= #3 Id1Id2_srlv;
`endif
      Id2Ex_beq    <= #3 !r_pr_fail && Id1Id2_beq;
      Id2Ex_bne    <= #3 !r_pr_fail && Id1Id2_bne;
      Id2Ex_lw     <= #3 !r_pr_fail && Id1Id2_lw;
      Id2Ex_sw     <= #3 !r_pr_fail && Id1Id2_sw;

      Id2Ex_rs     <= #3 Id1Id2_rs;
      Id2Ex_rt     <= #3 Id1Id2_rt;
      Id2Ex_rd2    <= #3 r_pr_fail ? 0 : Id1Id2_rd2;

      Id2Ex_tpc    <= #3 Id1Id2_tpc;

      Id2Ex_rrs    <= #3 Id2RRS_FW;
      Id2Ex_rrt    <= #3 Id2RRT_FW;
      Id2Ex_rrt2   <= #3 Id2RRT2_IS_IMM ? Id1Id2_imm32 : Id2RRT_FW;
      // MEM, WBからのフォワーディングの必要性を判定
      Id2Ex_rsfwme <= #3 Id2Ex_rd2 && Id2Ex_rd2 == Id1Id2_rs;
      Id2Ex_rsfwwb <= #3 ExMe_rd2 && ExMe_rd2 == Id1Id2_rs;
      // rtはRフォーマット(IdOP > 5)の場合のみ必要
      Id2Ex_rtfwme <= #3 !Id2RRT2_IS_IMM && Id2Ex_rd2 && Id2Ex_rd2 == Id1Id2_rt;
      Id2Ex_rtfwwb <= #3 !Id2RRT2_IS_IMM && ExMe_rd2 && ExMe_rd2 == Id1Id2_rt; 
    end else begin
      Id2Ex_rrs    <= #3 Id2RRS_INTERLOCK_FW;
      Id2Ex_rrt    <= #3 Id2RRT_INTERLOCK_FW;
      Id2Ex_rrt2   <= #3 ExRRT2_IS_IMM ? Id2Ex_rrt2 : Id2RRT_INTERLOCK_FW;
    end
  end

  /**************************** EX stage ***********************************/
  // 必要に応じてフォワーディングする(WBからはWbRSLTでなくMeWb_rsltをフォワード(lwの結果はフォワードしない))
  wire [31:0] ExOP1 = (Id2Ex_rsfwme) ? ExMe_rslt : 
                      /*(Id2Ex_rsfwwb) ? MeWb_rslt :*/ Id2Ex_rrs;
  wire [31:0] ExOP2 = (Id2Ex_rtfwme) ? ExMe_rslt :
                      /*(Id2Ex_rtfwwb) ? MeWb_rslt :*/ Id2Ex_rrt2;

  // ALU
`ifdef ENABLE_SHIFT
  wire [31:0] #10 ExRSLT = Id2Ex_sllv ? ExOP1 << ExOP2[4:0] :
                           Id2Ex_srlv ? ExOP1 >> ExOP2[4:0] :
                           ExOP1 + ExOP2;
`else
  wire [31:0] #10 ExRSLT = ExOP1 + ExOP2;
`endif
  
  // WBからメモリの読み出し結果をフォワードしなければならない場合、インターロック
  assign WBEXFW_INTERLOCK = MeWb_lw && (Id2Ex_rsfwwb || Id2Ex_rtfwwb);
  
  // 分岐判定
  wire ExBE = !r_pr_fail && (Id2Ex_beq || Id2Ex_bne);
  assign ExTAKEN = (Id2Ex_bne) ? ExOP1!=ExOP2 : ExOP1==ExOP2;
  wire ExPR_FAIL = ExBE && ExTAKEN != Id2Ex_pr; // 分岐に失敗したかどうか
  // r_pr_fail = 1 の場合、その時点でID, EXステージにある命令はNOP化しなければならない
  // 以下の方法を用いてNOP化する
  // ・rd2を0にする(レジスタ書き込み無効化)
  // ・beq/bne/lw/sw を0にする(分岐、メモリ書き込み無効化)
  reg r_pr_fail;
  m_predictor m_brp (CLK, Id2Ex_pc, ExTAKEN, Id2Ex_tpc, ExBE, IfPC, PR_ISBR, PR_ADDR, PR_E);

  // sw
  wire [31:0] ExRRT;
  FORWARDING_SINGLE exfrt(.SRC(Id2Ex_rt), .DIN0(Id2Ex_rrt), .DOUT(ExRRT), .DST1(MeWb_rd2), .DIN1(WbRSLT));
  
  always @(posedge CLK) begin
    if (!RST_X) {r_pr_fail, ExMe_pc,
      ExMe_lw, ExMe_sw,
      ExMe_rt, ExMe_rd2,
      ExMe_rslt, ExMe_rrt} <= #3 0;
    else if (!Ex_STALL) begin
      r_pr_fail <= #3 ExPR_FAIL;
      ExMe_pc   <= #3 Id2Ex_pc;

      ExMe_lw   <= #3 !r_pr_fail && Id2Ex_lw;
      ExMe_sw   <= #3 !r_pr_fail && Id2Ex_sw;

      ExMe_rt   <= #3 Id2Ex_rt;
      ExMe_rd2  <= #3 r_pr_fail ? 0 : Id2Ex_rd2;

      ExMe_rslt <= #3 ExRSLT;
      ExMe_rrt  <= #3 ExRRT;
    end
  end

  /**************************** MEM stage **********************************/
  wire [31:0] MeSTD;  // (ME) value to store into memory
  FORWARDING_SINGLE mefrt(.SRC(ExMe_rt), .DIN0(ExMe_rrt), .DOUT(MeSTD), .DST1(MeWb_rd2), .DIN1(WbRSLT));

  // communicate with DRAM
  assign D_ADDR = ExMe_rslt;
  assign D_OUT  = MeSTD;
  assign D_WE   = (!Me_STALL && ExMe_sw) ? 4'b1111 : 0;
  assign D_OE   = !Me_STALL && ExMe_lw;

  always @(posedge CLK) begin
    if (!RST_X) {MeWb_pc,
      MeWb_lw, MeWb_rd2,
      MeWb_rslt} <= #3 0;
    else if (!Me_STALL) begin
      MeWb_pc <= #3 ExMe_pc;

      MeWb_lw <= #3 ExMe_lw;
      MeWb_rd2 <= #3 ExMe_rd2;

      MeWb_rslt <= #3 ExMe_rslt;
    end
    // if (ExMe_we) $write("Storing at %04x: %08x\n", ExMe_rslt[12:2], MeSTD);
  end
  /**************************** WB stage ***********************************/
  assign WbRSLT = MeWb_lw ? D_IN : MeWb_rslt;
  /*************************************************************************/
endmodule

/******************************************************************************************************/
/* 32bitx32 2R/1W General Purpose Registers (Register File)                                           */
/******************************************************************************************************/
module GPR(CLK, REGNUM0, REGNUM1, REGNUM2, DIN0, WE0, DOUT0, DOUT1);
    input wire        CLK;
    input wire  [4:0] REGNUM0, REGNUM1, REGNUM2;
    input wire [31:0] DIN0;
    input wire        WE0;
    output reg [31:0] DOUT0, DOUT1;

    reg [31:0] r[0:31];

    always @(negedge CLK) DOUT0 <= (REGNUM0==0) ? 0 : r[REGNUM0];
    always @(negedge CLK) DOUT1 <= (REGNUM1==0) ? 0 : r[REGNUM1];
    always @(posedge CLK) if(WE0) r[REGNUM2] <= DIN0;
endmodule

/***** data forwarding unit                                                                       *****/
/******************************************************************************************************/
module FORWARDING(SRC, DIN0, DST1, DIN1, DST2, DIN2, DOUT);
    input  wire  [4:0] SRC, DST1, DST2;   // register number 
    input  wire [31:0] DIN0, DIN1, DIN2;  // 32bit value
    output wire [31:0] DOUT;              // 32bit data output

    assign DOUT = (SRC!=0 && DST1==SRC) ? DIN1 : (SRC!=0 && DST2==SRC) ? DIN2 : DIN0;
endmodule
module FORWARDING_SINGLE(SRC, DIN0, DST1, DIN1, DOUT);
    input  wire  [4:0] SRC, DST1;   // register number 
    input  wire [31:0] DIN0, DIN1;  // 32bit value
    output wire [31:0] DOUT;              // 32bit data output

    assign DOUT = (SRC!=0 && DST1==SRC) ? DIN1 : DIN0;
endmodule
/******************************************************************************************************/