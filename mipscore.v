/******************************************************************************************************/
/* Sample Verilog HDL Code for CSC.T363 Computer Architecture                    Arch Lab. TOKYO TECH */
/******************************************************************************************************/
`include "define.v"
/******************************************************************************************************/
/* MipsCore: 32bit-MIPS 5-stage pipelining processor                                                  */
/******************************************************************************************************/
`ifdef USE_MIPSCORE
module MIPSCORE(CLK, RST_X, STALL, I_ADDR, I_IN, D_ADDR, D_IN, D_OUT, D_OE, D_WE);
    input  wire         CLK, RST_X, STALL;
    output wire [`ADDR] I_ADDR, D_ADDR;
    input  wire [31:0]  I_IN, D_IN;
    output wire [31:0]  D_OUT;
    output wire [3:0]   D_WE;
    output wire         D_OE;

    reg [`ADDR]  pc;          // program counter
    assign I_ADDR = pc;
    /**************************************************************************************************/
    reg [`ADDR] IfId_npc;     // IF-ID pipeline reg: next pc, pc + 4
    reg [31:0]  IfId_ir;      // IF-ID pipeline reg: instruction

    reg [`ADDR] IdEx_npc;     // ID-EX pipeline reg: next pc, pc + 4
    reg [31:0]  IdEx_rrs;     // ID-EX pipeline reg: fetched operand of R[rs]  
    reg [31:0]  IdEx_rrt;     // ID-EX pipeline reg: fetched operand of R[rt]
    reg [4:0]   IdEx_dst;     // ID-EX pipeline reg: decoded destination reg number
    reg [31:0]  IdEx_ir;      // ID-EX pipeline reg: instruction
    reg [`OPNT] IdEx_opn;     // ID-EX pipeline reg: decoded operation ID
    reg [`ATTR] IdEx_attr;    // ID-EX pipeline reg: decoded instruction attribute 

    reg [4:0]   ExMa_dst;     // EX-MA pipeline reg: decoded destination reg number
    reg [31:0]  ExMa_rslt;    // EX-MA pipeline reg: execution result
    reg [3:0]   ExMa_mwe;     // EX-MA pipeline reg: mem write enable
    reg         ExMa_oe;      // EX-MA pipeline reg: data memory output enable
    reg [4:0]   ExMa_lds;     // EX-MA pipeline reg: load selector
    reg [31:0]  ExMa_std;     // EX-MA pipeline reg: store data
    
    reg [4:0]   MaWb_dst;     // MA-WB pipeline reg: decoded destination reg number 
    reg [31:0]  MaWb_rslt;    // MA-WB pipeline reg: execution result

    /**************************************************************************************************/
    reg [`ADDR] IdTPC;        // wire, calculated branch Taken address (Taken Program Counter)
    reg         IdC;          // wire, Calculated branch result, branch Taken or Untaken
    reg         IdB;          // wire, branch/jump instruction ?
    wire [31:0] MaRSLT;       // wire, execution result on MA stage
    wire        bstall;       // stall signal by branch instruction
    wire        PSTALL = STALL;  // pipeline stall
    
    /**************************************************************************************************/
    /* Stage 1 : IF, instruction fetch                                                                */
    /**************************************************************************************************/
    wire [`ADDR] IfNPC = pc + 4;
    
    always @( posedge CLK ) begin ///// update program counter
        if(!RST_X)                  pc <= 0;
        else if(!PSTALL && !bstall) pc <= (IdC) ? IdTPC : IfNPC; // If branch taken, uses taken PC
    end

    always @( posedge CLK ) begin  ///// update pipeline registers
        if(!RST_X) {IfId_npc, IfId_ir} <= 0;
        else if(!PSTALL && !bstall) begin
            IfId_npc <= IfNPC;
            IfId_ir  <= I_IN;  // if branch taken then NOP else instruction from imem.
        end
    end
        
    /**************************************************************************************************/
    /* Stage 2 : ID, instruction decode & operand fetch                                               */
    /**************************************************************************************************/
    wire [5:0] IdOP   = IfId_ir[31:26]; // opcode field of instruction  
    wire [4:0] IdRS   = IfId_ir[25:21]; // rs field of instruction    
    wire [4:0] IdRT   = IfId_ir[20:16]; // rt field of instruction
    wire [4:0] IdRD   = IfId_ir[15:11]; // rd field of instruction
    wire [5:0] IdFCT  = IfId_ir[5:0];   // funct field of instruction   
    wire [31:0] IdRRS, IdRRT;           // register value of rs and rt

    assign bstall = IdB && 
                  ((IdRS!=0 && (IdRS==IdEx_dst || IdRS==ExMa_dst)) ||
                   (IdRT!=0 && (IdRT==IdEx_dst || IdRT==ExMa_dst))); // branch stall
                   
    GPR gpr(.CLK(CLK), .REGNUM0(IdRS), .REGNUM1(IdRT), .DOUT0(IdRRS), .DOUT1(IdRRT), // register file
            .REGNUM2(ExMa_dst), .DIN0(MaRSLT), .WE0(!PSTALL));

    reg [6:0]   IdOPN;   // instruction operation number (unique operation ID)
    reg [4:0]   IdDST;   // destination register
    reg [`ATTR] IdATTR;  // instruction attribute
    always @(IfId_ir) begin
        IdOPN   = `ERROR____; 
        IdDST  = 0;  
        IdATTR = 0;
        case (IdOP) // OP
            6'h00: case (IdFCT) // FUNCT 
                6'h00: begin IdOPN= (IdRD) ? `SLL______ : `NOP______; IdDST=IdRD; end
                6'h02: begin IdOPN=`SRL______; IdDST=IdRD;                        end
                6'h03: begin IdOPN=`SRA______; IdDST=IdRD;                        end
                6'h04: begin IdOPN=`SLLV_____; IdDST=IdRD;                        end
                6'h06: begin IdOPN=`SRLV_____; IdDST=IdRD;                        end
                6'h07: begin IdOPN=`SRAV_____; IdDST=IdRD;                        end
                6'h08: begin IdOPN=`JR_______; IdDST=0;                           end
                6'h09: begin IdOPN=`JALR_____; IdDST=31;                          end
                6'h20: begin IdOPN=`ADD______; IdDST=IdRD;                        end
                6'h21: begin IdOPN=`ADDU_____; IdDST=IdRD;                        end
                6'h22: begin IdOPN=`SUB______; IdDST=IdRD;                        end
                6'h23: begin IdOPN=`SUBU_____; IdDST=IdRD;                        end
                6'h24: begin IdOPN=`AND______; IdDST=IdRD;                        end
                6'h25: begin IdOPN=`OR_______; IdDST=IdRD;                        end
                6'h26: begin IdOPN=`XOR______; IdDST=IdRD;                        end
                6'h27: begin IdOPN=`NOR______; IdDST=IdRD;                        end
                6'h2a: begin IdOPN=`SLT______; IdDST=IdRD;                        end
                6'h2b: begin IdOPN=`SLTU_____; IdDST=IdRD;                        end
            endcase
            6'h01: case (IdRT)
                5'h00: begin IdOPN=`BLTZ_____; IdDST=0;                           end
                5'h01: begin IdOPN=`BGEZ_____; IdDST=0;                           end
            endcase
            6'h02:     begin IdOPN=`J________; IdDST=0;                           end
            6'h03:     begin IdOPN=`JAL______; IdDST=31;                          end
            6'h04:     begin IdOPN=`BEQ______; IdDST=0;                           end
            6'h05:     begin IdOPN=`BNE______; IdDST=0;                           end
            6'h06:     begin IdOPN=`BLEZ_____; IdDST=0;                           end
            6'h07:     begin IdOPN=`BGTZ_____; IdDST=0;                           end
            6'h08:     begin IdOPN=`ADDI_____; IdDST=IdRT;                        end
            6'h09:     begin IdOPN=`ADDIU____; IdDST=IdRT;                        end
            6'h0a:     begin IdOPN=`SLTI_____; IdDST=IdRT;                        end
            6'h0b:     begin IdOPN=`SLTIU____; IdDST=IdRT;                        end
            6'h0c:     begin IdOPN=`ANDI_____; IdDST=IdRT;                        end
            6'h0d:     begin IdOPN=`ORI______; IdDST=IdRT;                        end
            6'h0e:     begin IdOPN=`XORI_____; IdDST=IdRT;                        end
            6'h0f:     begin IdOPN=`LUI______; IdDST=IdRT;                        end
            6'h23:     begin IdOPN=`LW_______; IdDST=IdRT; IdATTR=`LD_4B;         end
            6'h2b:     begin IdOPN=`SW_______; IdDST=0;    IdATTR=`ST_4B;         end
        endcase
    end
    
    wire [`ADDR] IdBPC = IfId_npc + ({{16{IfId_ir[15]}}, IfId_ir[15:0]} << 2);
    always @(*) begin  ////// branch & jump resolution unit
        {IdTPC, IdC, IdB} = 0;
        case (IdOP)
            6'h04: begin IdB=1; IdTPC = IdBPC; IdC = (IdRRS == IdRRT);         end        // BEQ
            6'h05: begin IdB=1; IdTPC = IdBPC; IdC = (IdRRS != IdRRT);         end        // BNE
            6'h06: begin IdB=1; IdTPC = IdBPC; IdC = ( IdRRS[31]||(IdRRS==0)); end        // BLEZ
            6'h07: begin IdB=1; IdTPC = IdBPC; IdC = (~IdRRS[31]&&(IdRRS!=0)); end        // BGTZ
            6'h01: begin IdB=1; IdTPC = IdBPC; IdC = (IdRT) ? ~IdRRS[31] : IdRRS[31]; end // BGEZ,BLTZ
            6'h02: begin IdB=1; IdTPC = {IfId_npc[31:28], IfId_ir[25:0], 2'b0}; IdC=1;end // J
            6'h03: begin IdB=1; IdTPC = {IfId_npc[31:28], IfId_ir[25:0], 2'b0}; IdC=1;end // JAL
            6'h00: if      (IdFCT==6'h08) begin IdB=1; IdTPC = IdRRS; IdC = 1; end        // JR
                   else if (IdFCT==6'h09) begin IdB=1; IdTPC = IdRRS; IdC = 1; end        // JALR
        endcase
    end
    
    always @(posedge CLK) begin ///// update pipeline registers
        if(!RST_X) {IdEx_npc, IdEx_rrs, IdEx_rrt, IdEx_dst, IdEx_ir, IdEx_opn, IdEx_attr} <= 0;
        else if(!PSTALL) begin
            IdEx_npc   <= (bstall) ? 0 : IfId_npc;
            IdEx_rrs   <= (bstall) ? 0 : IdRRS;    // data from general-purpose register file
            IdEx_rrt   <= (bstall) ? 0 : IdRRT;    // data from general-purpose register file
            IdEx_dst   <= (bstall) ? 0 : IdDST;
            IdEx_ir    <= (bstall) ? 0 : IfId_ir;
            IdEx_opn   <= (bstall) ? 0 : IdOPN;
            IdEx_attr  <= (bstall) ? 0 : IdATTR;
        end
    end
    
    /**************************************************************************************************/
    /* Stage 3 : EX, execute & address generation for LD/ST                                           */
    /**************************************************************************************************/
    wire        [31:0] RRS_U, RRT_U;                       // output of data forwarding unit
    wire signed [31:0] RRS_S = RRS_U;                      // signed wire
    wire signed [31:0] RRT_S = RRT_U;                      // signed wire
    wire         [4:0] SHAMT  = IdEx_ir[10:6];             // Shift amount
    wire        [15:0] IMM    = IdEx_ir[15:0];             // Immediate
    wire        [31:0] SET32I = {{16{IMM[15]}}, IMM};      // Sign Extended Imm.
    wire       [`ADDR] SETADI = SET32I[`ADDR] << 2;        // shifted immediate of address bit width
    wire       [`ADDR] JADDR  = IdEx_ir[`ADDR] << 2;       // Juma address
    wire       [`ADDR] ExA = RRS_U[`ADDR] + SET32I[`ADDR]; // mem address of LD/ST
    reg         [31:0] ExRSLT;                             // execution result
    reg          [3:0] ExWE;                               // memory write enable vector

    FORWARDING frs(.SRC(IdEx_ir[25:21]), .DIN0(IdEx_rrs), .DOUT(RRS_U), 
                   .DST1(ExMa_dst), .DIN1(ExMa_rslt), .DST2(MaWb_dst), .DIN2(MaWb_rslt));
    FORWARDING frt(.SRC(IdEx_ir[20:16]), .DIN0(IdEx_rrt), .DOUT(RRT_U), 
                   .DST1(ExMa_dst), .DIN1(ExMa_rslt), .DST2(MaWb_dst), .DIN2(MaWb_rslt));

    always @(*) begin
        {ExRSLT, ExWE} = 0;
        case ( IdEx_opn )
          `ADD______ : begin ExRSLT = RRS_U + RRT_U;                                             end
          `ADDI_____ : begin ExRSLT = RRS_U + SET32I;                                            end
          `ADDIU____ : begin ExRSLT = RRS_U + SET32I;                                            end
          `ADDU_____ : begin ExRSLT = RRS_U + RRT_U;                                             end
          `SUB______ : begin ExRSLT = RRS_U - RRT_U;                                             end
          `SUBU_____ : begin ExRSLT = RRS_U - RRT_U;                                             end
          `AND______ : begin ExRSLT = RRS_U & RRT_U;                                             end
          `ANDI_____ : begin ExRSLT = RRS_U & {16'h0, IMM};                                      end
          `NOR______ : begin ExRSLT = ~(RRS_U | RRT_U);                                          end
          `OR_______ : begin ExRSLT = RRS_U | RRT_U;                                             end
          `ORI______ : begin ExRSLT = RRS_U | {16'h0, IMM};                                      end
          `XOR______ : begin ExRSLT = RRS_U ^ RRT_U;                                             end
          `XORI_____ : begin ExRSLT = RRS_U ^ {16'h0, IMM};                                      end
          `SLL______ : begin ExRSLT = RRT_U <<  SHAMT;                                           end
          `SRL______ : begin ExRSLT = RRT_U >>  SHAMT;                                           end
          `SRA______ : begin ExRSLT = RRT_S >>> SHAMT;                                           end
          `SLLV_____ : begin ExRSLT = RRT_U <<  RRS_U[4:0];                                      end
          `SRLV_____ : begin ExRSLT = RRT_U >>  RRS_U[4:0];                                      end
          `SRAV_____ : begin ExRSLT = RRT_S >>> RRS_U[4:0];                                      end
          `SLT______ : begin ExRSLT = (RRS_U[31] ^ RRT_U[31]) ? RRS_U[31] : (RRS_U < RRT_U);     end
          `SLTI_____ : begin ExRSLT = (RRS_U[31] ^   IMM[15]) ? RRS_U[31] : (RRS_U < SET32I);    end
          `SLTIU____ : begin ExRSLT = (RRS_U < SET32I);                                          end
          `SLTU_____ : begin ExRSLT = (RRS_U < RRT_U);                                           end
          `JAL______ : begin ExRSLT = IdEx_npc + 4;                                              end
          `JALR_____ : begin ExRSLT = IdEx_npc + 4;                                              end
          `LUI______ : begin ExRSLT = {IMM, 16'h0};                                              end
          `LW_______ : begin ExRSLT = {ExA[31:2], 2'b00};                                        end
          `SW_______ : begin ExRSLT = {ExA[31:2], 2'b00}; ExWE = 4'b1111;                        end
        endcase
    end

    always @( posedge CLK ) begin ///// update pipeline registers
        if(!RST_X) {ExMa_rslt, ExMa_dst, ExMa_mwe, ExMa_oe, ExMa_lds, ExMa_std} <= 0;
        else if(!PSTALL) begin
            ExMa_rslt <= ExRSLT;
            ExMa_dst  <= IdEx_dst;
            ExMa_std  <= RRT_U;
            ExMa_mwe  <= ExWE;
            ExMa_oe   <= (IdEx_attr & `LD_ANY) ?   1 : 0;
            ExMa_lds  <= (IdEx_opn==`LW_______) ?  1 : 0;
        end 
    end

    /**************************************************************************************************/
    /* Stage 4 : MA, memory access for LD/ST                                                          */
    /**************************************************************************************************/
   assign D_ADDR = ExRSLT;
   assign D_OUT  = RRT_U;
   assign D_WE   = (PSTALL) ? 0 : ExWE; // 4-bit wire
   assign D_OE   = (IdEx_attr && `LDST_ANY && !PSTALL);
   
   assign MaRSLT = (ExMa_lds) ? D_IN : ExMa_rslt;

    always @( posedge CLK ) begin ///// update pipeline registers
        if(!RST_X) {MaWb_rslt, MaWb_dst} <= 0;
        else if(!PSTALL) begin
            MaWb_rslt <= MaRSLT;
            MaWb_dst  <= ExMa_dst;
        end
    end
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
/******************************************************************************************************/
`endif