/******************************************************************************************************/
/* Sample Verilog HDL Code for CSC.T363 Computer Architecture                    Arch Lab. TOKYO TECH */
/******************************************************************************************************/
`define PROG_SIZE  (512*1024) // MEMORY IMAGE FILE Load Size in byte (default 512KB)
`define IMEM_SIZE  ( 64*1024) // Instruction Memory Size in byte (default 64KB)
/******************************************************************************************************/
/* Clock Interval Definition                                                                          */
/******************************************************************************************************/
// b = baud rate (in Mbps)
// f = frequency of the clk for the MIPS core (in MHz)
// SERIAL_WCNT = f/b
// e.g. b = 1, f = 50 -> SERIAL_WCNT = 50/1 = 50
`define SERIAL_WCNT  50 // 1M baud UART wait count
/******************************************************************************************************/
`define WORD  31:0                 // 1 word is 32 bit
`define ADDR  31:0                 // Address Range of 32bit

/* Instruction Attribute Definition                                                                   */
/******************************************************************************************************/
`define ATTR  7:0                 // type of instruction Attribute

`define LD_1B        8'b00000001
`define LD_2B        8'b00000010
`define LD_4B        8'b00000100
`define ST_1B        8'b00001000
`define ST_2B        8'b00010000
`define ST_4B        8'b00100000
`define BRANCH       8'b01000000
`define LD_ANY       8'b00000111 // MASK
`define ST_ANY       8'b00111000 // MASK
`define LDST_ANY     8'b00111111 // MASK

/* Unique instruction operation number or OPN (unique instruction id)                                 */
/******************************************************************************************************/
`define OPNT  5:0                 // type of operation number

`define NOP______   6'd00 // NOP must be zero!
`define ERROR____   6'd01
`define SLL______   6'd02 
`define SRL______   6'd03 
`define SRA______   6'd04 
`define SLLV_____   6'd05 
`define SRLV_____   6'd06 
`define SRAV_____   6'd07 
`define JR_______   6'd08
`define JALR_____   6'd09 
// `define MFHI_____   6'd10
// `define MFLO_____   6'd11
// `define MULT_____   6'd12
// `define MULTU____   6'd13
`define ADD______   6'd14
`define ADDU_____   6'd15
`define SUB______   6'd16
`define SUBU_____   6'd17
`define AND______   6'd18
`define OR_______   6'd19
`define XOR______   6'd20
`define NOR______   6'd21
`define SLT______   6'd22
`define SLTU_____   6'd23
`define J________   6'd24
`define JAL______   6'd25
`define BEQ______   6'd26
`define BNE______   6'd27
`define ADDI_____   6'd28
`define ADDIU____   6'd29
`define SLTI_____   6'd30
`define SLTIU____   6'd31
`define ANDI_____   6'd32
`define ORI______   6'd33
`define XORI_____   6'd34
`define LUI______   6'd35
`define LW_______   6'd38
`define SW_______   6'd43
`define BLEZ_____   6'd44
`define BGTZ_____   6'd45
`define BLTZ_____   6'd46
`define BGEZ_____   6'd47
// `define DIV______   6'd48
// `define DIVU_____   6'd49
/******************************************************************************************************/
