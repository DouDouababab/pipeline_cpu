`include "def.v"

module Ctrl (
    input [6:0] Op,
    input [2:0] Funct3,
    input [6:0] Funct7,
    output RegWrite,  //Register Write
    output MemWrite,  //Memory Write
    output [5:0] EXTOp,  //Extend Operation
    output [4:0] ALUOp,  //ALU Operation
    output ASource,  //ALU A Source
    output BSource,  //ALU B Source
    output [2:0] DMType,  //Data Memory Type
    output [1:0] WDSel,  //Memory Write Data Select
    output i_jalr,  //jalr 
    output u_lui,  //lui
    output MemRead
);

  //R type
  wire rtype = ~Op[6] & Op[5] & Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];  //rtype op 0110011
  wire r_add=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&~Funct3[0]; // add 0000000 000
  wire r_sub=rtype&~Funct7[6]&Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&~Funct3[0]; // sub 0100000 000
  wire r_and=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]&Funct3[1]&Funct3[0]; // and 0000000 111
  wire r_or = rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]&Funct3[1]&~Funct3[0]; // or 0000000 110
  wire r_xor =rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]&~Funct3[1]&~Funct3[0]; // xor 0000000 100
  wire r_sll =rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&Funct3[0]; // sll 0000000 001
  wire r_srl =rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]&~Funct3[1]&Funct3[0]; // srl 0000000 101
  wire r_sra =rtype&~Funct7[6]&Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]&~Funct3[1]&Funct3[0]; // sra 0100000 101
  wire r_slt =rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&Funct3[1]&~Funct3[0]; // slt 0000000 010
  wire r_sltu=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&Funct3[1]&Funct3[0]; // sltu 0000000 011
  //I type
  //load
  wire itype_l = ~Op[6] & ~Op[5] & ~Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];  //0000011
  wire i_lb = itype_l & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];  //lb 000
  wire i_lbu = itype_l & Funct3[2] & ~Funct3[1] & ~Funct3[0];  //lbu 100
  wire i_lh = itype_l & ~Funct3[2] & ~Funct3[1] & Funct3[0];  //lh 001
  wire i_lhu = itype_l & Funct3[2] & ~Funct3[1] & Funct3[0];  //lhu 101
  wire i_lw = itype_l & ~Funct3[2] & Funct3[1] & ~Funct3[0];  //lw 010
  //i
  wire itype_r = ~Op[6] & ~Op[5] & Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];  //0010011
  wire i_addi = itype_r & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];  // addi 000 func3
  wire i_andi = itype_r & Funct3[2] & Funct3[1] & Funct3[0];  // andi 111
  wire i_ori = itype_r & Funct3[2] & Funct3[1] & ~Funct3[0];  // ori 110
  wire i_xori = itype_r & Funct3[2] & ~Funct3[1] & ~Funct3[0];  // xori 100
  wire i_slli = itype_r && (Funct7 == 7'b0000000) && (Funct3 == 3'b001);  //0000000 001
  wire i_srli = itype_r && (Funct7 == 7'b0000000) && (Funct3 == 3'b101);  //0000000 101
  wire i_srai = itype_r && (Funct7 == 7'b0100000) && (Funct3 == 3'b101);  //0100000 101
  wire i_slti = itype_r & ~Funct3[2] & Funct3[1] & ~Funct3[0];  // slti 010
  wire i_sltiu = itype_r & ~Funct3[2] & Funct3[1] & Funct3[0];  // sltiu 011
  //B type
  wire btype = Op[6] & Op[5] & ~Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];  //1100011
  //S type
  wire stype = ~Op[6] & Op[5] & ~Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];  //0100011
  wire s_sb = stype & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];  //sb 000
  wire s_sh = stype && ~Funct3[2] & ~Funct3[1] & Funct3[0];  //sh 001
  wire s_sw = stype & ~Funct3[2] & Funct3[1] & ~Funct3[0];  // sw 010
  //J type
  wire j_jal = Op[6] & Op[5] & ~Op[4] & Op[3] & Op[2] & Op[1] & Op[0];  //op=1101111
  assign i_jalr= Op[6] & Op[5] & ~Op[4] & ~Op[3] & Op[2] & Op[1] & Op[0] & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];  //op=1100111 000
  //U type
  wire u_auipc = ~Op[6] & ~Op[5] & Op[4] & ~Op[3] & Op[2] & Op[1] & Op[0];  //op=0010111
  assign u_lui = ~Op[6] & Op[5] & Op[4] & ~Op[3] & Op[2] & Op[1] & Op[0];  //op=0110111
  //signal
  assign RegWrite = rtype | itype_r | itype_l | u_auipc | u_lui | i_jalr | j_jal;  // register write
  assign MemWrite = stype & (s_sb | s_sh | s_sw);  // memory write
  assign ASource = u_auipc;  // ALU A is from register or PC 
  assign BSource = itype_l | itype_r | u_auipc | stype | i_jalr | u_lui;  // ALU B is from instruction immediate
  assign WDSel = (i_jalr | j_jal) ? `WDSel_FromPC : (itype_l) ? `WDSel_FromMEM : `WDSel_FromALU;
  //ALUOp
  assign ALUOp = (r_add | i_addi | itype_l | stype | u_auipc | j_jal | i_jalr|u_lui) ? `ALUOp_add :
               (r_sub) ? `ALUOp_sub :
               (r_and | i_andi) ? `ALUOp_and :
               (r_or | i_ori) ? `ALUOp_or :
               (r_xor | i_xori) ? `ALUOp_xor :
               (r_sll | i_slli) ? `ALUOp_sll :
               (r_srl | i_srli) ? `ALUOp_srl :
               (r_sra | i_srai) ? `ALUOp_sra :
               (r_slt | i_slti) ? `ALUOp_slt :
               (r_sltu | i_sltiu) ? `ALUOp_sltu :
               (btype) ? `ALUOp_sub :
               (u_lui) ? `ALUOp_lui :
               `ALUOp_nop;

  //EXTOp
  assign EXTOp[5] = i_slli | i_srai | i_srli;
  assign EXTOp[4] = (itype_l | itype_r | i_jalr) & (~i_slli & ~i_srai & ~i_srli);
  assign EXTOp[3] = stype;
  assign EXTOp[2] = btype;
  assign EXTOp[1] = u_lui | u_auipc;
  assign EXTOp[0] = j_jal;
  assign DMType[2] = i_lbu;
  assign DMType[1] = i_lb | s_sb | i_lhu;
  assign DMType[0] = i_lh | s_sh | i_lb | s_sb;
  assign MemRead=itype_l;

endmodule
