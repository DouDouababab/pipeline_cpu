module HazardDetection(
    input ID_EX_MemRead,
    input [4:0] EX_rd,
    input [4:0] ID_rs1,
    input [4:0] ID_rs2,
    output load_Hazard
);
//EX阶段是load指令
//ID阶段的源寄存器与EX的目标寄存器相同
//目标寄存器不是0
assign load_Hazard = (ID_EX_MemRead && ((EX_rd == ID_rs1) || (EX_rd == ID_rs2)) && (EX_rd != 5'b0)) ? 1'b1 : 1'b0;

endmodule