`include "def.v"

module Forward(
  input MEM_RegWrite,
  input [4:0] MEM_rd,
  input WB_RegWrite,
  input [4:0] WB_rd,
  input [4:0] EX_rs1,
  input [4:0] EX_rs2,
  output reg [1:0] Forward_A,
  output reg [1:0] Forward_B
);

always @(*)begin
  if(MEM_RegWrite&& (MEM_rd != 0) && (MEM_rd == EX_rs1))begin//从EX_MEM读取的条件
    Forward_A = `FORWARD_EX_MEM;
  end
  else if(WB_RegWrite && (WB_rd != 0) && (WB_rd == EX_rs1))begin//从MEM_WB读取的条件
    Forward_A = `FORWARD_MEM_WB;
  end
  else begin
    Forward_A = `FORWARD_REG;
  end
end

always @(*)begin
  if(MEM_RegWrite && (MEM_rd != 0) && (MEM_rd == EX_rs2))begin//从EX_MEM读取的条件
    Forward_B = `FORWARD_EX_MEM;
  end
  else if(WB_RegWrite && (WB_rd != 0) && (WB_rd == EX_rs2))begin//从MEM_WB读取的条件
    Forward_B = `FORWARD_MEM_WB;
  end
  else begin
    Forward_B = `FORWARD_REG;
  end
end
  

endmodule