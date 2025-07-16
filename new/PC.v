`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/07/14 20:55:39
// Design Name: 
// Module Name: PC
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module PC(
    input clk,
    input rst,
    input [1:0] EX_PCSel,
    input [31:0] EX_immout,
    input [31:0] EX_ALUout,
    input EX_i_jalr,
    input stall,
    input EX_PC,
    output reg [31:0] PC,
    output reg IF_ID_flush,
    output reg ID_EX_flush,
    output reg IF_ID_write_enable
    );

    reg [31:0] cycles;

   always @(posedge clk or posedge rst) begin
    cycles <= cycles + 1;
    if (rst) begin
      cycles <= 0;
      PC <= 0;
    end else begin//分支跳转处理
      if (EX_PCSel==2'b01 || EX_PCSel == 2'b10) begin
        if (EX_PCSel == 2'b01) begin//分支指令
          PC<=EX_PC+EX_immout;
        end else if (EX_PCSel == 2'b10) begin//跳转指令
          PC<=(EX_i_jalr)?(EX_ALUout&32'hFFFFFFFE):EX_ALUout;//jalr需要将最低地址置0
        end
      end else begin
        if(!stall)begin//没有冒险暂停
        PC <= PC + 4;
        end
      end
    end
  end

  //分支决策在EX阶段进行
  always@(*)begin//当检测到分支跳转时，刷新IF/ID和ID/EX流水线及寄存器
    if (EX_PCSel==2'b01 || EX_PCSel == 2'b10 ) begin
        IF_ID_write_enable<=1;//IF/ID阶段写入
        IF_ID_flush  <= 1;
        ID_EX_flush  <= 1;
    end
    else if(stall)begin//数据冒险发生
        IF_ID_write_enable<=0;//暂停取值
        ID_EX_flush  <= 1;//插入气泡
    end
    else begin
        IF_ID_write_enable<=1;
        IF_ID_flush  <= 0;
        ID_EX_flush  <= 0;
    end
  end

endmodule
