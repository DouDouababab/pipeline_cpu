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
    end else begin//��֧��ת����
      if (EX_PCSel==2'b01 || EX_PCSel == 2'b10) begin
        if (EX_PCSel == 2'b01) begin//��ָ֧��
          PC<=EX_PC+EX_immout;
        end else if (EX_PCSel == 2'b10) begin//��תָ��
          PC<=(EX_i_jalr)?(EX_ALUout&32'hFFFFFFFE):EX_ALUout;//jalr��Ҫ����͵�ַ��0
        end
      end else begin
        if(!stall)begin//û��ð����ͣ
        PC <= PC + 4;
        end
      end
    end
  end

  //��֧������EX�׶ν���
  always@(*)begin//����⵽��֧��תʱ��ˢ��IF/ID��ID/EX��ˮ�߼��Ĵ���
    if (EX_PCSel==2'b01 || EX_PCSel == 2'b10 ) begin
        IF_ID_write_enable<=1;//IF/ID�׶�д��
        IF_ID_flush  <= 1;
        ID_EX_flush  <= 1;
    end
    else if(stall)begin//����ð�շ���
        IF_ID_write_enable<=0;//��ͣȡֵ
        ID_EX_flush  <= 1;//��������
    end
    else begin
        IF_ID_write_enable<=1;
        IF_ID_flush  <= 0;
        ID_EX_flush  <= 0;
    end
  end

endmodule
