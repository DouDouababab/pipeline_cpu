`timescale 1ns / 1ps


module top (
    input clk,
    input rstn,
    input [15:0] sw_i,
    input [4:0] btn_i,
    output [7:0] disp_seg_o,
    output [7:0] disp_an_o
);

  wire rst = !rstn;
  //wire pause = SW_out[15];
  wire [4:0] BTN_out;
  wire [15:0] SW_out;
  //wire Clk_CPU = pause ? 1'b0 : Clk_addr;  //SW_out[15]控制CPU时钟是否暂停
  wire Clk_CPU; 
  wire [31:0] clkdiv;

  //SPIO
  wire [1:0] counter_set;
  wire [15:0] LED_out;
  wire [15:0] led;
  wire [13:0] GPIOf0;

  //Counter_x
  wire clk0 = clkdiv[6];
  wire clk1 = clkdiv[9];
  wire clk2 = clkdiv[11];
  wire [1:0] counter_ch = counter_set;
  wire [31:0] counter_val = CPU2IO;
  wire counter_we;
  wire counter0_OUT;
  wire counter1_OUT;
  wire counter2_OUT;
  wire [31:0] counter_out;

  //SCPU
  wire [4:0] reg_disp_addr = SW_out[13:9]; //寄存器显示地址
  wire [31:0] reg_disp_data;
  wire [31:0] Data_in = Data_read;
  wire INT = counter0_OUT;
  wire MIO_ready;
  wire CPU_MIO = MIO_ready;
  wire [31:0] inst_in = data;
  wire [31:0] Addr_out;
  wire [31:0] Data_out;
  wire [31:0] PC_out;
  wire [2:0] dm_ctrl;
  wire mem_w;

  //dm_controller
  wire [31:0] Addr_in = Addr_out;
  wire [31:0] Data_read_from_dm = Cpu_data4bus;// 从RAM读出的数据，写给cpu
  wire [31:0] Data_write = ram_data_in;
  wire [31:0] Data_read;//发送给cpu
  wire [31:0] Data_write_to_dm; //将对齐后的数据写入RAM
  wire [3:0] wea_mem;//写使能信号 控制RAM的四个字节

  //RAM_B
  wire [9:0] addra = ram_addr;
  wire clka = !clk;//mind this
  wire [31:0] dina = Data_write_to_dm;
  wire [3:0] wea = wea_mem;
  wire [31:0] douta;

  //MIO_BUS
  wire [4:0] BTN = BTN_out;
  wire [31:0] Cpu_data2bus = Data_out; //从CPU发出的数据
  wire [15:0] SW = SW_out;
  wire [31:0] addr_bus = Addr_out;
  wire counter0_out = counter0_OUT;
  wire counter1_out = counter1_OUT;
  wire counter2_out = counter2_OUT;
  wire [15:0] led_out = LED_out;
  wire [31:0] ram_data_out = douta;
  wire data_ram_we;
  wire [31:0] Cpu_data4bus;
  wire GPIOEO;
  wire GPIOFO;
  wire [31:0] CPU2IO;
  wire [9:0] ram_addr;
  wire [31:0] ram_data_in;

 //ROM_D
  wire [9:0] addr = PC_out[11:2];
  wire [31:0] data;

  SCPU U1_SCPU (
      .clk( Clk_CPU ),
      .rst(rst),
      .MIO_ready(MIO_ready),
      .instr(inst_in),
      .Data_in(Data_in),
      .INT(INT),
      .reg_disp_addr(reg_disp_addr),
      .mem_w(mem_w),
      .PC_out(PC_out),
      .Addr_out(Addr_out),
      .Data_out(Data_out),
      .dm_ctrl(dm_ctrl),
      .CPU_MIO(CPU_MIO),
      .reg_disp_data(reg_disp_data)
  );

  ROM_D U2_ROMD (
      .a  (addr),
      .spo(data)
  );

  dm_controller U3_dm_controller (
      .mem_w(mem_w),
      .Addr_in(Addr_in),
      .Data_write(Data_write),
      .dm_ctrl(dm_ctrl),
      .Data_read_from_dm(Data_read_from_dm),
      .Data_read(Data_read),
      .Data_write_to_dm(Data_write_to_dm),
      .wea_mem(wea_mem)
  );

  RAM_B U3_RAM_B (
      .addra(addra),
      .clka (clka /* & ~pause */), //暂停时不更新RAM
      .dina (dina),
      .wea  (wea),
      .douta(douta)
  );

  MIO_BUS U4_MIO_BUS (
      .clk(clk /* & ~pause */), //暂停时不更新MIO_BUS
      .rst(rst),
      .BTN(BTN),
      .SW(SW),
      .mem_w(mem_w),
      .Cpu_data2bus(Cpu_data2bus),
      .addr_bus(addr_bus),
      .ram_data_out(ram_data_out),
      .led_out(led_out),
      .counter_out(counter_out),
      .counter0_out(counter0_out),
      .counter1_out(counter1_out),
      .counter2_out(counter2_out),
      .PC(PC_out),
      .Cpu_data4bus(Cpu_data4bus),
      .ram_data_in(ram_data_in),
      .ram_addr(ram_addr),
      .data_ram_we(data_ram_we),
      .GPIOf0000000_we(GPIOFO),
      .GPIOe0000000_we(GPIOEO),
      .counter_we(counter_we),
      .Peripheral_in(CPU2IO)
  );

  //Multi_8CH32
  wire [63:0] LES = 0;
  wire [31:0] Disp_num;
  wire [7:0] LE_out;
  wire [7:0] point_out;
  Multi_8CH32 U5_Multi_8CH32 (
      .clk(/* Clk_CPU */clk),
      .rst(rst),
      .EN(GPIOEO),
      .Switch(SW_out[7:5]),
      .point_in({clkdiv[31:0], clkdiv[31:0]}),
      .LES(~64'h00000000),
      .data0(CPU2IO),
      .data1({1'b0, 1'b0, PC_out[31:2]}),
      .data2(inst_in),
      //.data3(counter_out),
      .data3((reg_disp_addr == 0) ? 32'h00000000 : reg_disp_data),
      .data4(Addr_out),
      .data5(Data_out),
      .data6(Cpu_data4bus),
      .data7(PC_out),
      .point_out(point_out),
      .LE_out(LE_out),
      .Disp_num(Disp_num)
  );

  //SSeg7
  SSeg7 U6_SSeg7 (
      .clk(clk /* & ~pause */), //暂停时不更新SSeg7
      .rst(rst),
      .SW0(SW_out[0]),
      .flash(clkdiv[10]),
    /*   .pause(SW_out[15]), */
      .Hexs(Disp_num),
      .point(point_out),
      .LES(LE_out),
      .seg_an(disp_an_o),
      .seg_sout(disp_seg_o)
  );

    SPIO U7_SPIO (
      .clk(!Clk_CPU),
      .rst(rst),
      .EN(GPIOFO),
      .P_Data(CPU2IO),
      .counter_set(counter_set),
      .LED_out(LED_out),
      .led(led),
      .GPIOf0(GPIOf0)
  );

  clk_div U8_clk_div (
      .clk(clk /* & ~pause */), //暂停时不更新clkdiv
      .rst(rst),
      .SW2(SW_out[2]),
      .clkdiv(clkdiv),
      .Clk_CPU(Clk_CPU)
  );

  Counter_x U9_Counter_x (
      .clk(/* !Clk_CPU */!Clk_addr),
      .rst(rst),
      .clk0(clk0),
      .clk1(clk1),
      .clk2(clk2),
      .counter_we(counter_we),
      .counter_val(counter_val),
      .counter_ch(counter_ch),
      .counter0_OUT(counter0_OUT),
      .counter1_OUT(counter1_OUT),
      .counter2_OUT(counter2_OUT),
      .counter_out(counter_out)
  );

  Enter U10_Enter (
      .clk(clk/*  & ~pause */), //暂停时不更新Enter
      .BTN(btn_i),
      .SW(sw_i),
      .BTN_out(BTN_out),
      .SW_out(SW_out)
  );

endmodule
  //寄存器显示



