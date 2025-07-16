`include "def.v"

module SCPU (
    input clk,
    input rst,
    input MIO_ready,
    input [31:0] instr,  //inst_in,instruction
    input [31:0] Data_in,  // 从内存中读取的数据
    output mem_w, // 是否写内存
    output [31:0] PC_out,
    output [31:0] Addr_out,  // 地址输出
    output [31:0] Data_out,  // 要写入内存的数据 rs2
    output [2:0] dm_ctrl,
    output CPU_MIO,
    input INT,
    input [4:0] reg_disp_addr,
    output [31:0] reg_disp_data
);

  //IF阶段
  //PC update
  reg [31:0] PC;
  reg [31:0] cycles;
  reg IF_ID_flush = 0;
  reg ID_EX_flush = 0;
  reg EX_MEM_flush = 0;
  reg IF_ID_write_enable=1;
  wire [1:0] PCSel;//PC选择信号
  wire i_jalr;
  wire MEM_PC = EX_MEM_data_out[132:101];  //PC
  wire MEM_i_jalr = EX_MEM_data_out[167];  //i_jalr
  wire MEM_ALUout = EX_MEM_data_out[63:32];  //ALUout
  wire MEM_immout = EX_MEM_data_out[166:135];  //immout
  wire MEM_PCSel = EX_MEM_data_out[169:168];  //PCSel
  wire stall = load_Hazard;//数据冒险信号

  //IF/ID regs
  assign write_enable = 1;
  assign flush = 0;
  wire [63:0] IF_ID_data_in;
  wire [63:0] IF_ID_data_out;
  assign IF_ID_data_in[31:0]  = instr;  //31-0位为指令
  assign IF_ID_data_in[63:32] = PC;  //63-32位为PC
  
  //ID阶段
  wire [6:0] Op;
  wire [4:0] rd_addr;
  wire [6:0] Funct7;
  wire [2:0] Funct3;
  wire [4:0] rs1_addr;
  wire [4:0] rs2_addr;
  wire [31:0] ID_instr = IF_ID_data_out[31:0];
  wire [31:0] ID_PC = IF_ID_data_out[63:32];
  assign Op = ID_instr[6:0];
  assign rd_addr = ID_instr[11:7];
  assign Funct3 = ID_instr[14:12];
  assign rs1_addr = ID_instr[19:15];
  assign rs2_addr = ID_instr[24:20];
  assign Funct7 = ID_instr[31:25];

  //EXT imm extend unit
  wire [ 5:0] EXTOp;
  wire [31:0] immout;

  // RF
  wire RegWrite;
  wire [4:0] rs1;
  wire [4:0] rs2;
  wire [4:0] rd;
  reg [31:0] data_in;
  wire [31:0] rd1;
  wire [31:0] rd2;
  wire [1:0] WDSel;
  assign rs1 = rs1_addr;
  assign rs2 = rs2_addr;
  assign rd  = MEM_WB_data_out[11:7];

  //ID/EX regs
  wire [181:0] ID_EX_data_in;
  wire [181:0] ID_EX_data_out;
  assign ID_EX_data_in[31:0] = ID_instr;  //31-0位为指令
  assign ID_EX_data_in[63:32] = ID_PC;  //63-32位为PC
  assign ID_EX_data_in[95:64] = rd1;  //95-64位为rs1读出的数据
  assign ID_EX_data_in[127:96] = rd2;  //127-66位为rs2读出的数据
  assign ID_EX_data_in[159:128] = immout;  //159-128位为立即数
  assign ID_EX_data_in[160] = RegWrite;  //160位为RegWrite
  assign ID_EX_data_in[161] = DMWr;  //161位为DMWr
  assign ID_EX_data_in[163:162] = 2'b0;//PCSel;  //163-162位为PCSel
  assign ID_EX_data_in[165:164] = WDSel;  //165-164位为WDSel
  assign ID_EX_data_in[166] = ASource;  //166位为ASel 1bit 
  assign ID_EX_data_in[167] = BSource;  //167位为BSel 1bit
  assign ID_EX_data_in[172:168] = ALUop;  //172-168位为ALUop 5bit
  assign ID_EX_data_in[175:173] = DMType;  //175-173位为DMType 3bit
  assign ID_EX_data_in[176] = i_jalr;  //176位为i_jalr 1bit
  assign ID_EX_data_in[177] = 0;//BrUnsigned;  //177位为BrUn 1bit
  assign ID_EX_data_in[178] = 0;//BrLess;  //178位为zero 1bit
  assign ID_EX_data_in[179] = 0;//zero;  //179位为zero 1bit
  assign ID_EX_data_in[180] = u_lui;
  assign ID_EX_data_in[181] = MemRead;

  //EX阶段 
  wire [31:0] EX_instr = ID_EX_data_out[31:0];  //指令
  wire [31:0] EX_PC = ID_EX_data_out[63:32];  //PC
  wire [31:0] EX_RD1 = ID_EX_data_out[95:64];  //rs1数据
  wire [31:0] EX_RD2 = ID_EX_data_out[127:96];  //rs2数据
  wire [31:0] EX_immout = ID_EX_data_out[159:128];  //立即数
  wire EX_RegWrite = ID_EX_data_out[160];  //RegWrite
  wire EX_DMWr = ID_EX_data_out[161];  //DMWr
  wire EX_ASel = ID_EX_data_out[166];  //ASource
  wire EX_BSel = ID_EX_data_out[167];  //BSource
  wire [4:0] EX_ALUop = ID_EX_data_out[172:168];  //ALUop
  wire [2:0] EX_DMType = ID_EX_data_out[175:173];  //DMType
  wire EX_i_jalr = ID_EX_data_out[176];  //i_jalr
  wire EX_u_lui = ID_EX_data_out[180];
  wire [1:0]ID_EX_WDSel = ID_EX_data_out[165:164];  //WDSel

  //ALU
  wire [31:0] A;
  wire [31:0] B;
  wire [31:0] EX_ALUout;
  wire MEM_RegWrite = EX_MEM_data_out[96];  //RegWrite 
  wire [4:0] MEM_rd = EX_MEM_data_out[11:7];  //EX_MEM_rd
  wire [4:0] WB_rd = MEM_WB_data_out[11:7];  //MEM_WB_rd
  wire [4:0] EX_rs1 = ID_EX_data_out[19:15];  //ID_EX_rs1
  wire [4:0] EX_rs2 = ID_EX_data_out[24:20];  //ID_EX_rs2 
  wire [1:0] Forward_A;
  wire [1:0] Forward_B;
  wire zero = rd1 == rd2 ? 1'b1 : 1'b0;
  wire [4:0] ALUop;
  wire ASource;
  wire BSource;
  wire [2:0] DMType;
  wire MemWrite;
  wire u_lui;
  wire EX_BrUnsigned;
  wire EX_BrLess = EX_BrUnsigned ? (A < B) : (($signed(A)) < $signed(B));  //less than for blt bltu bge bgeu
  wire [1:0] EX_PCSel;

  //EX/MEM regs
  wire [169:0] EX_MEM_data_in;
  wire [169:0] EX_MEM_data_out;
  wire EX_MEM_WDSel = EX_MEM_data_out[134:133];
  reg [31:0] EX_Data_A;
  reg [31:0] EX_Data_B;
  reg [31:0] EX_MEM_forward_Data;
  reg [31:0] MEM_WB_forward_Data;
  assign EX_MEM_data_in[31:0] = EX_instr;  //31-0位为指令
  assign EX_MEM_data_in[63:32] = EX_ALUout;  // 63-32位为ALUout
  assign EX_MEM_data_in[95:64] = EX_Data_B;  //rs2数据,用于写入内存,由于前递的问题,这里选择经过前递单元后的数据
  assign EX_MEM_data_in[96] = EX_RegWrite;  //96位为RegWrite
  assign EX_MEM_data_in[97] = EX_DMWr;  //97位为DMWr
  assign EX_MEM_data_in[100:98] = EX_DMType;  //100-98位为DMType
  assign EX_MEM_data_in[132:101] = EX_PC;  //132-101位为PC
  assign EX_MEM_data_in[134:133] = ID_EX_WDSel;  //134-133位为WDSel
  assign EX_MEM_data_in[166:135] = EX_immout;
  assign EX_MEM_data_in[167] = EX_i_jalr;  //167位为i_jalr
  assign EX_MEM_data_in[169:168] = EX_PCSel;  //168-169位为PCSel

  //MEM 阶段
  wire [31:0] MEM_instr = EX_MEM_data_out[31:0];  //指令
  wire [63:32] EX_MEM_ALUout = EX_MEM_data_out[63:32];  //ALUout
  wire [31:0] MEM_RD2 = EX_MEM_data_out[95:64];  //rs2数据
  wire MEM_DMWr = EX_MEM_data_out[97];  //DMWr
  wire [2:0] MEM_DMType = EX_MEM_data_out[100:98];  //DMType
  wire [31:0] EX_MEM_PC = EX_MEM_data_out[132:101];  //PC
  wire [31:0] EX_MEM_immout = EX_MEM_data_out[166:135];  //immout

  //DM
  assign Addr_out = EX_MEM_ALUout;
  assign Data_out = MEM_RD2;
  assign mem_w = MEM_DMWr;
  assign dm_ctrl = MEM_DMType;

  //MEM/WB regs
  wire [163:0] MEM_WB_data_in;
  wire [163:0] MEM_WB_data_out;
  assign MEM_WB_data_in[31:0] = MEM_instr;  //31-0位为指令
  assign MEM_WB_data_in[63:32] = Data_in;  //63-32位为DM读出的数据
  assign MEM_WB_data_in[95:64] = EX_MEM_ALUout;  //95-64位为ALUout
  assign MEM_WB_data_in[96] = MEM_RegWrite;  //96位为RegWrite
  assign MEM_WB_data_in[128:97] = EX_MEM_PC;  //128-97位为PC
  assign MEM_WB_data_in[130:129] = EX_MEM_data_out[134:133];  //130-129位为WDSel
  assign MEM_WB_data_in[162:131] = EX_MEM_data_out[166:135];  // 162-131位为immout
  assign MEM_WB_data_in[163] = EX_MEM_data_out[167];  //163位为i_jalr
  wire [31:0] MEM_WB_DMout_data = MEM_WB_data_out[63:32];  //DM读出的数据
  wire [31:0] MEM_WB_ALUout = MEM_WB_data_out[95:64];  //ALUout
  wire [31:0] MEM_WB_Pc = MEM_WB_data_out[128:97];  //PC
  wire [1:0] MEM_WB_WDSel = MEM_WB_data_out[130:129];  //WDSel
  wire [31:0] MEM_WB_immout = MEM_WB_data_out[162:131];  //immout(for lui)
  wire RegWrite_MEM_out = MEM_WB_data_out[96];  //RegWrite
  wire MEM_WB_i_jalr = MEM_WB_data_out[163];
  assign CPU_MIO = 1'b0;
  assign INT = 1'b0;

  //寄存器转发
  assign A = (EX_ASel) ? EX_PC : EX_Data_A;  //第一个
  assign B = (EX_BSel) ? EX_immout : EX_Data_B;  //选择ALU的第二个输入数据
  wire EX_Zero;


  //Hazard Detection
  wire load_Hazard;
  wire [4:0] EX_rd = ID_EX_data_out[11:7];//EX阶段目标寄存器
  wire ID_EX_MemRead = ID_EX_data_out[181];//判断是否为load指令

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

  assign PC_out = PC; 


//pc作为模块可能有时序问题，指令可以正常显示，但跳转似乎有问题，将其直接写在cpu顶层模块中，可以解决
/* PC pc(
      .clk(clk),
      .rst(rst),
      .EX_PCSel(EX_PCSel),
      .EX_immout(EX_immout),
      .EX_ALUout(EX_ALUout),
      .EX_i_jalr(EX_i_jalr),
      .stall(stall),
      .EX_PC(EX_PC),
      .PC(PC_out),  // 直接输出到PC_out
      .IF_ID_flush(IF_ID_flush),
      .ID_EX_flush(ID_EX_flush),
      .IF_ID_write_enable(IF_ID_write_enable)
);  */
 
  Pipeline_reg #(
      .WIDTH(64)
  ) IF_ID (
      clk,
      rst,
      IF_ID_write_enable,
      IF_ID_flush,
      IF_ID_data_in,
      IF_ID_data_out
  );

   Pipeline_reg #(
      .WIDTH(182)
  ) ID_EX (
      clk,
      rst,
      write_enable,
      ID_EX_flush,//冒险控制
      ID_EX_data_in,
      ID_EX_data_out
  );

    Pipeline_reg #(
      .WIDTH(170)
  ) EX_MEM (
      clk,
      rst,
      write_enable,
      EX_MEM_flush,
      EX_MEM_data_in,
      EX_MEM_data_out
  );

  Pipeline_reg #(
      .WIDTH(164)
  ) MEM_WB (
      clk,
      rst,
      write_enable,
      flush,
      MEM_WB_data_in,
      MEM_WB_data_out
  );

  EXT ext (
      .iimm_shamt(ID_instr[24:20]),
      .iimm(ID_instr[31:20]),
      .simm({ID_instr[31:25], ID_instr[11:7]}),
      .bimm({ID_instr[31], ID_instr[7], ID_instr[30:25], ID_instr[11:8]}),
      .uimm(ID_instr[31:12]),
      .jimm({ID_instr[31], ID_instr[19:12], ID_instr[20], ID_instr[30:21]}),
      .EXTOp(EXTOp),
      .immout(immout)
  );

  RF rf (
      .clk (clk),
      .rst (rst),
      .RFWrite(RegWrite_MEM_out),//from WB
      .addr1  (rs1),
      .addr2  (rs2),
      .addr3  (rd),
      .data_in  (data_in),
      .rd1 (rd1),
      .rd2 (rd2)
  );

  //Control Unit
  Ctrl ctrl (
      .Op(Op),
      .Funct3(Funct3),
      .Funct7(Funct7),
      .RegWrite(RegWrite),
      .MemWrite(DMWr),
      .EXTOp(EXTOp),
      .ALUOp(ALUop),
      .ASource(ASource),
      .BSource(BSource),
      .DMType(DMType),
      .WDSel(WDSel),//写回数据选择信号
      .i_jalr(i_jalr),
      .u_lui(u_lui),
      .MemRead(MemRead)
  );

  ALU alu (
      .A(A),
      .B(B),
      .ALUop(EX_ALUop),
      .ALUout(EX_ALUout),
      .Zero(EX_Zero)
  );
  
  HazardDetection u_hazard (
      .ID_EX_MemRead(ID_EX_MemRead),
      .EX_rd(EX_rd),
      .ID_rs1(rs1),
      .ID_rs2(rs2),
      .load_Hazard(load_Hazard)
  );

  //Write Data to reg select
  always @(*) begin
    case (MEM_WB_WDSel)
      `WDSel_FromALU: data_in = MEM_WB_ALUout;
      `WDSel_FromMEM: data_in = MEM_WB_DMout_data;
      `WDSel_FromPC:  data_in = MEM_WB_Pc + 4;
    endcase
  end

  Forward forward (
      .MEM_RegWrite(MEM_RegWrite),
      .MEM_rd(MEM_rd),
      .WB_RegWrite(RegWrite_MEM_out),
      .WB_rd(WB_rd),
      .EX_rs1(EX_rs1),
      .EX_rs2(EX_rs2),
      .Forward_A(Forward_A),
      .Forward_B(Forward_B)
  );

  //选择从EX/MEM寄存器中要前递的数据
  always @(*) begin
    case (EX_MEM_WDSel)
      `WDSel_FromALU: begin
        EX_MEM_forward_Data = EX_MEM_ALUout;
      end
      `WDSel_FromPC: begin
        EX_MEM_forward_Data = EX_MEM_PC + 4;
      end
    endcase
  end

  always @(*) begin
    case (MEM_WB_WDSel)
      `WDSel_FromALU: begin
        MEM_WB_forward_Data = MEM_WB_ALUout;
      end
      `WDSel_FromMEM: begin
        MEM_WB_forward_Data = MEM_WB_DMout_data;
      end
      `WDSel_FromPC: begin
        MEM_WB_forward_Data = MEM_WB_Pc + 4;
      end
    endcase
  end

  //00 正常从寄存器读取
  //01 从EX_MEM读取
  //10 从MEM_WB读取
  always @(*) begin
    if (EX_u_lui) begin//lui指令处理
      EX_Data_A = 0;
    end else begin
      case (Forward_A)
        2'b00:   EX_Data_A = EX_RD1;
        2'b01:   EX_Data_A = EX_MEM_forward_Data;
        2'b10:   EX_Data_A = MEM_WB_forward_Data;
        default: EX_Data_A = EX_RD1;
      endcase
    end
    case (Forward_B)
      2'b00:   EX_Data_B = EX_RD2;
      2'b01:   EX_Data_B = EX_MEM_forward_Data;
      2'b10:   EX_Data_B = MEM_WB_forward_Data;
      default: EX_Data_B = EX_RD2;
    endcase
  end

  //Branch Unit
  Branch branch (
      .Op(EX_instr[6:0]),
      .Funct3(EX_instr[14:12]),
      .Funct7(EX_instr[31:25]),
      .BrLess(EX_BrLess),
      .Zero(EX_Zero),
      .BrUnsigned(EX_BrUnsigned),
      .PCSel(EX_PCSel)
  );


endmodule
