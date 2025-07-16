module RF (
    input clk,
    input rst,
    input RFWrite,
    input [4:0] addr1,
    input [4:0] addr2,
    input [4:0] addr3,
    input [31:0] data_in,
    output [31:0] rd1,
    output [31:0] rd2
);

  reg [31:0] rf[31:0];

  integer i;
  always @(negedge clk or posedge rst) begin
    if (rst) begin
      for (i = 0; i < 32; i = i + 1) begin
        rf[i] <= 0;
      end
    end else begin
      if (RFWrite) begin
        if (addr3 != 0) begin
          rf[addr3] <= data_in;
        end
      end
    end
  end

  assign rd1 = rf[addr1];
  assign rd2 = rf[addr2];



endmodule
