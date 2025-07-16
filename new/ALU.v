`include "def.v"

module ALU (
    input signed [31:0] A,B,
    input [4:0] ALUop,
    output reg signed [31:0] ALUout,
    output reg Zero
);

  always @(*) begin
    case (ALUop)
      `ALUOp_nop: ALUout <= A + B;
      `ALUOp_lui: ALUout <= B;
      `ALUOp_add: ALUout <= A + B;
      `ALUOp_sub: ALUout <= A - B;
      `ALUOp_slt: ALUout <= A < B ? 1 : 0;
      `ALUOp_sltu: ALUout <= $unsigned(A) < $unsigned(B) ? 1 : 0;
      `ALUOp_xor: ALUout <= A ^ B;
      `ALUOp_or: ALUout <= A | B;
      `ALUOp_and: ALUout <= A & B;
      `ALUOp_sll: ALUout <= A << B[4:0];
      `ALUOp_srl: ALUout <= A >> B[4:0];
      `ALUOp_sra: ALUout <= A >>> B[4:0];
      default: ALUout <= 32'b0;
    endcase
    Zero = ALUout == 0 ? 1 : 0;
  end

endmodule
