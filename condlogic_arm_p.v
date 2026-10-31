//====================================================================
// condlogic_arm_p.v   (ARMv4 5-stage pipelined CPU, step 4)  -  Conditional Logic of the Execute stage
//
//   Harris & Harris pipelined processor (section 7.5.2): the Conditional Logic sits in the Execute stage. It checks
//   CondE against the Flags register (CondExE) and writes the flags at the end of Execute when FlagWriteE & CondExE.
//   A following instruction in Execute therefore always sees the flags of the previous one (no flag hazard).
//   cond = 1111 -> CondEx = 0 (illegal encoding, the decoder reports it).
//   'en' = validE (a bubble writes nothing). FlagW[2] N and Z, FlagW[1] C, FlagW[0] V.
//====================================================================
module condlogic_arm_p (
    input        clk,
    input        rst,
    input        en,            // validE
    input  [3:0] cond,          // CondE
    input  [3:0] nzcv_in,       // {N, Z, C, V} of the Execute stage
    input  [2:0] FlagW,         // FlagWriteE
    output       CondEx,        // CondExE
    output [3:0] flags
);
    reg [3:0] nzcv = 4'b0000;
    wire N = nzcv[3], Z = nzcv[2], C = nzcv[1], V = nzcv[0];
    reg ex;
    always @(*) begin
        case (cond)
            4'b0000: ex = Z;              4'b0001: ex = ~Z;
            4'b0010: ex = C;              4'b0011: ex = ~C;
            4'b0100: ex = N;              4'b0101: ex = ~N;
            4'b0110: ex = V;              4'b0111: ex = ~V;
            4'b1000: ex = C & ~Z;         4'b1001: ex = ~C | Z;
            4'b1010: ex = (N == V);       4'b1011: ex = (N != V);
            4'b1100: ex = ~Z & (N == V);  4'b1101: ex = Z | (N != V);
            4'b1110: ex = 1'b1;           default: ex = 1'b0;
        endcase
    end
    assign CondEx = ex;
    assign flags  = nzcv;
    wire [2:0] fw = FlagW & {3{ex & en}};
    always @(posedge clk) begin
        if (rst) nzcv <= 4'b0000;
        else begin
            if (fw[2]) nzcv[3:2] <= nzcv_in[3:2];
            if (fw[1]) nzcv[1]   <= nzcv_in[1];
            if (fw[0]) nzcv[0]   <= nzcv_in[0];
        end
    end
endmodule
