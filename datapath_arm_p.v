//====================================================================
// datapath_arm_p.v   (ARMv4 5-stage pipelined CPU, step 4)  -  pipelined datapath (arm_p_spec.md section 5)
//
//   Harris & Harris pipelined datapath (section 7.5.1) with its pipeline registers, the control pipeline registers of
//   section 7.5.2 (E, M, W), the Conditional Logic in Execute and the forwarding multiplexers of section 7.5.3:
//     F : PC register, PCPlus4F, PCNext mux (ResultW / ALUResultE / PCPlus4F)
//     D : register file (regfile_arm_p, read addresses RA1D..RA3D, R15 = PCPlus4F), Extend value from the controller
//     E : forwarding muxes (RD1E / RD2E / RD3E, ALUResultM, ResultW), shifter, multiplier, ALU, Conditional Logic,
//         branch target, memory address / store data (to the M9K input registers = E/M register), link value,
//         HoldE / Creg / Zlo (uop registers, spec section 6)
//     M : ReadDataM (aligner output of the memory) -> M/W register
//     W : ResultW mux, register-file write, PC write (PCSrcW), halt / fault
//   The instruction and data memories are outside (cpu_core_arm_p.v). The D-stage control comes from controller_arm_p,
//   the stall / flush / forward signals from hazard_arm_p.
//
//   Control registers of E: loaded every cycle; validE = validD & ~FlushE (H&H 'clear' of the E register): every write
//   in E, M, W is ANDed with valid, so a flushed slot does nothing. RegWriteM / MemWrite / PCSrcM / flags are gated
//   by CondExE (H&H: they come out of the Conditional Logic).
//   SrcASel 0 RD1 (forwarded)  1 HoldE  2 zero        SrcBSel 0 shifter  1 ExtImmE  2 product[31:0]
//   CarrySel 0 adder  1 shifter  2 immediate bit 31  3 old C        HoldSel 0 none  1 ALUResult  2 SrcA  3 product[63:32]
//   MemAdrSel 1 ALUResultE (pre-index), 0 SrcA (post-index)
//====================================================================
module datapath_arm_p #(
    parameter SH_EN    = 1,        // barrel shifter          (FEATURES[1])
    parameter SHREG_EN = 1,        // shift amount from Rs    (FEATURES[2])
    parameter MUL_EN   = 1,        // MUL / MLA               (FEATURES[8] | FEATURES[9])
    parameter MULL_EN  = 1         // long multiplies         (FEATURES[9])
) (
    input         clk,
    input         rst,
    // ---------------- Fetch ----------------
    input         pc_en,           // ~StallF | BranchTakenE | PCSrcW
    output [31:0] PCF,
    output [31:0] PCPlus4F,
    // ---------------- Decode (from the controller) ----------------
    input         validD,
    input  [31:0] InstrD,
    input  [3:0]  RA1D, RA2D, RA3D, WA3D,
    input         RegWriteD, PCSD, MemtoRegD, MemWriteD, MemSignD, MemAdrSelD,
    input  [1:0]  MemSizeD,
    input  [3:0]  ALUControlD,
    input  [1:0]  SrcASelD, SrcBSelD, CarrySelD, HoldSelD,
    input         ShByRegD, ShDisD, CinSelD, ZloUseD, CregWD, MulSignedD,
    input  [2:0]  FlagWriteD,
    input         BranchD, LinkD, BXD, SwiD, BselfD, IllegalD, LastD,
    input  [31:0] ExtImmD,
    // ---------------- hazard unit ----------------
    input         FlushE,
    input  [1:0]  Fwd1E, Fwd2E, Fwd3E,       // 0 RDxE, 1 ResultW, 2 ALUResultM
    output reg [3:0] RA1E, RA2E, RA3E, WA3E,
    output reg    validE,
    output        MemtoRegE_o,
    output        PCSE_o,          // validE & PCSE (before the condition, [HH] PCSrcE of the hazard unit)
    output        BranchTakenE,
    output        StopE,
    output reg [3:0] WA3M, WA3W,
    output reg    RegWriteM, RegWriteW, PCSrcM, PCSrcW,
    output reg    validM, validW,
    // ---------------- memory (E -> M input registers, M output) ----------------
    output [31:0] MemAdrE,
    output        MemWriteE_g,     // validE & MemWriteE & CondExE
    output [1:0]  MemSizeE_o,
    output        MemSignE_o,
    output [31:0] WriteDataE,
    input  [31:0] ReadDataM,
    // ---------------- status ----------------
    output [3:0]  flags,
    output reg    halted,
    output reg    fault,
    output reg [31:0] StopPC,
    output        retire,          // validW & LastW: one instruction finished (condition-failed and stop ones included)
    output [31:0] ResultW,
    // ---------------- board view: register-file port 1 ----------------
    input         view,
    input  [3:0]  dbg_ra,
    output [31:0] rd1_raw
);
    // power-up values of the output registers (the reset clears the ones that matter)
    initial begin
        RA1E = 4'd0; RA2E = 4'd0; RA3E = 4'd0; WA3E = 4'd0; validE = 1'b0; WA3M = 4'd0; WA3W = 4'd0;
        RegWriteM = 1'b0; RegWriteW = 1'b0; PCSrcM = 1'b0; PCSrcW = 1'b0; validM = 1'b0; validW = 1'b0;
        halted = 1'b0; fault = 1'b0; StopPC = 32'd0;
    end

    // =====================================================================================================  Fetch
    reg  [31:0] pc_r = 32'd0;
    reg  [31:0] ALUResultM = 32'd0, ALUResultW = 32'd0, ReadDataW = 32'd0;
    reg         MemtoRegW = 1'b0;
    wire [31:0] ALUResultE;
    assign PCF      = pc_r;
    assign PCPlus4F = pc_r + 32'd4;
    assign ResultW  = MemtoRegW ? ReadDataW : ALUResultW;
    wire [31:0] PCNext = PCSrcW ? ResultW : BranchTakenE ? ALUResultE : PCPlus4F;
    always @(posedge clk) begin
        if (rst)        pc_r <= 32'd0;
        else if (pc_en) pc_r <= {PCNext[31:2], 2'b00};
    end

    // =====================================================================================================  Decode
    wire [31:0] RD1E, RD2E, RD3E;
    regfile_arm_p rf (.clk(clk), .rst(rst), .ra1(view ? dbg_ra : RA1D), .ra2(RA2D), .ra3(RA3D), .pc8(PCPlus4F),
                      .rd1(RD1E), .rd2(RD2E), .rd3(RD3E), .we(RegWriteW), .wa(WA3W), .wd(ResultW));
    assign rd1_raw = RD1E;

    // ---------------- D/E pipeline register ----------------
    reg        RegWriteE = 1'b0, PCSE = 1'b0, MemtoRegE = 1'b0, MemWriteE = 1'b0, MemSignE = 1'b0, MemAdrSelE = 1'b0;
    reg [1:0]  MemSizeE = 2'd0, SrcASelE = 2'd0, SrcBSelE = 2'd0, CarrySelE = 2'd0, HoldSelE = 2'd0;
    reg [3:0]  ALUControlE = 4'd0, CondE = 4'd0;
    reg        ShByRegE = 1'b0, ShDisE = 1'b0, CinSelE = 1'b0, ZloUseE = 1'b0, CregWE = 1'b0, MulSignedE = 1'b0;
    reg [2:0]  FlagWriteE = 3'd0;
    reg        BranchE = 1'b0, LinkE = 1'b0, BXE = 1'b0, SwiE = 1'b0, BselfE = 1'b0, IllegalE = 1'b0, LastE = 1'b0;
    reg [31:0] ExtImmE = 32'd0, PCPlus8E = 32'd0;
    reg [1:0]  ShTypE = 2'd0;
    reg [4:0]  ShAmtE = 5'd0;
    always @(posedge clk) begin
        validE      <= ~rst & validD & ~FlushE;
        RA1E <= RA1D; RA2E <= RA2D; RA3E <= RA3D; WA3E <= WA3D;
        RegWriteE   <= RegWriteD;   PCSE <= PCSD;   MemtoRegE <= MemtoRegD;   MemWriteE <= MemWriteD;
        MemSignE    <= MemSignD;    MemAdrSelE <= MemAdrSelD;   MemSizeE <= MemSizeD;
        SrcASelE    <= SrcASelD;    SrcBSelE <= SrcBSelD;   CarrySelE <= CarrySelD;   HoldSelE <= HoldSelD;
        ALUControlE <= ALUControlD; CondE <= InstrD[31:28];
        ShByRegE    <= ShByRegD;    ShDisE <= ShDisD;   CinSelE <= CinSelD;   ZloUseE <= ZloUseD;
        CregWE      <= CregWD;      MulSignedE <= MulSignedD;   FlagWriteE <= FlagWriteD;
        BranchE     <= BranchD;     LinkE <= LinkD;   BXE <= BXD;   SwiE <= SwiD;   BselfE <= BselfD;
        IllegalE    <= IllegalD;    LastE <= LastD;
        ExtImmE     <= ExtImmD;     PCPlus8E <= PCPlus4F;
        ShTypE      <= InstrD[6:5]; ShAmtE <= InstrD[11:7];
    end

    // =====================================================================================================  Execute
    // ---------------- forwarding ([HH] 7.5.3) ----------------
    wire [31:0] F1 = (Fwd1E == 2'd2) ? ALUResultM : (Fwd1E == 2'd1) ? ResultW : RD1E;
    wire [31:0] F2 = (Fwd2E == 2'd2) ? ALUResultM : (Fwd2E == 2'd1) ? ResultW : RD2E;
    wire [31:0] F3 = (Fwd3E == 2'd2) ? ALUResultM : (Fwd3E == 2'd1) ? ResultW : RD3E;

    // ---------------- Conditional Logic ----------------
    wire [3:0] nzcv_in;
    wire       CondExE;
    condlogic_arm_p cl (.clk(clk), .rst(rst), .en(validE), .cond(CondE), .nzcv_in(nzcv_in), .FlagW(FlagWriteE),
                        .CondEx(CondExE), .flags(flags));
    wire Cflag = flags[1];

    // ---------------- shifter, multiplier ----------------
    wire [31:0] Shifted;
    wire        ShC;
    shifter_arm #(.SH_EN(SH_EN), .REG_EN(SHREG_EN)) shf (
        .din(F2), .typ(ShTypE), .amt_imm(ShAmtE), .amt_reg(F3[7:0]), .by_reg(ShByRegE), .dis(ShDisE),
        .cin(Cflag), .dout(Shifted), .cout(ShC));
    wire [63:0] Prod;
    mul_arm_p #(.MUL_EN(MUL_EN), .LONG_EN(MULL_EN)) mul (.a(F2), .b(F3), .sgn(MulSignedE), .p(Prod));

    // ---------------- uop registers (spec section 6) ----------------
    reg [31:0] HoldE = 32'd0;
    reg        Creg = 1'b0, Zlo = 1'b0;

    // ---------------- ALU ----------------
    wire [31:0] SrcAE = (SrcASelE == 2'd0) ? F1 : (SrcASelE == 2'd1) ? HoldE : 32'd0;
    wire [31:0] SrcBE = (SrcBSelE == 2'd0) ? Shifted : (SrcBSelE == 2'd1) ? ExtImmE : Prod[31:0];
    wire        aN, aZ, aC, aV;
    alu_arm alu (.A(SrcAE), .B(SrcBE), .ALUControl(ALUControlE), .Cin(CinSelE ? Creg : Cflag), .Result(ALUResultE),
                 .N(aN), .Z(aZ), .Cadd(aC), .Vadd(aV));
    wire CarryE = (CarrySelE == 2'd0) ? aC : (CarrySelE == 2'd1) ? ShC : (CarrySelE == 2'd2) ? ExtImmE[31] : Cflag;
    assign nzcv_in = {aN, ZloUseE ? (aZ & Zlo) : aZ, CarryE, aV};

    always @(posedge clk) if (validE) begin
        case (HoldSelE)
            2'd1: HoldE <= ALUResultE;
            2'd2: HoldE <= F1;
            2'd3: HoldE <= Prod[63:32];
            default: ;
        endcase
        if (CregWE) begin Creg <= aC; Zlo <= aZ; end
    end

    // ---------------- branch, stop, memory, result ----------------
    assign BranchTakenE = validE & BranchE & CondExE;
    wire   bx_odd  = BXE & ALUResultE[0];
    assign StopE   = validE & (IllegalE | (CondExE & (SwiE | BselfE | bx_odd)));
    wire   StopFaultE = IllegalE | bx_odd;
    assign MemAdrE     = MemAdrSelE ? ALUResultE : F1;
    assign MemWriteE_g = validE & MemWriteE & CondExE;
    assign MemSizeE_o  = MemSizeE;
    assign MemSignE_o  = MemSignE;
    assign WriteDataE  = F3;
    assign MemtoRegE_o = MemtoRegE;
    assign PCSE_o      = validE & PCSE;
    wire [31:0] PCPlus4E = PCPlus8E - 32'd4;                // address of the instruction + 4 (BL link, stop PC)
    wire [31:0] ExResultE = LinkE ? PCPlus4E : ALUResultE;

    // =====================================================================================================  E/M register
    reg MemtoRegM = 1'b0, LastM = 1'b0, StopM = 1'b0, StopFaultM = 1'b0;
    reg [31:0] StopPCM = 32'd0;
    always @(posedge clk) begin
        validM     <= ~rst & validE;
        RegWriteM  <= ~rst & validE & RegWriteE & CondExE;
        PCSrcM     <= ~rst & validE & PCSE & CondExE;
        MemtoRegM  <= MemtoRegE;
        WA3M       <= WA3E;
        ALUResultM <= ExResultE;
        LastM      <= LastE;
        StopM      <= ~rst & StopE;
        StopFaultM <= StopFaultE;
        StopPCM    <= BselfE & ~IllegalE ? PCPlus8E - 32'd8 : PCPlus4E;
    end

    // =====================================================================================================  M/W register
    reg LastW = 1'b0, StopW = 1'b0, StopFaultW = 1'b0;
    reg [31:0] StopPCW = 32'd0;
    always @(posedge clk) begin
        validW     <= ~rst & validM;
        RegWriteW  <= ~rst & RegWriteM;
        PCSrcW     <= ~rst & PCSrcM;
        MemtoRegW  <= MemtoRegM;
        WA3W       <= WA3M;
        ALUResultW <= ALUResultM;
        ReadDataW  <= ReadDataM;
        LastW      <= LastM;
        StopW      <= ~rst & StopM;
        StopFaultW <= StopFaultM;
        StopPCW    <= StopPCM;
    end
    assign retire = validW & LastW;

    // =====================================================================================================  Writeback: stop
    always @(posedge clk) begin
        if (rst) begin
            halted <= 1'b0; fault <= 1'b0;
        end else if (StopW) begin
            halted <= ~StopFaultW; fault <= StopFaultW; StopPC <= StopPCW;
        end
    end
endmodule
