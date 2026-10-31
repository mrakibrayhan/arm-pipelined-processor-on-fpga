//====================================================================
// controller_arm_p.v   (ARMv4 5-stage pipelined CPU, step 5)  -  Decode-stage control unit + uop sequencer
//
//   Harris & Harris pipelined control (section 7.5.2): the instruction is decoded in Decode and the control signals
//   travel down the pipeline with it (the E, M, W control registers are in datapath_arm_p.v). Legality comes from the
//   UNCHANGED single-cycle decoder (decoder_arm.v, FEATURES), so all three ARM cores run exactly the same subset.
//
//   Micro-operations ([FUR] Ch. 4, arm_p_spec.md section 6): an instruction that needs two register writes or several
//   memory beats stays in Decode and sends one uop per cycle to Execute ('SeqMore' = more uops follow: StallD/StallF,
//   but no FlushE). 'adv' = the current uop leaves Decode (validD & ~ldrStall); 'kill' = KillD (branch / stop in E).
//     LDR/STR(B/H) with write-back : u0 access (HoldE <- Rn +/- off), u1 Rn <- HoldE
//     UMULL/SMULL/UMLAL/SMLAL      : u0 RdLo <- RdLo(or 0) + P[31:0] (HoldE <- P[63:32], Creg, Zlo), u1 RdHi <- HoldE + RdHi(or 0) + Creg
//     LDM/STM                      : one beat per register (ldmstm_seq_arm_p), then Rn <- HoldE +/- 4N if W
//
//   Register ports: RA1 Rn, RA2 Rm, RA3 Rs or Rd (store data). use1..3 = the uop really reads that port (hazard unit).
//   R15 is never reported as 'used' (it is the PC, it can't be a hazard).
//   Encodings: SrcASel 0 RD1  1 HoldE  2 zero;  SrcBSel 0 shifter  1 ExtImm  2 product;  CarrySel 0 adder  1 shifter
//              2 ExtImm[31]  3 old C;  HoldSel 0 -  1 ALUResult  2 SrcA  3 product[63:32];  MemSize 0 word 1 half 2 byte
//====================================================================
module controller_arm_p #(
    parameter [10:0] FEATURES = 11'h7FF
) (
    input             clk,
    input             rst,
    input      [31:0] InstrD,
    input             validD,
    input             adv,
    input             kill,
    // register ports
    output reg [3:0]  RA1D, RA2D, RA3D, WA3D,
    output            use1D, use2D, use3D,
    // uop control (to the D/E register)
    output            RegWriteD, PCSD, MemWriteD,
    output reg        MemtoRegD, MemSignD, MemAdrSelD,
    output reg [1:0]  MemSizeD,
    output reg [3:0]  ALUControlD,
    output reg [1:0]  SrcASelD, SrcBSelD, CarrySelD, HoldSelD,
    output reg        ShByRegD, ShDisD, CinSelD, ZloUseD, CregWD, MulSignedD,
    output     [2:0]  FlagWriteD,
    output            BranchD, LinkD, BXD, SwiD, BselfD,
    output            IllegalD,
    output            LastD,
    output            SeqMore,
    output     [31:0] ExtImmD
);
    localparam [3:0] ADD = 4'b0100, SUB = 4'b0010, MOV = 4'b1101, ADC = 4'b0101;

    // ---------------- legality (unchanged decoder; only 'illegal' is used) ----------------
    wire illegal;
    decoder_arm #(.FEATURES(FEATURES)) dec (.instr(InstrD), .illegal(illegal),
        .RA1sel(), .RA3sel(), .WA1sel(), .BSrc(), .ImmSrc(), .ShAmtSel(), .ShDis(), .ALUControl(), .MemAddrSel(), .MemR(),
        .MemW(), .MemSize(), .MemSigned(), .ResSrc(), .Res2Src(), .RegW1(), .RegW2(), .FlagW(), .CSrc(), .PCS(), .PCTSel(),
        .Mul(), .MulLong(), .MulSigned(), .MulAcc(), .Halt(), .Seq(), .BX());

    // ---------------- instruction class (same field tests as decoder_arm.v) ----------------
    wire [31:0] ins = InstrD;
    wire [2:0] op = ins[27:25];
    wire P = ins[24], U = ins[23], Bb = ins[22], W = ins[21], L = ins[20];
    wire [3:0] rn = ins[19:16], rd = ins[15:12], rs = ins[11:8], rm = ins[3:0];
    wire is_mulspace = (op == 3'b000) & (ins[7:4] == 4'b1001);
    wire is_mul  = is_mulspace & (ins[27:22] == 6'b000000);
    wire is_mull = is_mulspace & (ins[27:23] == 5'b00001);
    wire is_half = (op == 3'b000) & ins[7] & ins[4] & ~is_mulspace;
    wire is_bx   = (op == 3'b000) & (ins[27:4] == 24'h12FFF1);
    wire is_dpi  = (op == 3'b001);
    wire is_dpr  = (op == 3'b000) & ~is_mulspace & ~is_half & ~is_bx;
    wire is_mem  = (op == 3'b010) | (op == 3'b011);
    wire is_ldm  = (op == 3'b100);
    wire is_br   = (op == 3'b101);
    wire is_swi  = (op == 3'b111);
    wire regshift = is_dpr & ins[4];
    wire [3:0] opc = ins[24:21];
    wire compare = (opc[3:2] == 2'b10);                    // TST TEQ CMP CMN
    wire nouse_rn = (opc == 4'd13) | (opc == 4'd15);       // MOV MVN
    wire arith   = (opc == 4'd2) | (opc == 4'd3) | (opc == 4'd4) | (opc == 4'd5) | (opc == 4'd6) | (opc == 4'd7) |
                   (opc == 4'd10) | (opc == 4'd11);
    wire wb      = ~P | W;                                 // load/store write-back
    wire bself   = (ins[23:0] == 24'hFFFFFE);              // "B ."

    // ---------------- sequencer state ----------------
    reg  u1 = 1'b0;                                        // second uop of a 2-uop instruction
    wire [3:0] lsm_pos;
    wire       lsm_lastbeat, lsm_ddone;
    wire [4:0] lsm_k;
    wire       done = adv & LastD;
    ldmstm_seq_arm_p #(.EN(FEATURES[10])) seq (.clk(clk), .clear(rst | kill | done), .adv(adv & is_ldm & ~illegal),
        .list(ins[15:0]), .U(U), .pos(lsm_pos), .lastbeat(lsm_lastbeat), .k(lsm_k), .ddone(lsm_ddone));
    always @(posedge clk) begin
        if (rst | kill | done) u1 <= 1'b0;
        else if (adv)          u1 <= 1'b1;
    end

    // ---------------- extend unit ----------------
    wire [31:0] ext;
    reg  [1:0]  ImmSrc;
    ext_arm #(.ROT_EN(FEATURES[1])) ex (.instr24(ins[23:0]), .ImmSrc(ImmSrc), .ext(ext));
    reg         uimm_sel;
    reg  [31:0] uimm;
    assign ExtImmD = uimm_sel ? uimm : ext;

    // ---------------- per-uop decode ----------------
    reg regw, pcs, memw, last, u_1, u_2, u_3, br, link, bx, swi, bs;
    reg [2:0] flagw;
    always @(*) begin
        // defaults
        RA1D = rn; RA2D = rm; RA3D = rs; WA3D = rd;
        u_1 = 1'b0; u_2 = 1'b0; u_3 = 1'b0;
        regw = 1'b0; pcs = 1'b0; memw = 1'b0; last = 1'b1; flagw = 3'b000;
        MemtoRegD = 1'b0; MemSignD = 1'b0; MemAdrSelD = 1'b1; MemSizeD = 2'd0;
        ALUControlD = ADD; SrcASelD = 2'd0; SrcBSelD = 2'd1; CarrySelD = 2'd0; HoldSelD = 2'd0;
        ShByRegD = 1'b0; ShDisD = 1'b0; CinSelD = 1'b0; ZloUseD = 1'b0; CregWD = 1'b0; MulSignedD = 1'b0;
        br = 1'b0; link = 1'b0; bx = 1'b0; swi = 1'b0; bs = 1'b0;
        ImmSrc = is_dpi ? 2'd0 : is_br ? 2'd2 : is_half ? 2'd3 : 2'd1;
        uimm_sel = 1'b0; uimm = 32'd0;

        if (illegal) begin
            // FAULT uop: no reads, no writes (StopE in Execute)
        end else if (is_dpi | is_dpr) begin
            RA1D = rn; RA2D = rm; RA3D = rs; WA3D = rd;
            u_1 = ~nouse_rn; u_2 = is_dpr; u_3 = regshift;
            SrcBSelD = is_dpi ? 2'd1 : 2'd0; ShByRegD = regshift;
            ALUControlD = opc;
            flagw = L ? {2'b11, arith} : 3'b000;
            CarrySelD = arith ? 2'd0 : is_dpr ? 2'd1 : (ins[11:8] != 4'd0 && FEATURES[1]) ? 2'd2 : 2'd3;
            regw = ~compare & (rd != 4'd15); pcs = ~compare & (rd == 4'd15);
        end else if (is_mem | is_half) begin
            if (~u1) begin                                                     // u0: the memory access
                RA1D = rn; RA2D = rm; RA3D = rd; WA3D = rd;
                u_1 = 1'b1;
                u_2 = is_mem ? ins[25] : ~Bb;                                  // register offset
                u_3 = ~L;                                                      // store data
                SrcBSelD = is_mem ? (ins[25] ? 2'd0 : 2'd1) : (Bb ? 2'd1 : 2'd0);
                ShDisD = is_half;                                              // halfword register offset: no shift
                ALUControlD = U ? ADD : SUB;
                MemAdrSelD = P;
                MemSizeD = is_mem ? (Bb ? 2'd2 : 2'd0) : ((ins[6:5] == 2'b10) ? 2'd2 : 2'd1);
                MemSignD = is_half & ins[6];
                if (L) begin MemtoRegD = 1'b1; regw = (rd != 4'd15); pcs = (rd == 4'd15); end
                else memw = 1'b1;
                if (wb) begin HoldSelD = 2'd1; last = 1'b0; end
            end else begin                                                     // u1: write-back Rn <- HoldE
                WA3D = rn; SrcASelD = 2'd1; SrcBSelD = 2'd1; uimm_sel = 1'b1; uimm = 32'd0; ALUControlD = ADD;
                regw = 1'b1;
            end
        end else if (is_mul) begin                                             // MUL / MLA: Rd = [19:16], Ra = [15:12]
            RA1D = rd; RA2D = rm; RA3D = rs; WA3D = rn;
            u_1 = W; u_2 = 1'b1; u_3 = 1'b1;
            SrcASelD = W ? 2'd0 : 2'd2; SrcBSelD = 2'd2; ALUControlD = ADD;
            flagw = L ? 3'b100 : 3'b000;
            regw = 1'b1;
        end else if (is_mull) begin                                            // RdHi = [19:16], RdLo = [15:12]
            MulSignedD = Bb;
            if (~u1) begin                                                     // u0: lo
                RA1D = rd; RA2D = rm; RA3D = rs; WA3D = rd;
                u_1 = W; u_2 = 1'b1; u_3 = 1'b1;
                SrcASelD = W ? 2'd0 : 2'd2; SrcBSelD = 2'd2; ALUControlD = ADD;
                HoldSelD = 2'd3; CregWD = 1'b1;
                regw = 1'b1; last = 1'b0;
            end else begin                                                     // u1: hi
                RA2D = rn; WA3D = rn;
                u_2 = W;
                SrcASelD = 2'd1; SrcBSelD = W ? 2'd0 : 2'd1; ShDisD = 1'b1; uimm_sel = 1'b1; uimm = 32'd0;
                ALUControlD = ADC; CinSelD = 1'b1; ZloUseD = 1'b1;
                flagw = L ? 3'b100 : 3'b000;
                regw = 1'b1;
            end
        end else if (is_ldm) begin
            if (~lsm_ddone) begin                                              // data beat k
                RA1D = rn; RA3D = lsm_pos; WA3D = lsm_pos;
                u_1 = (lsm_k == 5'd0); u_3 = ~L;
                SrcASelD = (lsm_k == 5'd0) ? 2'd0 : 2'd1;
                HoldSelD = (lsm_k == 5'd0) ? 2'd2 : 2'd0;                       // beat 0: HoldE <- Rn
                SrcBSelD = 2'd1; uimm_sel = 1'b1;
                uimm = P ? {25'd0, lsm_k + 5'd1, 2'b00} : {25'd0, lsm_k, 2'b00};
                ALUControlD = U ? ADD : SUB; MemAdrSelD = 1'b1;
                if (L) begin MemtoRegD = 1'b1; regw = (lsm_pos != 4'd15); pcs = (lsm_pos == 4'd15); end
                else memw = 1'b1;
                last = lsm_lastbeat & ~W;
            end else begin                                                     // write-back Rn <- HoldE +/- 4N
                WA3D = rn; SrcASelD = 2'd1; SrcBSelD = 2'd1; uimm_sel = 1'b1; uimm = {25'd0, lsm_k, 2'b00};
                ALUControlD = U ? ADD : SUB;
                regw = 1'b1;
            end
        end else if (is_br) begin
            RA1D = 4'd15; SrcASelD = 2'd0; SrcBSelD = 2'd1; ALUControlD = ADD;
            br = 1'b1; bs = ~P & bself;
            if (P) begin link = 1'b1; regw = 1'b1; WA3D = 4'd14; end
        end else if (is_bx) begin
            RA2D = rm; u_2 = 1'b1; SrcBSelD = 2'd0; ShDisD = 1'b1; ALUControlD = MOV;
            br = 1'b1; bx = 1'b1;
        end else if (is_swi) begin
            swi = 1'b1;
        end
    end

    // ---------------- outputs (a bubble in D decodes to nothing) ----------------
    wire ok = validD & ~illegal;
    assign use1D = validD & u_1 & (RA1D != 4'd15);
    assign use2D = validD & u_2 & (RA2D != 4'd15);
    assign use3D = validD & u_3 & (RA3D != 4'd15);
    assign RegWriteD  = ok & regw;
    assign PCSD       = ok & pcs;
    assign MemWriteD  = ok & memw;
    assign FlagWriteD = flagw & {3{ok}};
    assign BranchD    = ok & br;
    assign LinkD      = ok & link;
    assign BXD        = ok & bx;
    assign SwiD       = ok & swi;
    assign BselfD     = ok & bs;
    assign IllegalD   = validD & illegal;
    assign LastD      = illegal | last;
    assign SeqMore    = validD & ~LastD;
endmodule
