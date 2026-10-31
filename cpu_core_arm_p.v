//====================================================================
// cpu_core_arm_p.v   (ARMv4 5-stage pipelined CPU, step 5)  -  processor core
//
//   controller_arm_p (Decode control + uop sequencer) + datapath_arm_p (pipeline registers, E-stage units, W) +
//   hazard_arm_p + mem_arm_p (synchronous-read M9K: dual-port ROM + byte-enable RAM). Harris & Harris section 7.5.
//
//   F/D register: the ROM port A output is InstrD; 'validD' is its valid bit (0 = bubble):
//       rst | KillD -> 0     StallD -> hold     FlushD -> 0     else 1        (KillD > StallD > FlushD, spec section 7)
//
//   BUILT : classes whose hardware is enabled in this build step (FEATURES bits, see decoder_arm.v). FEATURES is ANDed
//           with BUILT, so a class that is not built yet decodes as illegal (same mechanism as the SC / MC cores).
//           Step 6: 0 (the H&H base set)   step 7: bits 0,1,2   step 8: 3,4   step 9: 5,6,7   step 10: 8,9   step 11: 10.
//   Run control: the stop instruction (SWI or "B ." with the condition true, an illegal encoding, BX to an odd address)
//   kills everything younger in Execute and sets 'halted' / 'fault' when it reaches Writeback. rst returns to PC 0 and
//   clears r0..r14 (rst must be held >= 16 cycles, regfile_arm_p.v).
//
//   Board view (DBG_EN = 1): 'freeze' stops fetching; when the pipeline is empty (frozen, halted or faulted) 'view_rdy'
//   = 1 and the view borrows register-file port 1 and the data port: dbg_idx 0..14 = r0..r14, 15 = PC, 16.. = RAM word
//   (idx - 16). SYNCHRONOUS read: dbg_val belongs to the dbg_idx of the PREVIOUS cycle (the snapshot unit allows for it).
//   PC shown = StopPC after a halt / fault (MC convention: SWI / fault -> address + 4, "B ." -> its address),
//   otherwise PCF (= the next instruction to execute when the pipeline is empty).
//====================================================================
module cpu_core_arm_p #(
    parameter [10:0] FEATURES  = 11'h7FF,
    parameter        IMEM_AW   = 7,
    parameter        DMEM_AW   = 5,
    parameter        INIT_FILE = "program.hex",
    parameter        DATA_FILE = "data.hex",
    parameter [10:0] BUILT     = 11'h7FF,
    parameter        DBG_EN    = 0
) (
    input         clk,
    input         rst,
    output        halted,
    output        fault,
    output        retire,           // 1 when the last uop of an instruction is in Writeback (condition-failed ones included)
    output [31:0] pc,               // PC shown on the board (see above)
    output [31:0] instr,            // instruction in Decode (InstrD)
    output [3:0]  flags,
    output [4:0]  pstat,            // pipe status {stopping, validD, validE, validM, validW}
    // board view
    input         freeze,
    output        view_rdy,
    input  [5:0]  dbg_idx,
    output [31:0] dbg_val,
    // trace (testbench)
    output        tr_regw,  output [3:0] tr_wa,  output [31:0] tr_wd,
    output        tr_memw,  output [31:0] tr_adr, output [31:0] tr_mdata, output [1:0] tr_msize
);
    localparam [10:0] FEAT_RAW = FEATURES & BUILT;
    localparam [10:0] FEAT     = {FEAT_RAW[10:3], FEAT_RAW[2] & FEAT_RAW[1], FEAT_RAW[1:0]};   // SHREG needs SHIFT

    // ---------------- wires ----------------
    wire [31:0] PCF, PCPlus4F, InstrD, ExtImmD, MemAdrE, WriteDataE, ReadDataM, ResultW, StopPC, rd1_raw, d_q;
    wire [3:0]  RA1D, RA2D, RA3D, WA3D, RA1E, RA2E, RA3E, WA3E, WA3M, WA3W, ALUControlD;
    wire        use1D, use2D, use3D, RegWriteD, PCSD, MemWriteD, MemtoRegD, MemSignD, MemAdrSelD;
    wire [1:0]  MemSizeD, SrcASelD, SrcBSelD, CarrySelD, HoldSelD, MemSizeE, Fwd1E, Fwd2E, Fwd3E;
    wire        ShByRegD, ShDisD, CinSelD, ZloUseD, CregWD, MulSignedD, BranchD, LinkD, BXD, SwiD, BselfD, IllegalD, LastD;
    wire [2:0]  FlagWriteD;
    wire        SeqMore, validE, validM, validW, MemtoRegE, PCSE, BranchTakenE, StopE, RegWriteM, RegWriteW, PCSrcM, PCSrcW;
    wire        MemWriteE, MemSignE, ldrStall, StallF, StallD, FlushD, KillD, FlushE, pc_en;

    // ---------------- F/D valid bit, stop state ----------------
    reg validD = 1'b0, stopping = 1'b0;
    always @(posedge clk) begin
        if (rst | KillD)  validD <= 1'b0;
        else if (StallD)  validD <= validD;
        else              validD <= ~FlushD;
        if (rst)          stopping <= 1'b0;
        else if (StopE)   stopping <= 1'b1;
    end

    wire frz = DBG_EN ? freeze : 1'b0;

    // ---------------- control ----------------
    controller_arm_p #(.FEATURES(FEAT)) ctl (
        .clk(clk), .rst(rst), .InstrD(InstrD), .validD(validD), .adv(validD & ~ldrStall), .kill(KillD),
        .RA1D(RA1D), .RA2D(RA2D), .RA3D(RA3D), .WA3D(WA3D), .use1D(use1D), .use2D(use2D), .use3D(use3D),
        .RegWriteD(RegWriteD), .PCSD(PCSD), .MemWriteD(MemWriteD), .MemtoRegD(MemtoRegD), .MemSignD(MemSignD),
        .MemAdrSelD(MemAdrSelD), .MemSizeD(MemSizeD), .ALUControlD(ALUControlD), .SrcASelD(SrcASelD), .SrcBSelD(SrcBSelD),
        .CarrySelD(CarrySelD), .HoldSelD(HoldSelD), .ShByRegD(ShByRegD), .ShDisD(ShDisD), .CinSelD(CinSelD),
        .ZloUseD(ZloUseD), .CregWD(CregWD), .MulSignedD(MulSignedD), .FlagWriteD(FlagWriteD), .BranchD(BranchD),
        .LinkD(LinkD), .BXD(BXD), .SwiD(SwiD), .BselfD(BselfD), .IllegalD(IllegalD), .LastD(LastD), .SeqMore(SeqMore),
        .ExtImmD(ExtImmD));

    hazard_arm_p hz (
        .validD(validD), .RA1D(RA1D), .RA2D(RA2D), .RA3D(RA3D), .use1D(use1D), .use2D(use2D), .use3D(use3D),
        .PCSD(PCSD), .SeqMore(SeqMore), .validE(validE), .RA1E(RA1E), .RA2E(RA2E), .RA3E(RA3E), .WA3E(WA3E),
        .MemtoRegE(MemtoRegE), .PCSE(PCSE), .BranchTakenE(BranchTakenE), .StopE(StopE), .WA3M(WA3M), .WA3W(WA3W),
        .RegWriteM(RegWriteM), .RegWriteW(RegWriteW), .PCSrcM(PCSrcM), .PCSrcW(PCSrcW), .freeze(frz), .stopping(stopping),
        .Fwd1E(Fwd1E), .Fwd2E(Fwd2E), .Fwd3E(Fwd3E), .ldrStall(ldrStall), .StallF(StallF), .StallD(StallD),
        .FlushD(FlushD), .KillD(KillD), .FlushE(FlushE), .pc_en(pc_en));

    // ---------------- board view ----------------
    wire empty = ~validD & ~validE & ~validM & ~validW;
    assign view_rdy = DBG_EN & empty & (frz | halted | fault);
    wire       view = view_rdy;
    wire [5:0] dbg_ridx = dbg_idx - 6'd16;
    wire [31:0] dbg_adr = {4'h1, 20'd0, dbg_ridx, 2'b00};
    reg  [5:0] idx_q = 6'd0;
    always @(posedge clk) idx_q <= dbg_idx;
    generate
        if (DBG_EN) begin : g_dbg
            assign dbg_val = (idx_q < 6'd15) ? rd1_raw : (idx_q == 6'd15) ? pc :
                             ({1'b0, idx_q - 6'd16} < (7'd1 << DMEM_AW)) ? d_q : 32'd0;
        end else begin : g_nodbg
            assign dbg_val = 32'd0;
        end
    endgenerate

    // ---------------- datapath ----------------
    datapath_arm_p #(.SH_EN(FEAT[1]), .SHREG_EN(FEAT[2]), .MUL_EN(FEAT[8] | FEAT[9]), .MULL_EN(FEAT[9])) dp (
        .clk(clk), .rst(rst), .pc_en(pc_en), .PCF(PCF), .PCPlus4F(PCPlus4F),
        .validD(validD), .InstrD(InstrD), .RA1D(RA1D), .RA2D(RA2D), .RA3D(RA3D), .WA3D(WA3D),
        .RegWriteD(RegWriteD), .PCSD(PCSD), .MemtoRegD(MemtoRegD), .MemWriteD(MemWriteD), .MemSignD(MemSignD),
        .MemAdrSelD(MemAdrSelD), .MemSizeD(MemSizeD), .ALUControlD(ALUControlD), .SrcASelD(SrcASelD), .SrcBSelD(SrcBSelD),
        .CarrySelD(CarrySelD), .HoldSelD(HoldSelD), .ShByRegD(ShByRegD), .ShDisD(ShDisD), .CinSelD(CinSelD),
        .ZloUseD(ZloUseD), .CregWD(CregWD), .MulSignedD(MulSignedD), .FlagWriteD(FlagWriteD), .BranchD(BranchD),
        .LinkD(LinkD), .BXD(BXD), .SwiD(SwiD), .BselfD(BselfD), .IllegalD(IllegalD), .LastD(LastD), .ExtImmD(ExtImmD),
        .FlushE(FlushE), .Fwd1E(Fwd1E), .Fwd2E(Fwd2E), .Fwd3E(Fwd3E), .RA1E(RA1E), .RA2E(RA2E), .RA3E(RA3E), .WA3E(WA3E),
        .validE(validE), .MemtoRegE_o(MemtoRegE), .PCSE_o(PCSE), .BranchTakenE(BranchTakenE), .StopE(StopE),
        .WA3M(WA3M), .WA3W(WA3W), .RegWriteM(RegWriteM), .RegWriteW(RegWriteW), .PCSrcM(PCSrcM), .PCSrcW(PCSrcW),
        .validM(validM), .validW(validW), .MemAdrE(MemAdrE), .MemWriteE_g(MemWriteE), .MemSizeE_o(MemSizeE),
        .MemSignE_o(MemSignE), .WriteDataE(WriteDataE), .ReadDataM(ReadDataM), .flags(flags), .halted(halted),
        .fault(fault), .StopPC(StopPC), .retire(retire), .ResultW(ResultW), .view(view), .dbg_ra(dbg_idx[3:0]),
        .rd1_raw(rd1_raw));

    // ---------------- memories ----------------
    mem_arm_p #(.IMEM_AW(IMEM_AW), .DMEM_AW(DMEM_AW), .INIT_FILE(INIT_FILE), .DATA_FILE(DATA_FILE)) mem (
        .clk(clk), .ia_adr(PCF), .ia_hold(StallD), .instr(InstrD),
        .d_adr(view ? dbg_adr : MemAdrE), .d_we(MemWriteE & ~view), .d_size(view ? 2'd0 : MemSizeE),
        .d_sgn(view ? 1'b0 : MemSignE), .d_wdata(WriteDataE), .d_q(d_q), .d_rdata(ReadDataM));

    assign pc    = (halted | fault) ? StopPC : PCF;
    assign instr = InstrD;
    assign pstat = {stopping, validD, validE, validM, validW};

    // ---------------- trace ----------------
    assign tr_regw = RegWriteW; assign tr_wa = WA3W; assign tr_wd = ResultW;
    assign tr_memw = MemWriteE; assign tr_adr = MemAdrE; assign tr_mdata = WriteDataE; assign tr_msize = MemSizeE;
endmodule
