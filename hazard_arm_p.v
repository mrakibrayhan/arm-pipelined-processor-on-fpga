//====================================================================
// hazard_arm_p.v   (ARMv4 5-stage pipelined CPU, step 5)  -  hazard unit (arm_p_spec.md section 7)
//
//   Harris & Harris section 7.5.3, extended to three register-file ports and to multi-uop instructions:
//     forwarding  ForwardXE = 10 (ALUResultM) if RAXE == WA3M & RegWriteM, else 01 (ResultW) if RAXE == WA3W & RegWriteW
//     ldrStall    a load in Execute writes a register that the uop in Decode reads (any of the 3 ports)
//     control     BranchTakenE (B/BL/BX resolved in Execute): flush D and E
//                 PCWrPendingF (a uop that writes the PC in Writeback is in D, E or M): stall F, bubbles into D
//     uops        SeqMore: the instruction in D sends one more uop: stall F and D, but E takes the current uop
//     stop        StopE (SWI / B . / illegal / BX to odd in Execute): kill D, flush E, then 'stopping' stalls F
//     board       freeze: stop fetching (bubbles into D) until the snapshot is done
//   StallD wins over FlushD (as in [HH]: the D register's clear needs its enable); KillD wins over StallD.
//   R15 is never forwarded / stalled on: the register-file write port never writes R15 (it is the PC).
//====================================================================
module hazard_arm_p (
    // Decode
    input        validD,
    input  [3:0] RA1D, RA2D, RA3D,
    input        use1D, use2D, use3D,
    input        PCSD,           // uop in D writes the PC in W
    input        SeqMore,
    // Execute
    input        validE,
    input  [3:0] RA1E, RA2E, RA3E, WA3E,
    input        MemtoRegE,
    input        PCSE,           // validE & PCSE (before the condition)
    input        BranchTakenE,
    input        StopE,
    // Memory, Writeback
    input  [3:0] WA3M, WA3W,
    input        RegWriteM, RegWriteW,
    input        PCSrcM, PCSrcW,
    // run control
    input        freeze,
    input        stopping,
    // outputs
    output [1:0] Fwd1E, Fwd2E, Fwd3E,
    output       ldrStall,
    output       StallF, StallD, FlushD, KillD, FlushE,
    output       pc_en
);
    assign Fwd1E = (RegWriteM & (RA1E == WA3M)) ? 2'b10 : (RegWriteW & (RA1E == WA3W)) ? 2'b01 : 2'b00;
    assign Fwd2E = (RegWriteM & (RA2E == WA3M)) ? 2'b10 : (RegWriteW & (RA2E == WA3W)) ? 2'b01 : 2'b00;
    assign Fwd3E = (RegWriteM & (RA3E == WA3M)) ? 2'b10 : (RegWriteW & (RA3E == WA3W)) ? 2'b01 : 2'b00;

    assign ldrStall = validE & MemtoRegE & (WA3E != 4'd15) &
                      ((use1D & (RA1D == WA3E)) | (use2D & (RA2D == WA3E)) | (use3D & (RA3D == WA3E)));
    wire PCWrPendingF = PCSD | PCSE | PCSrcM;

    assign StallF = ldrStall | PCWrPendingF | SeqMore | freeze | stopping;
    assign StallD = ldrStall | SeqMore;
    assign FlushD = PCWrPendingF | PCSrcW | freeze | stopping;
    assign KillD  = BranchTakenE | StopE;
    assign FlushE = ldrStall | BranchTakenE | StopE | ~validD;
    assign pc_en  = ~StallF | BranchTakenE | PCSrcW;
endmodule
