//====================================================================
// regfile_arm_p.v   (ARMv4 5-stage pipelined CPU, step 3)  -  register file with SYNCHRONOUS read (three M9K copies)
//
//   Harris & Harris pipelined datapath (section 7.5.1): the register file is read in Decode and the values are
//   registered into the D/E pipeline register (RD1E, RD2E); it is written in Writeback (A3 = WA3W, WD3 = ResultW).
//   Here each read port is an M9K copy whose ADDRESS register is loaded at the D->E edge, so the M9K outputs ARE
//   RD1E / RD2E / RD3E (arm_p_spec.md section 4). The three copies share the one write port, so they always hold the
//   same r0..r14.
//       port 1 : Rn                port 2 : Rm (also RdHi of UMLAL/SMLAL, BX target)
//       port 3 : Rs (shift amount, multiplier b) or Rd (store / STM data)
//
//   WRITE-FIRST BYPASS: [HH] writes the register file in the first half of the cycle and reads it in the second half,
//   so an instruction in Decode sees the value written by Writeback in the same cycle. A synchronous M9K returns the OLD
//   word when the read address is latched on the same edge as the write. So at that edge the hit is remembered
//   (hit_i <= we & wa == ra_i) together with the written value, and the output takes the bypass value: E sees exactly
//   what [HH]'s register file would have delivered.
//
//   R15 = PC + 8: [HH] feeds PCPlus8D (= PCPlus4F) into the register file as R15. A read of address 15 on any port
//   returns 'pc8' latched at the same edge. A write to address 15 never reaches this module (it is a PC write).
//
//   RESET: rst = 1 clears r0..r14 through the write port, one register per cycle (counter 'clr'). rst must be held at
//   least 16 cycles (the board holds it for milliseconds, the testbenches for 20 cycles).
//====================================================================
module regfile_arm_p (
    input         clk,
    input         rst,
    // read addresses (Decode), data valid in the next cycle (Execute)
    input  [3:0]  ra1,
    input  [3:0]  ra2,
    input  [3:0]  ra3,
    input  [31:0] pc8,          // PCPlus8D = PCPlus4F
    output [31:0] rd1,          // RD1E
    output [31:0] rd2,          // RD2E
    output [31:0] rd3,          // RD3E
    // write port (Writeback)
    input         we,           // RegWriteW (never for address 15)
    input  [3:0]  wa,           // WA3W
    input  [31:0] wd            // ResultW
);
    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] rf1 [0:15];
    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] rf2 [0:15];
    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] rf3 [0:15];
    integer i;
    initial for (i = 0; i < 16; i = i + 1) begin rf1[i] = 32'd0; rf2[i] = 32'd0; rf3[i] = 32'd0; end

    reg  [3:0]  clr = 4'd0;
    wire        wen = rst | (we & (wa != 4'd15));
    wire [3:0]  wad = rst ? clr : wa;
    wire [31:0] wdd = rst ? 32'd0 : wd;

    reg [31:0] q1, q2, q3;                               // M9K read registers: no power-up value (as in the MC)
    always @(posedge clk) begin
        if (wen) rf1[wad] <= wdd;
        q1 <= rf1[ra1];
    end
    always @(posedge clk) begin
        if (wen) rf2[wad] <= wdd;
        q2 <= rf2[ra2];
    end
    always @(posedge clk) begin
        if (wen) rf3[wad] <= wdd;
        q3 <= rf3[ra3];
    end

    // ---------------- R15 and write-first bypass (registered with the read address) ----------------
    reg [31:0] r15q = 32'd0, wq = 32'd0;
    reg        is1 = 1'b0, is2 = 1'b0, is3 = 1'b0, h1 = 1'b0, h2 = 1'b0, h3 = 1'b0;
    always @(posedge clk) begin
        clr  <= rst ? clr + 4'd1 : 4'd0;
        r15q <= pc8;
        wq   <= wdd;
        is1  <= (ra1 == 4'd15);
        is2  <= (ra2 == 4'd15);
        is3  <= (ra3 == 4'd15);
        h1   <= wen & (wad == ra1);
        h2   <= wen & (wad == ra2);
        h3   <= wen & (wad == ra3);
    end
    assign rd1 = is1 ? r15q : h1 ? wq : q1;
    assign rd2 = is2 ? r15q : h2 ? wq : q2;
    assign rd3 = is3 ? r15q : h3 ? wq : q3;
endmodule
