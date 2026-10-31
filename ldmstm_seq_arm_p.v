//====================================================================
// ldmstm_seq_arm_p.v   (ARMv4 5-stage pipelined CPU, step 5/11)  -  LDM / STM register-list sequencer in Decode
//
//   Same list scan as the board-proven single-cycle / multi-cycle sequencers (no copy of the list):
//       cand = (k == 0) ? list : list bits beyond lastpos (above it for U = 1, below it for U = 0)
//       one  = U ? lowest set bit of cand : highest          pos = index of 'one'
//       lastbeat = no other bit left in cand
//   One data beat (uop) per cycle leaves Decode ([FUR] Ch. 4, multi-cycle instructions). 'k' counts the beats already
//   issued: the offset of the current beat is 4k (P = 0) or 4(k+1) (P = 1), and after the last beat k = N, so the
//   write-back uop uses 4N. 'ddone' = all data beats issued (the write-back uop is next).
//     adv  : the current uop leaves Decode (validD & ~ldrStall)
//     clear: the instruction finished (adv & last uop), it was killed (branch / stop in E), or reset
//   EN = 0 : not built (FEATURES[10] = 0): constant outputs, the decoder rejects LDM/STM.
//====================================================================
module ldmstm_seq_arm_p #(
    parameter EN = 1
) (
    input         clk,
    input         clear,
    input         adv,           // a data beat of this LDM/STM leaves Decode
    input  [15:0] list,          // Instr[15:0]
    input         U,             // Instr[23]
    output [3:0]  pos,           // register of the current beat
    output        lastbeat,      // the current beat is the last one
    output [4:0]  k,             // number of beats already issued
    output        ddone          // all beats issued: write-back uop
);
    generate
        if (!EN) begin : g_none
            assign pos = 4'd0; assign lastbeat = 1'b1; assign k = 5'd0; assign ddone = 1'b0;
        end else begin : g_seq
            reg [3:0] lastpos = 4'd0;
            reg [4:0] kq = 5'd0;
            reg       dq = 1'b0;

            wire [15:0] beyond;
            genvar g;
            for (g = 0; g < 16; g = g + 1) begin : g_bey
                assign beyond[g] = U ? (lastpos < g) : (lastpos > g);
            end
            wire [15:0] cand = (kq == 5'd0) ? list : (list & beyond);

            reg [15:0] lo_bit, hi_bit;
            reg        seen_lo, seen_hi;
            integer    i;
            always @(*) begin
                seen_lo = 1'b0;
                for (i = 0; i < 16; i = i + 1) begin lo_bit[i] = cand[i] & ~seen_lo; seen_lo = seen_lo | cand[i]; end
                seen_hi = 1'b0;
                for (i = 15; i >= 0; i = i - 1) begin hi_bit[i] = cand[i] & ~seen_hi; seen_hi = seen_hi | cand[i]; end
            end
            wire [15:0] one = U ? lo_bit : hi_bit;
            reg  [3:0]  p;
            integer     m;
            always @(*) begin
                p = 4'd0;
                for (m = 0; m < 16; m = m + 1) if (one[m]) p = m[3:0];
            end
            assign lastbeat = ((cand & ~one) == 16'd0);
            assign pos = p;

            always @(posedge clk) begin
                if (clear) begin
                    kq <= 5'd0; dq <= 1'b0; lastpos <= 4'd0;
                end else if (adv & ~dq) begin
                    lastpos <= p;
                    kq      <= kq + 5'd1;
                    dq      <= lastbeat;
                end
            end
            assign k = kq; assign ddone = dq;
        end
    endgenerate
endmodule
