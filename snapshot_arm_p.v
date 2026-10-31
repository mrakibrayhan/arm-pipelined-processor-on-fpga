//====================================================================
// snapshot_arm_p.v   (ARMv4 5-stage pipelined CPU, board view, step 13)  -  register / RAM snapshot for the In-System
//                                                                       Memory Content Editor
//   (same unit as snapshot_arm_mc.v of the multi-cycle board; only R16 word 6 changes meaning: pipe status)
//
//   Same idea, same four RAMs and same declaration order (R00, R16, D00, D16) as snapshot_arm.v of the single-cycle board:
//        R00 = r0 .. r15   (r15 = PC register: after a halt by "B ." its address, after SWI the SWI address + 4)
//        R16 = status words: 0 CPSR {N,Z,C,V, 20'b0, 8'h10}   1 PC   2 instruction word   3 cycle counter
//              4 instruction counter   5 {29'b0, fault, halted, started}   6 pipe status   7..15 = 0
//        D00 = RAM word  0 .. 15      (0x10000000 .. 0x1000003C)
//        D16 = RAM word 16 .. 31      (0x10000040 .. 0x1000007C)
//   WHEN: the program halts or faults (trigger_auto) or the ISSP source bit is set (trigger_src, works at any time).
//
//   Differences from the single-cycle unit (synchronous-read CPU):
//     1. The CPU stops only when its pipeline is EMPTY: 'freeze' stops fetching, the instructions already in the pipeline
//        finish, then the core raises 'view_rdy' (also after a halt / fault); the unit waits for it (state WAIT).
//     2. The register file and the RAM are synchronous: the value for index i comes one cycle after i is presented, so the
//        sweep presents idx 0..63 and writes word (idx - 1) one cycle later (65 cycles). 'widx' is that write index (the top
//        uses it to select the R16 status word).
//   IDLE (CPU runs) -> WAIT (freeze, until view_rdy) -> SNAP (65 cycles) -> DONE (frozen while the trigger stays true) -> IDLE.
//   RESET CLEAR: while rst = 1 (KEY3 held, or the CPU not started yet) the four RAMs are written with 0, word by word
//   (all four in parallel, 16 cycles; the button filter keeps rst high for milliseconds). So after KEY3 the Memory Content
//   Editor shows zeros, and after KEY2 the fresh snapshot of the new run.
//====================================================================
module snapshot_arm_p (
    input         clk,           // cpu_clk
    input         rst,           // debounced reset (level)
    input         started,
    input         trigger_auto,  // halted | fault
    input         trigger_src,   // ISSP source bit (JTAG clock domain)
    input         view_rdy,      // the core is stopped and its view ports are free
    input  [31:0] view_val,      // core dbg_val (belongs to the idx of the previous cycle)
    input  [31:0] extra_val,     // R16 status word number widx[3:0] (from the top)
    output        freeze,
    output        done,
    output [5:0]  idx,           // view index presented to the core
    output [5:0]  widx           // index of the word written in this cycle
);
    reg s1 = 1'b0, s2 = 1'b0;
    always @(posedge clk) begin
        s1 <= trigger_src;
        s2 <= s1;
    end
    wire trigger = trigger_auto | s2;

    localparam IDLE = 2'd0, WAIT = 2'd1, SNAP = 2'd2, DONE = 2'd3;
    reg [1:0] state = IDLE;
    reg [6:0] cnt   = 7'd0;
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            cnt   <= 7'd0;
        end else begin
            case (state)
                IDLE: if (started && trigger) state <= WAIT;
                WAIT: if (view_rdy) begin state <= SNAP; cnt <= 7'd0; end
                SNAP: if (cnt == 7'd64) state <= DONE; else cnt <= cnt + 7'd1;
                DONE: if (!trigger) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end
    assign freeze = (state != IDLE);
    assign done   = (state == DONE);
    assign idx    = cnt[5:0];
    wire [6:0] wcnt = cnt - 7'd1;
    assign widx   = wcnt[5:0];
    wire   wr     = (state == SNAP) && (cnt != 7'd0);

    // reset clear: a 4-bit word counter that runs while rst = 1
    reg  [3:0]  clr = 4'd0;
    always @(posedge clk) clr <= rst ? clr + 4'd1 : 4'd0;

    wire [3:0]  word_in   = rst ? clr : widx[3:0];
    wire [31:0] snap_data = rst ? 32'd0 : (widx[5:4] == 2'd3) ? extra_val : view_val;
    wire [31:0] q_R00_unused, q_D00_unused, q_D16_unused, q_R16_unused;

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=R00"), .lpm_type ("altsyncram")
    ) snap_ram_R00 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (wr && widx[5:4] == 2'd0)), .q_a (q_R00_unused));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=R16"), .lpm_type ("altsyncram")
    ) snap_ram_R16 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (wr && widx[5:4] == 2'd3)), .q_a (q_R16_unused));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=D00"), .lpm_type ("altsyncram")
    ) snap_ram_D00 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (wr && widx[5:4] == 2'd1)), .q_a (q_D00_unused));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=D16"), .lpm_type ("altsyncram")
    ) snap_ram_D16 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (wr && widx[5:4] == 2'd2)), .q_a (q_D16_unused));
endmodule
