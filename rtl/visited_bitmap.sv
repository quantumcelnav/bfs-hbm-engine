// Word-addressed visited bitmap backed by inferred block RAM.
//
// Organization: BITMAP_WORDS x 32b words.
//   word address = vertex_id[VERTEX_W-1:5]
//   bit  select  = vertex_id[4:0]
//
// Single-port BRAM with read-modify-write for set operations.
// One operation per 2 cycles (read cycle + modify/write cycle).
//
// Protocol:
//   To check vertex v:  assert op_valid, op_vid=v, op_set=0.
//                       Result appears on chk_visited one cycle later (chk_valid high).
//   To set vertex v:    assert op_valid, op_vid=v, op_set=1.
//                       set_done pulses one cycle after the write commits.
//   busy is high while an RMW is in progress — caller must not issue a new
//   op until busy is de-asserted.

module visited_bitmap #(
    parameter int unsigned VERTEX_W = 20    // supports 2^VERTEX_W vertices
) (
    input  logic                 clk,
    input  logic                 rst_n,

    // Unified request port
    input  logic                 op_valid,     // new operation
    input  logic [VERTEX_W-1:0]  op_vid,       // target vertex
    input  logic                 op_set,       // 0=check, 1=set (RMW)

    // Results
    output logic                 chk_valid,    // check result ready
    output logic                 chk_visited,  // 1 if vertex was already visited
    output logic                 set_done,     // set operation committed
    output logic                 busy          // cannot accept new op
);

    localparam int unsigned BITMAP_WORDS = (2**VERTEX_W + 31) / 32;  // ≥1 word
    // WORD_ADDR_W must be ≥1; when VERTEX_W<=5 all vertices fit in one word
    localparam int unsigned WORD_ADDR_W  = (VERTEX_W > 5) ? (VERTEX_W - 5) : 1;

    // Inferred block RAM (sync read, read-first)
    logic [31:0] bram [0:BITMAP_WORDS-1];

    // Pipeline registers
    logic                    p1_valid, p1_set;
    logic [VERTEX_W-1:0]     p1_vid;
    logic [31:0]             p1_rdata;
    logic [WORD_ADDR_W-1:0]  p1_waddr;
    logic [4:0]              p1_bsel;

    logic [WORD_ADDR_W-1:0]  op_waddr;
    logic [4:0]              op_bsel;

    // Word address: upper bits above bit 5; zero when VERTEX_W<=5 (single word)
    generate
        if (VERTEX_W > 5)
            assign op_waddr = op_vid[VERTEX_W-1:5];
        else
            assign op_waddr = '0;
    endgenerate
    assign op_bsel  = op_vid[4:0];
    assign busy     = p1_valid;   // one operation in-flight

    // Stage 0 → 1: register address, issue BRAM read
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p1_valid <= 1'b0;
            p1_vid   <= '0;
            p1_set   <= 1'b0;
            p1_waddr <= '0;
            p1_bsel  <= '0;
        end else begin
            p1_valid <= op_valid & ~busy;
            p1_vid   <= op_vid;
            p1_set   <= op_set;
            p1_waddr <= op_waddr;
            p1_bsel  <= op_bsel;
        end
    end

    // BRAM read (synchronous, 1-cycle latency)
    always_ff @(posedge clk) begin
        if (op_valid && !busy)
            p1_rdata <= bram[op_waddr];
    end

    // Stage 1: check or RMW write, generate outputs
    logic [31:0] modified_word;
    assign modified_word = p1_rdata | (32'b1 << p1_bsel);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            chk_valid   <= 1'b0;
            chk_visited <= 1'b0;
            set_done    <= 1'b0;
        end else begin
            chk_valid   <= p1_valid & ~p1_set;
            chk_visited <= p1_valid & ~p1_set & p1_rdata[p1_bsel];
            set_done    <= p1_valid &  p1_set;
            // RMW write
            if (p1_valid && p1_set)
                bram[p1_waddr] <= modified_word;
        end
    end

    // Initialise all words to zero on reset (generates synchronous clear for BRAM)
    // Synthesis: Yosys will keep this as a reset-init attribute; tools targeting
    // devices with BRAM init support use it directly.
    integer i;
    initial begin
        for (i = 0; i < BITMAP_WORDS; i++)
            bram[i] = 32'b0;
    end

endmodule
