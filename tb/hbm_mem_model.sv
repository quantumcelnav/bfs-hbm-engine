// Behavioural HBM memory model.
//
// Implements AXI4 slave (AR + R channels only).
// Fixed read latency of HBM_LATENCY_CYC cycles from ARVALID/ARREADY
// to first RVALID — models tRCD + tRL as seen from the controller.
//
// Memory is word-addressed (32-bit words). Initialised by the testbench
// via the init_word() task before simulation starts.

module hbm_mem_model #(
    parameter int unsigned AXI_DATA_W      = 256,
    parameter int unsigned AXI_ADDR_W      = 33,
    parameter int unsigned AXI_ID_W        = 4,
    parameter int unsigned MEM_DEPTH_WORDS = 65536,    // 256 KB
    parameter int unsigned HBM_LATENCY_CYC = 20        // tRCD+tRL ≈ 20 cycles
) (
    input  logic                    clk,
    input  logic                    rst_n,

    // AXI4 AR channel
    input  logic                    s_axi_arvalid,
    output logic                    s_axi_arready,
    input  logic [AXI_ADDR_W-1:0]   s_axi_araddr,
    input  logic [7:0]              s_axi_arlen,
    input  logic [2:0]              s_axi_arsize,
    input  logic [1:0]              s_axi_arburst,
    input  logic [AXI_ID_W-1:0]     s_axi_arid,

    // AXI4 R channel
    output logic                    s_axi_rvalid,
    input  logic                    s_axi_rready,
    output logic [AXI_DATA_W-1:0]   s_axi_rdata,
    output logic                    s_axi_rlast,
    output logic [1:0]              s_axi_rresp,
    output logic [AXI_ID_W-1:0]     s_axi_rid
);

    localparam int unsigned BYTES_PER_BEAT = AXI_DATA_W / 8;   // 32
    localparam int unsigned WORDS_PER_BEAT = BYTES_PER_BEAT / 4; // 8

    // Memory array
    logic [31:0] mem [0:MEM_DEPTH_WORDS-1];

    // Transaction queue: address + length, queued after AR handshake
    localparam int unsigned Q_DEPTH = 4;
    logic [AXI_ADDR_W-1:0]  txn_addr  [0:Q_DEPTH-1];
    logic [7:0]             txn_len   [0:Q_DEPTH-1];
    logic [AXI_ID_W-1:0]    txn_id    [0:Q_DEPTH-1];
    logic [1:0]             txn_wr, txn_rd;   // queue write/read pointers
    logic                   txn_empty, txn_full;

    assign txn_empty = (txn_wr == txn_rd);
    assign txn_full  = ((txn_wr + 2'd1) == txn_rd);

    // Always ready to accept AR (when queue not full)
    assign s_axi_arready = ~txn_full;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            txn_wr <= '0;
        end else if (s_axi_arvalid && s_axi_arready) begin
            txn_addr[txn_wr] <= s_axi_araddr;
            txn_len [txn_wr] <= s_axi_arlen;
            txn_id  [txn_wr] <= s_axi_arid;
            txn_wr           <= txn_wr + 2'd1;
        end
    end

    // Response generation with fixed latency
    typedef enum logic [1:0] {
        RS_IDLE  = 2'd0,
        RS_DELAY = 2'd1,
        RS_DATA  = 2'd2
    } rstate_t;

    rstate_t           rstate;
    logic [5:0]        delay_cnt;
    logic [7:0]        beats_remaining;
    logic [AXI_ADDR_W-1:0] cur_addr;
    logic [AXI_ID_W-1:0]   cur_id;

    assign s_axi_rresp = 2'b00;   // OKAY
    assign s_axi_rid   = cur_id;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rstate          <= RS_IDLE;
            txn_rd          <= '0;
            s_axi_rvalid    <= 1'b0;
            s_axi_rlast     <= 1'b0;
            s_axi_rdata     <= '0;
            delay_cnt       <= '0;
            beats_remaining <= '0;
            cur_addr        <= '0;
            cur_id          <= '0;
        end else begin
            case (rstate)
                RS_IDLE: begin
                    if (!txn_empty) begin
                        cur_addr        <= txn_addr[txn_rd];
                        beats_remaining <= txn_len[txn_rd];
                        cur_id          <= txn_id [txn_rd];
                        txn_rd          <= txn_rd + 2'd1;
                        delay_cnt       <= HBM_LATENCY_CYC[5:0] - 6'd1;
                        rstate          <= RS_DELAY;
                    end
                end

                RS_DELAY: begin
                    if (delay_cnt == '0)
                        rstate <= RS_DATA;
                    else
                        delay_cnt <= delay_cnt - 6'd1;
                end

                RS_DATA: begin
                    // Build beat data from memory
                    s_axi_rvalid <= 1'b1;
                    for (int w = 0; w < WORDS_PER_BEAT; w++) begin
                        s_axi_rdata[32*w +: 32] <=
                            (((cur_addr >> 2) + w) < MEM_DEPTH_WORDS)
                            ? mem[(cur_addr >> 2) + w]
                            : 32'hDEAD_BEEF;
                    end
                    s_axi_rlast <= (beats_remaining == '0);

                    if (s_axi_rvalid && s_axi_rready) begin
                        if (beats_remaining == '0) begin
                            s_axi_rvalid <= 1'b0;
                            s_axi_rlast  <= 1'b0;
                            rstate       <= RS_IDLE;
                        end else begin
                            cur_addr        <= cur_addr + BYTES_PER_BEAT;
                            beats_remaining <= beats_remaining - 8'd1;
                        end
                    end
                end

                default: rstate <= RS_IDLE;
            endcase
        end
    end

    // Initialisation task — called from testbench
    task init_word(input logic [AXI_ADDR_W-1:0] byte_addr,
                   input logic [31:0]            data);
        mem[byte_addr >> 2] = data;
    endtask

    initial begin
        for (int i = 0; i < MEM_DEPTH_WORDS; i++)
            mem[i] = 32'h0;
    end

endmodule
