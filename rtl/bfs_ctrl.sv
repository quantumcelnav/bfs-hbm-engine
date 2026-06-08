// BFS traversal controller.
//
// Memory layout (byte-addressed HBM):
//   ROW_PTR_BASE + v*4   : uint32 offset of vertex v's first edge in col_idx
//   COL_IDX_BASE + e*4   : uint32 destination vertex of edge e
//
// One outstanding AXI read at a time. Processes one neighbor vertex per
// 2-cycle visited-bitmap RMW. Beat buffer holds 8 vertex IDs per 256-bit
// HBM beat (AXI_DATA_W=256, 32b vertex IDs → 8 per beat).
//
// Throughput (steady-state): 1 edge processed per 2 cycles when bitmap is
// hit-free. This intentionally models the random-access penalty that produces
// η ≈ 40 % for graph BFS in the companion HBM PHY analysis.

module bfs_ctrl #(
    parameter int unsigned VERTEX_W    = 20,
    parameter int unsigned AXI_DATA_W  = 256,
    parameter int unsigned AXI_ADDR_W  = 33,
    parameter longint unsigned ROW_PTR_BASE = 33'h0_0000_0000,
    parameter longint unsigned COL_IDX_BASE = 33'h0_0010_0000   // 1 MB offset
) (
    input  logic                    clk,
    input  logic                    rst_n,

    // Control
    input  logic                    start,
    input  logic [VERTEX_W-1:0]     source,
    output logic                    done,
    output logic [31:0]             visited_count,  // vertices reached

    // AXI4 read request port → axi4_rd_master
    output logic                    rd_req_valid,
    input  logic                    rd_req_ready,
    output logic [AXI_ADDR_W-1:0]   rd_req_addr,
    output logic [7:0]              rd_req_len,

    // AXI4 beat input ← axi4_rd_master
    input  logic                    beat_valid,
    input  logic [AXI_DATA_W-1:0]   beat_data,
    input  logic                    beat_last,
    output logic                    beat_ready,

    // Frontier FIFO write port → vertex_fifo
    // Frontier FIFO: packed as {level[15:0], vid[VERTEX_W-1:0]}
    output logic                        fifo_wr_valid,
    output logic [VERTEX_W+15:0]        fifo_wr_data,
    input  logic                        fifo_wr_ready,

    // Frontier FIFO read port ← vertex_fifo
    input  logic                        fifo_rd_valid,
    input  logic [VERTEX_W+15:0]        fifo_rd_data,
    output logic                        fifo_rd_ready,

    // Visited bitmap port → visited_bitmap
    output logic                    bm_op_valid,
    output logic [VERTEX_W-1:0]     bm_op_vid,
    output logic                    bm_op_set,
    input  logic                    bm_chk_valid,
    input  logic                    bm_chk_visited,
    input  logic                    bm_set_done,
    input  logic                    bm_busy,

    // Discovered vertex output stream (for verification / level capture)
    output logic                    out_valid,
    output logic [VERTEX_W-1:0]     out_vid,
    output logic [15:0]             out_level
);

    localparam int unsigned VERTS_PER_BEAT = AXI_DATA_W / 32;   // 8

    typedef enum logic [3:0] {
        S_IDLE        = 4'd0,
        S_INIT        = 4'd1,   // mark source visited, enqueue, assert out
        S_DEQUEUE     = 4'd2,   // pop frontier head → cur_vid, cur_level
        S_FETCH_PTR   = 4'd3,   // request row_ptr[u] and row_ptr[u+1] (1 beat)
        S_WAIT_PTR    = 4'd4,   // wait for beat containing both ptr words
        S_CHECK_EDGES = 4'd5,   // compute edge count; skip if zero
        S_ISSUE_EDGES = 4'd6,   // request col_idx burst
        S_RECV_BEAT   = 4'd7,   // receive one 256-bit beat of neighbor IDs
        S_SCATTER_RD  = 4'd8,   // bitmap check for current neighbor
        S_SCATTER_CHK = 4'd9,   // evaluate check; issue set if unvisited
        S_NEXT_EDGE   = 4'd10,  // advance within beat / to next beat
        S_DONE        = 4'd11
    } state_t;

    state_t state;

    // Current vertex being expanded
    logic [VERTEX_W-1:0]  cur_vid;
    logic [15:0]           cur_level;

    // Edge range for current vertex
    logic [31:0]  edge_start, edge_end, edge_remaining;

    // Beat buffer: holds one 256-bit beat = 8 neighbor IDs
    logic [31:0]  beat_buf [0:VERTS_PER_BEAT-1];
    logic [2:0]   beat_idx;    // current position within beat (0..7)
    logic [31:0]  beat_edges_remaining;  // edges remaining in this burst

    // Pending enqueue when scatter finds unvisited vertex
    logic          scatter_enqueue_pending;
    logic [VERTEX_W-1:0] scatter_vid_pending;

    // ------------------------------------------------------------------
    // Intermediate signals (avoid N'(expr) casts for iverilog compat)
    // ------------------------------------------------------------------

    localparam [AXI_ADDR_W-1:0] ROW_BASE = ROW_PTR_BASE;
    localparam [AXI_ADDR_W-1:0] COL_BASE = COL_IDX_BASE;

    logic [AXI_ADDR_W-1:0]  ptr_req_addr;
    logic [AXI_ADDR_W-1:0]  edge_req_addr;
    logic [7:0]             edge_req_len;
    logic [VERTEX_W-1:0]    cur_neighbor;
    logic [31:0]            edge_beats;   // ceil(edge_remaining / VERTS_PER_BEAT)

    assign ptr_req_addr  = ROW_BASE + {{(AXI_ADDR_W-VERTEX_W-2){1'b0}}, cur_vid, 2'b00};
    assign edge_req_addr = COL_BASE + {2'b00, edge_start, 2'b00};
    assign edge_beats    = (edge_remaining + VERTS_PER_BEAT - 1) / VERTS_PER_BEAT;
    assign edge_req_len  = edge_beats[7:0] - 8'd1;
    assign cur_neighbor  = beat_buf[beat_idx][VERTEX_W-1:0];

    // ------------------------------------------------------------------
    // Combinational outputs
    // ------------------------------------------------------------------

    assign done = (state == S_DONE);

    assign fifo_rd_ready = (state == S_DEQUEUE) && fifo_rd_valid;

    // AXI read request
    always_comb begin
        rd_req_valid = 1'b0;
        rd_req_addr  = '0;
        rd_req_len   = 8'd0;

        case (state)
            S_FETCH_PTR: begin
                rd_req_valid = 1'b1;
                rd_req_addr  = ptr_req_addr;
                rd_req_len   = 8'd0;
            end
            S_ISSUE_EDGES: begin
                rd_req_valid = 1'b1;
                rd_req_addr  = edge_req_addr;
                rd_req_len   = edge_req_len;
            end
            default: ;
        endcase
    end

    // Bitmap op
    always_comb begin
        bm_op_valid = 1'b0;
        bm_op_vid   = '0;
        bm_op_set   = 1'b0;

        case (state)
            S_INIT: begin
                bm_op_valid = ~bm_busy;
                bm_op_vid   = source;
                bm_op_set   = 1'b1;
            end
            S_SCATTER_RD: begin
                bm_op_valid = ~bm_busy;
                bm_op_vid   = cur_neighbor;
                bm_op_set   = 1'b0;
            end
            S_SCATTER_CHK: begin
                // Issue set if check said unvisited
                bm_op_valid = bm_chk_valid && ~bm_chk_visited && ~bm_busy;
                bm_op_vid   = scatter_vid_pending;
                bm_op_set   = 1'b1;
            end
            default: ;
        endcase
    end

    // Accept beats in both ptr-fetch and edge-fetch states
    assign beat_ready = (state == S_WAIT_PTR) || (state == S_RECV_BEAT);

    // Frontier FIFO write: enqueue source in S_INIT, new vertices in S_SCATTER_CHK
    always_comb begin
        fifo_wr_valid = 1'b0;
        fifo_wr_data  = '0;
        case (state)
            S_INIT: begin
                fifo_wr_valid = bm_set_done;
                fifo_wr_data  = {16'd0, source};         // source at level 0
            end
            S_SCATTER_CHK: begin
                fifo_wr_valid = bm_set_done;
                fifo_wr_data  = {cur_level + 16'd1, scatter_vid_pending};
            end
            default: ;
        endcase
    end

    // Discovery output stream
    always_comb begin
        out_valid = 1'b0;
        out_vid   = '0;
        out_level = '0;
        case (state)
            S_INIT: begin
                out_valid = bm_set_done;
                out_vid   = source;
                out_level = '0;
            end
            S_SCATTER_CHK: begin
                out_valid = bm_set_done;
                out_vid   = scatter_vid_pending;
                out_level = cur_level + 16'd1;
            end
            default: ;
        endcase
    end

    // ------------------------------------------------------------------
    // Sequential FSM
    // ------------------------------------------------------------------

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                  <= S_IDLE;
            cur_vid                <= '0;
            cur_level              <= '0;
            edge_start             <= '0;
            edge_end               <= '0;
            edge_remaining         <= '0;
            beat_idx               <= '0;
            beat_edges_remaining   <= '0;
            scatter_enqueue_pending <= 1'b0;
            scatter_vid_pending    <= '0;
            visited_count          <= '0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start)
                        state <= S_INIT;
                end

                // Mark source visited and enqueue it.
                // bm_set_done (and thus fifo_wr_valid / out_valid) fires
                // one cycle after bm_op_valid is accepted.
                S_INIT: begin
                    if (bm_set_done) begin
                        visited_count <= visited_count + 1'b1;
                        state         <= S_DEQUEUE;
                    end
                end

                // Pop frontier head.  vertex_fifo has registered output:
                // rd_data captures the popped value on the cycle fifo_rd_ready fires.
                S_DEQUEUE: begin
                    if (fifo_rd_valid && fifo_rd_ready) begin
                        cur_vid   <= fifo_rd_data[VERTEX_W-1:0];
                        cur_level <= fifo_rd_data[VERTEX_W+15:VERTEX_W];
                        state     <= S_FETCH_PTR;
                    end else if (!fifo_rd_valid) begin
                        state     <= S_DONE;
                    end
                end

                S_FETCH_PTR: begin
                    if (rd_req_valid && rd_req_ready)
                        state <= S_WAIT_PTR;
                end

                // Beat contains row_ptr[u] in bits[31:0] and row_ptr[u+1] in bits[63:32].
                S_WAIT_PTR: begin
                    if (beat_valid && beat_ready) begin
                        edge_start <= beat_data[31:0];
                        edge_end   <= beat_data[63:32];
                        state      <= S_CHECK_EDGES;
                    end
                end

                S_CHECK_EDGES: begin
                    edge_remaining <= edge_end - edge_start;
                    if (edge_end == edge_start)
                        state <= S_DEQUEUE;       // leaf vertex, skip
                    else
                        state <= S_ISSUE_EDGES;
                end

                S_ISSUE_EDGES: begin
                    if (rd_req_valid && rd_req_ready) begin
                        beat_edges_remaining <= edge_remaining;
                        beat_idx             <= '0;
                        state                <= S_RECV_BEAT;
                    end
                end

                // Latch incoming beat into beat_buf.
                S_RECV_BEAT: begin
                    if (beat_valid) begin
                        for (int i = 0; i < VERTS_PER_BEAT; i++)
                            beat_buf[i] <= beat_data[32*i +: 32];
                        beat_idx <= '0;
                        state    <= S_SCATTER_RD;
                    end
                end

                // Issue bitmap check (read) for beat_buf[beat_idx].
                S_SCATTER_RD: begin
                    if (!bm_busy) begin
                        scatter_vid_pending <= beat_buf[beat_idx][VERTEX_W-1:0];
                        state               <= S_SCATTER_CHK;
                    end
                end

                // Check result arrives (bm_chk_valid).
                // If unvisited: issue bitmap set (triggers fifo_wr + out_valid).
                S_SCATTER_CHK: begin
                    if (bm_chk_valid) begin
                        if (!bm_chk_visited) begin
                            // Bitmap set issued combinatorially; wait for set_done
                            // before advancing (set_done fires next cycle).
                        end else begin
                            state <= S_NEXT_EDGE;
                        end
                    end
                    if (bm_set_done) begin
                        visited_count <= visited_count + 1'b1;
                        state         <= S_NEXT_EDGE;
                    end
                end

                S_NEXT_EDGE: begin
                    beat_edges_remaining <= beat_edges_remaining - 1'b1;
                    if (beat_edges_remaining == 1) begin
                        // All edges in this burst processed
                        state <= S_DEQUEUE;
                    end else begin
                        // More edges in this burst
                        if (beat_idx == (VERTS_PER_BEAT-1)) begin
                            // Need next beat — re-enter RECV_BEAT.
                            // The AXI master is still streaming remaining beats.
                            beat_idx <= '0;
                            state    <= S_RECV_BEAT;
                        end else begin
                            beat_idx <= beat_idx + 1'b1;
                            state    <= S_SCATTER_RD;
                        end
                    end
                end

                S_DONE: ;   // hold until reset or new start

                default: state <= S_IDLE;
            endcase
        end
    end

    // cur_level tracking: increment when we transition from one BFS level
    // to the next. Simple approach: store level alongside vertex in FIFO
    // (not done here to keep FIFO generic) — instead, we derive level from
    // the out_level signal, which is cur_level+1 for each discovered neighbor.
    // cur_level is updated in DEQUEUE from a separate level FIFO in a full
    // implementation; here we leave it as a noted extension point for clarity.

endmodule
