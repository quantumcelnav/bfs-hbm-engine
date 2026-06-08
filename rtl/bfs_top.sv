// BFS-HBM top-level integration.
//
// Instantiates:
//   bfs_ctrl       — traversal FSM
//   axi4_rd_master — HBM read interface
//   vertex_fifo    — frontier queue (vertex IDs only)
//   visited_bitmap — on-chip visited tracker (BRAM)
//
// External interface: AXI4 read-only to HBM controller (AR + R channels).
// Graph data layout in HBM is described in bfs_ctrl.sv.

module bfs_top #(
    parameter int unsigned VERTEX_W     = 20,
    parameter int unsigned AXI_DATA_W   = 256,
    parameter int unsigned AXI_ADDR_W   = 33,
    parameter int unsigned AXI_ID_W     = 4,
    parameter int unsigned FIFO_DEPTH   = 2048,
    parameter longint unsigned ROW_PTR_BASE = 33'h0_0000_0000,
    parameter longint unsigned COL_IDX_BASE = 33'h0_0010_0000
) (
    input  logic                    clk,
    input  logic                    rst_n,

    // BFS control
    input  logic                    start,
    input  logic [VERTEX_W-1:0]     source,
    output logic                    done,
    output logic [31:0]             visited_count,

    // Discovered vertex output stream
    output logic                    out_valid,
    output logic [VERTEX_W-1:0]     out_vid,
    output logic [15:0]             out_level,

    // AXI4 AR channel → HBM
    output logic                    m_axi_arvalid,
    input  logic                    m_axi_arready,
    output logic [AXI_ADDR_W-1:0]   m_axi_araddr,
    output logic [7:0]              m_axi_arlen,
    output logic [2:0]              m_axi_arsize,
    output logic [1:0]              m_axi_arburst,
    output logic [AXI_ID_W-1:0]     m_axi_arid,
    output logic                    m_axi_arlock,
    output logic [3:0]              m_axi_arcache,
    output logic [2:0]              m_axi_arprot,
    output logic [3:0]              m_axi_arqos,

    // AXI4 R channel ← HBM
    input  logic                    m_axi_rvalid,
    output logic                    m_axi_rready,
    input  logic [AXI_DATA_W-1:0]   m_axi_rdata,
    input  logic                    m_axi_rlast,
    input  logic [1:0]              m_axi_rresp,
    input  logic [AXI_ID_W-1:0]     m_axi_rid
);

    // Internal wires: ctrl ↔ axi4_rd_master
    logic                   rd_req_valid, rd_req_ready;
    logic [AXI_ADDR_W-1:0]  rd_req_addr;
    logic [7:0]             rd_req_len;
    logic                   beat_valid, beat_last, beat_ready;
    logic [AXI_DATA_W-1:0]  beat_data;

    // Internal wires: ctrl ↔ vertex_fifo  (packed {level[15:0], vid[VERTEX_W-1:0]})
    logic                       fifo_wr_valid, fifo_wr_ready;
    logic [VERTEX_W+15:0]       fifo_wr_data;
    logic                       fifo_rd_valid, fifo_rd_ready;
    logic [VERTEX_W+15:0]       fifo_rd_data;

    // Internal wires: ctrl ↔ visited_bitmap
    logic                   bm_op_valid, bm_op_set, bm_busy;
    logic [VERTEX_W-1:0]    bm_op_vid;
    logic                   bm_chk_valid, bm_chk_visited, bm_set_done;

    // ------------------------------------------------------------------

    bfs_ctrl #(
        .VERTEX_W    (VERTEX_W),
        .AXI_DATA_W  (AXI_DATA_W),
        .AXI_ADDR_W  (AXI_ADDR_W),
        .ROW_PTR_BASE(ROW_PTR_BASE),
        .COL_IDX_BASE(COL_IDX_BASE)
    ) u_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .source         (source),
        .done           (done),
        .visited_count  (visited_count),
        .rd_req_valid   (rd_req_valid),
        .rd_req_ready   (rd_req_ready),
        .rd_req_addr    (rd_req_addr),
        .rd_req_len     (rd_req_len),
        .beat_valid     (beat_valid),
        .beat_data      (beat_data),
        .beat_last      (beat_last),
        .beat_ready     (beat_ready),
        .fifo_wr_valid  (fifo_wr_valid),
        .fifo_wr_data   (fifo_wr_data),
        .fifo_wr_ready  (fifo_wr_ready),
        .fifo_rd_valid  (fifo_rd_valid),
        .fifo_rd_data   (fifo_rd_data),
        .fifo_rd_ready  (fifo_rd_ready),
        .bm_op_valid    (bm_op_valid),
        .bm_op_vid      (bm_op_vid),
        .bm_op_set      (bm_op_set),
        .bm_chk_valid   (bm_chk_valid),
        .bm_chk_visited (bm_chk_visited),
        .bm_set_done    (bm_set_done),
        .bm_busy        (bm_busy),
        .out_valid      (out_valid),
        .out_vid        (out_vid),
        .out_level      (out_level)
    );

    axi4_rd_master #(
        .AXI_DATA_W(AXI_DATA_W),
        .AXI_ADDR_W(AXI_ADDR_W),
        .AXI_ID_W  (AXI_ID_W)
    ) u_rd_master (
        .clk            (clk),
        .rst_n          (rst_n),
        .req_valid      (rd_req_valid),
        .req_ready      (rd_req_ready),
        .req_addr       (rd_req_addr),
        .req_len        (rd_req_len),
        .beat_valid     (beat_valid),
        .beat_data      (beat_data),
        .beat_last      (beat_last),
        .beat_ready     (beat_ready),
        .m_axi_arvalid  (m_axi_arvalid),
        .m_axi_arready  (m_axi_arready),
        .m_axi_araddr   (m_axi_araddr),
        .m_axi_arlen    (m_axi_arlen),
        .m_axi_arsize   (m_axi_arsize),
        .m_axi_arburst  (m_axi_arburst),
        .m_axi_arid     (m_axi_arid),
        .m_axi_arlock   (m_axi_arlock),
        .m_axi_arcache  (m_axi_arcache),
        .m_axi_arprot   (m_axi_arprot),
        .m_axi_arqos    (m_axi_arqos),
        .m_axi_rvalid   (m_axi_rvalid),
        .m_axi_rready   (m_axi_rready),
        .m_axi_rdata    (m_axi_rdata),
        .m_axi_rlast    (m_axi_rlast),
        .m_axi_rresp    (m_axi_rresp),
        .m_axi_rid      (m_axi_rid)
    );

    vertex_fifo #(
        .WIDTH(VERTEX_W + 16),
        .DEPTH(FIFO_DEPTH)
    ) u_frontier (
        .clk      (clk),
        .rst_n    (rst_n),
        .wr_valid (fifo_wr_valid),
        .wr_data  (fifo_wr_data),
        .wr_ready (fifo_wr_ready),
        .rd_valid (fifo_rd_valid),
        .rd_data  (fifo_rd_data),
        .rd_ready (fifo_rd_ready),
        .count    ()
    );

    visited_bitmap #(
        .VERTEX_W(VERTEX_W)
    ) u_visited (
        .clk         (clk),
        .rst_n       (rst_n),
        .op_valid    (bm_op_valid),
        .op_vid      (bm_op_vid),
        .op_set      (bm_op_set),
        .chk_valid   (bm_chk_valid),
        .chk_visited (bm_chk_visited),
        .set_done    (bm_set_done),
        .busy        (bm_busy)
    );

endmodule
