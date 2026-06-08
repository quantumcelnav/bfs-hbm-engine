// BFS engine testbench.
//
// Graph under test (8 vertices, undirected):
//
//   0 — 1 — 2 — 3
//   |       |
//   4 — 5 — 6 — 7
//
// Adjacency lists (sorted):
//   0: [1, 4]      1: [0, 2, 5]   2: [1, 3, 6]   3: [2, 7]
//   4: [0, 5]      5: [1, 4, 6]   6: [2, 5, 7]   7: [3, 6]
//
// CSR encoding:
//   row_ptr = {0, 2, 5, 8, 10, 12, 15, 18, 20}  (9 words)
//   col_idx = {1,4, 0,2,5, 1,3,6, 2,7, 0,5, 1,4,6, 2,5,7, 3,6}  (20 words)
//
// Expected BFS levels from source vertex 0:
//   v0:0  v1:1  v2:2  v3:3  v4:1  v5:2  v6:3  v7:4
//
// ROW_PTR_BASE = 0x0000_0000  (row_ptr array)
// COL_IDX_BASE = 0x0010_0000  (col_idx array, 1 MB offset)

`timescale 1ns/1ps

module bfs_tb;

    localparam int unsigned VERTEX_W    = 6;    // 64-vertex space (test uses 8)
    localparam int unsigned AXI_DATA_W  = 256;
    localparam int unsigned AXI_ADDR_W  = 33;
    localparam int unsigned AXI_ID_W    = 4;
    localparam int unsigned FIFO_DEPTH  = 32;
    localparam longint unsigned ROW_PTR_BASE = 33'h0_0000_0000;
    localparam longint unsigned COL_IDX_BASE = 33'h0_0000_0040;  // 64B after row_ptr

    localparam int CLK_PERIOD = 4;   // 250 MHz (HBM4 domain)

    // ── Clock & reset ──────────────────────────────────────────────────
    logic clk, rst_n;
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    initial begin
        rst_n = 0;
        repeat(4) @(posedge clk);
        rst_n = 1;
    end

    // ── DUT ────────────────────────────────────────────────────────────
    logic                   start, done;
    logic [VERTEX_W-1:0]    source;
    logic [31:0]            visited_count;
    logic                   out_valid;
    logic [VERTEX_W-1:0]    out_vid;
    logic [15:0]            out_level;

    // AXI4 wires (DUT → memory model)
    logic                   arvalid, arready;
    logic [AXI_ADDR_W-1:0]  araddr;
    logic [7:0]             arlen;
    logic [2:0]             arsize;
    logic [1:0]             arburst;
    logic [AXI_ID_W-1:0]    arid;
    logic                   arlock;
    logic [3:0]             arcache, arqos;
    logic [2:0]             arprot;
    logic                   rvalid, rready;
    logic [AXI_DATA_W-1:0]  rdata;
    logic                   rlast;
    logic [1:0]             rresp;
    logic [AXI_ID_W-1:0]    rid;

    bfs_top #(
        .VERTEX_W    (VERTEX_W),
        .AXI_DATA_W  (AXI_DATA_W),
        .AXI_ADDR_W  (AXI_ADDR_W),
        .AXI_ID_W    (AXI_ID_W),
        .FIFO_DEPTH  (FIFO_DEPTH),
        .ROW_PTR_BASE(ROW_PTR_BASE),
        .COL_IDX_BASE(COL_IDX_BASE)
    ) u_dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .source         (source),
        .done           (done),
        .visited_count  (visited_count),
        .out_valid      (out_valid),
        .out_vid        (out_vid),
        .out_level      (out_level),
        .m_axi_arvalid  (arvalid),
        .m_axi_arready  (arready),
        .m_axi_araddr   (araddr),
        .m_axi_arlen    (arlen),
        .m_axi_arsize   (arsize),
        .m_axi_arburst  (arburst),
        .m_axi_arid     (arid),
        .m_axi_arlock   (arlock),
        .m_axi_arcache  (arcache),
        .m_axi_arprot   (arprot),
        .m_axi_arqos    (arqos),
        .m_axi_rvalid   (rvalid),
        .m_axi_rready   (rready),
        .m_axi_rdata    (rdata),
        .m_axi_rlast    (rlast),
        .m_axi_rresp    (rresp),
        .m_axi_rid      (rid)
    );

    // ── Memory model ───────────────────────────────────────────────────
    hbm_mem_model #(
        .AXI_DATA_W     (AXI_DATA_W),
        .AXI_ADDR_W     (AXI_ADDR_W),
        .AXI_ID_W       (AXI_ID_W),
        .MEM_DEPTH_WORDS(65536),
        .HBM_LATENCY_CYC(20)
    ) u_mem (
        .clk            (clk),
        .rst_n          (rst_n),
        .s_axi_arvalid  (arvalid),
        .s_axi_arready  (arready),
        .s_axi_araddr   (araddr),
        .s_axi_arlen    (arlen),
        .s_axi_arsize   (arsize),
        .s_axi_arburst  (arburst),
        .s_axi_arid     (arid),
        .s_axi_rvalid   (rvalid),
        .s_axi_rready   (rready),
        .s_axi_rdata    (rdata),
        .s_axi_rlast    (rlast),
        .s_axi_rresp    (rresp),
        .s_axi_rid      (rid)
    );

    // ── Graph initialisation ───────────────────────────────────────────
    // row_ptr[0..8] at ROW_PTR_BASE (byte offset 0x00000000)
    // col_idx[0..19] at COL_IDX_BASE (byte offset 0x00100000)

    localparam int NVERTICES = 8;
    localparam int NEDGES    = 20;

    logic [31:0] row_ptr [0:NVERTICES];
    logic [31:0] col_idx [0:NEDGES-1];

    initial begin
        // row_ptr
        row_ptr[0] = 0;   // vertex 0 edges start at col_idx[0]
        row_ptr[1] = 2;
        row_ptr[2] = 5;
        row_ptr[3] = 8;
        row_ptr[4] = 10;
        row_ptr[5] = 12;
        row_ptr[6] = 15;
        row_ptr[7] = 18;
        row_ptr[8] = 20;  // sentinel

        // col_idx (adjacency lists packed)
        // v0: [1,4]
        col_idx[0]  = 1;  col_idx[1]  = 4;
        // v1: [0,2,5]
        col_idx[2]  = 0;  col_idx[3]  = 2;  col_idx[4]  = 5;
        // v2: [1,3,6]
        col_idx[5]  = 1;  col_idx[6]  = 3;  col_idx[7]  = 6;
        // v3: [2,7]
        col_idx[8]  = 2;  col_idx[9]  = 7;
        // v4: [0,5]
        col_idx[10] = 0;  col_idx[11] = 5;
        // v5: [1,4,6]
        col_idx[12] = 1;  col_idx[13] = 4;  col_idx[14] = 6;
        // v6: [2,5,7]
        col_idx[15] = 2;  col_idx[16] = 5;  col_idx[17] = 7;
        // v7: [3,6]
        col_idx[18] = 3;  col_idx[19] = 6;

        // Load into memory model
        @(posedge rst_n);   // wait for reset release
        for (int i = 0; i <= NVERTICES; i++)
            u_mem.init_word(ROW_PTR_BASE + i*4, row_ptr[i]);
        for (int i = 0; i < NEDGES; i++)
            u_mem.init_word(COL_IDX_BASE + i*4, col_idx[i]);
    end

    // ── Golden reference ───────────────────────────────────────────────
    logic [15:0] golden_level [0:NVERTICES-1];
    initial begin
        golden_level[0] = 0;
        golden_level[1] = 1;
        golden_level[2] = 2;
        golden_level[3] = 3;
        golden_level[4] = 1;
        golden_level[5] = 2;
        golden_level[6] = 3;
        golden_level[7] = 4;
    end

    // ── Stimulus ───────────────────────────────────────────────────────
    logic [15:0] observed_level [0:NVERTICES-1];
    logic        observed_seen  [0:NVERTICES-1];
    int          total_cycles;
    int          errors;

    initial begin
        start  = 0;
        source = '0;
        for (int i = 0; i < NVERTICES; i++) begin
            observed_level[i] = 16'hFFFF;
            observed_seen[i]  = 0;
        end

        // Wait for reset and memory init
        @(posedge rst_n);
        repeat(5) @(posedge clk);

        // Kick off BFS from vertex 0
        source = '0;
        start  = 1'b1;
        @(posedge clk);
        start  = 1'b0;

        // Collect output stream
        fork
            begin : collect
                forever begin
                    @(posedge clk);
                    if (out_valid) begin
                        observed_level[out_vid] = out_level;
                        observed_seen [out_vid] = 1;
                        $display("[%0t] discovered v%0d at level %0d",
                                 $time, out_vid, out_level);
                    end
                end
            end
        join_none

        // Wait for done or timeout
        total_cycles = 0;
        while (!done && total_cycles < 100000) begin
            @(posedge clk);
            total_cycles++;
        end
        disable collect;

        repeat(2) @(posedge clk);

        // ── Result check ──────────────────────────────────────────────
        $display("\n==============================");
        $display("BFS complete in %0d cycles", total_cycles);
        $display("Visited count: %0d / %0d", visited_count, NVERTICES);
        $display("==============================");

        errors = 0;
        for (int v = 0; v < NVERTICES; v++) begin
            if (!observed_seen[v]) begin
                $display("FAIL: vertex %0d not reached", v);
                errors++;
            end else if (observed_level[v] !== golden_level[v]) begin
                $display("FAIL: v%0d level=%0d expected=%0d",
                         v, observed_level[v], golden_level[v]);
                errors++;
            end else begin
                $display("PASS: v%0d level=%0d", v, observed_level[v]);
            end
        end

        if (errors == 0)
            $display("\n*** ALL %0d VERTICES CORRECT ***\n", NVERTICES);
        else
            $display("\n*** %0d ERRORS ***\n", errors);

        if (!done)
            $display("TIMEOUT: BFS did not complete");

        $finish;
    end

    // ── Waveform dump ──────────────────────────────────────────────────
    initial begin
        $dumpfile("sim/bfs_waves.vcd");
        $dumpvars(0, bfs_tb);
    end

    // ── Timeout watchdog ──────────────────────────────────────────────
    initial begin
        #(100000 * CLK_PERIOD);
        $display("FATAL: simulation timeout");
        $finish;
    end

endmodule
