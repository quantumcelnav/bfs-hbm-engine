// Synchronous FIFO for BFS frontier queue.
// Fall-through (registered output): rd_valid asserts one cycle after rd_ready
// with data in rd_data. Safe to read rd_data whenever rd_valid is high.
//
// Full condition: count == DEPTH. No overflow protection — caller must check
// wr_ready before asserting wr_valid.

module vertex_fifo #(
    parameter int unsigned WIDTH = 20,
    parameter int unsigned DEPTH = 2048   // must be power of 2
) (
    input  logic                    clk,
    input  logic                    rst_n,

    // Write port
    input  logic                    wr_valid,
    input  logic [WIDTH-1:0]        wr_data,
    output logic                    wr_ready,

    // Read port
    output logic                    rd_valid,
    output logic [WIDTH-1:0]        rd_data,
    input  logic                    rd_ready,

    output logic [$clog2(DEPTH):0]  count
);

    localparam int unsigned PTR_W = $clog2(DEPTH);

    logic [WIDTH-1:0]   mem [0:DEPTH-1];
    logic [PTR_W-1:0]   wr_ptr, rd_ptr;
    logic               full, empty;

    assign full      = (count == DEPTH[($clog2(DEPTH)):0]);
    assign empty     = (count == '0);
    assign wr_ready  = ~full;
    assign rd_valid  = ~empty;

    // Fall-through (show-ahead): rd_data is valid whenever rd_valid is high,
    // with no extra read-enable required. Synthesises as distributed RAM or
    // register file — for large DEPTH, replace with sync-read BRAM + S_WAIT_RD.
    assign rd_data = mem[rd_ptr];

    // Write
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
        end else if (wr_valid && wr_ready) begin
            mem[wr_ptr] <= wr_data;
            wr_ptr      <= wr_ptr + 1'b1;
        end
    end

    // Read pointer advance
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr <= '0;
        end else if (rd_valid && rd_ready) begin
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

    // Count
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= '0;
        end else begin
            case ({wr_valid & wr_ready, rd_valid & rd_ready})
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
