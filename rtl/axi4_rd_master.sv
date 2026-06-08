// AXI4 read-only master.
//
// Issues a single outstanding read transaction at a time.
// Address and burst-length are presented on the req port; beat data is
// forwarded on the beat port with full backpressure support.
//
// AXI4 constraints enforced:
//   - ARSIZE fixed to log2(AXI_DATA_W/8) — one full data-bus word per beat
//   - ARBURST = INCR (2'b01)
//   - ARID    = AXI_ID_W'(0)
//   - ARLOCK  = 1'b0 (normal)
//   - ARCACHE = 4'b0010 (normal non-cacheable bufferable)
//   - ARPROT  = 3'b010 (unprivileged, non-secure, data)
//   - ARQOS   = 4'b0000

module axi4_rd_master #(
    parameter int unsigned AXI_DATA_W = 256,  // bits; must match HBM channel
    parameter int unsigned AXI_ADDR_W = 33,
    parameter int unsigned AXI_ID_W   = 4
) (
    input  logic                       clk,
    input  logic                       rst_n,

    // Request port (from BFS controller)
    input  logic                       req_valid,
    output logic                       req_ready,
    input  logic [AXI_ADDR_W-1:0]      req_addr,
    input  logic [7:0]                 req_len,    // ARLEN (beats-1)

    // Beat output port (to BFS controller)
    output logic                       beat_valid,
    output logic [AXI_DATA_W-1:0]      beat_data,
    output logic                       beat_last,
    input  logic                       beat_ready,

    // AXI4 AR channel (to HBM controller)
    output logic                       m_axi_arvalid,
    input  logic                       m_axi_arready,
    output logic [AXI_ADDR_W-1:0]      m_axi_araddr,
    output logic [7:0]                 m_axi_arlen,
    output logic [2:0]                 m_axi_arsize,
    output logic [1:0]                 m_axi_arburst,
    output logic [AXI_ID_W-1:0]        m_axi_arid,
    output logic                       m_axi_arlock,
    output logic [3:0]                 m_axi_arcache,
    output logic [2:0]                 m_axi_arprot,
    output logic [3:0]                 m_axi_arqos,

    // AXI4 R channel (from HBM controller)
    input  logic                       m_axi_rvalid,
    output logic                       m_axi_rready,
    input  logic [AXI_DATA_W-1:0]      m_axi_rdata,
    input  logic                       m_axi_rlast,
    input  logic [1:0]                 m_axi_rresp,
    input  logic [AXI_ID_W-1:0]        m_axi_rid
);

    localparam [2:0] ARSIZE_FULL = $clog2(AXI_DATA_W / 8);

    typedef enum logic [1:0] {
        S_IDLE  = 2'd0,  // waiting for request
        S_ADDR  = 2'd1,  // AR channel handshake in progress
        S_DATA  = 2'd2   // receiving R channel beats
    } state_t;

    state_t state;

    // Fixed AR channel fields
    assign m_axi_arsize  = ARSIZE_FULL;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arid    = '0;
    assign m_axi_arlock  = 1'b0;
    assign m_axi_arcache = 4'b0010;
    assign m_axi_arprot  = 3'b010;
    assign m_axi_arqos   = 4'b0000;

    // AR channel registers
    logic [AXI_ADDR_W-1:0] ar_addr_r;
    logic [7:0]             ar_len_r;

    // Accept new request only in IDLE
    assign req_ready = (state == S_IDLE);

    // AR valid: held high from ADDR entry until ARREADY
    assign m_axi_arvalid = (state == S_ADDR);
    assign m_axi_araddr  = ar_addr_r;
    assign m_axi_arlen   = ar_len_r;

    // R channel: accept data whenever consumer is ready
    assign m_axi_rready = beat_ready;
    assign beat_valid   = (state == S_DATA) && m_axi_rvalid;
    assign beat_data    = m_axi_rdata;
    assign beat_last    = m_axi_rlast;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            ar_addr_r <= '0;
            ar_len_r  <= '0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (req_valid) begin
                        ar_addr_r <= req_addr;
                        ar_len_r  <= req_len;
                        state     <= S_ADDR;
                    end
                end

                S_ADDR: begin
                    if (m_axi_arready)
                        state <= S_DATA;
                end

                S_DATA: begin
                    if (m_axi_rvalid && m_axi_rready && m_axi_rlast)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
