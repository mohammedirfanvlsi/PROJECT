
    `timescale 1ns / 1ps

    module dma_stream_top #(
        parameter DATA_WIDTH = 32,
        parameter ADDR_WIDTH = 32
    )(
        input  wire                   clk,
        input  wire                   reset_n,

        // CPU Slave Control
        input  wire                   reg_wr_en,
        input  wire [3:0]             reg_addr,
        input  wire [DATA_WIDTH-1:0]  reg_wdata,
        output wire [DATA_WIDTH-1:0]  reg_rdata,
        output wire                   irq,

        // AXI-Stream Input
        input  wire [DATA_WIDTH-1:0]  s_axis_tdata,
        input  wire                   s_axis_tvalid,
        output wire                   s_axis_tready,
        input  wire                   s_axis_tlast,

        // AXI4-Full Master Write Channels
        output wire [3:0]             m_axi_awid,
        output wire [ADDR_WIDTH-1:0]  m_axi_awaddr,
        output wire [7:0]             m_axi_awlen,
        output wire [2:0]             m_axi_awsize,
        output wire [1:0]             m_axi_awburst,
        output wire                   m_axi_awvalid,
        input  wire                   m_axi_awready,

        output wire [DATA_WIDTH-1:0]  m_axi_wdata,
        output wire [3:0]             m_axi_wstrb,
        output wire                   m_axi_wlast,
        output wire                   m_axi_wvalid,
        input  wire                   m_axi_wready,

        input  wire [3:0]             m_axi_bid,
        input  wire [1:0]             m_axi_bresp,
        input  wire                   m_axi_bvalid,
        output wire                   m_axi_bready,

        // AXI4-Full Master Read Channels (Unused / Tied off)
        output wire [3:0]             m_axi_arid,
        output wire [ADDR_WIDTH-1:0]  m_axi_araddr,
        output wire [7:0]             m_axi_arlen,
        output wire [2:0]             m_axi_arsize,
        output wire [1:0]             m_axi_arburst,
        output wire                   m_axi_arvalid,
        input  wire                   m_axi_arready,
        input  wire [3:0]             m_axi_rid,
        input  wire [DATA_WIDTH-1:0]  m_axi_rdata,
        input  wire [1:0]             m_axi_rresp,
        input  wire                   m_axi_rlast,
        input  wire                   m_axi_rvalid,
        output wire                   m_axi_rready
    );

        // Wires between Core, FIFO, and AXI Master
        wire                  fifo_wr_en;
        wire [DATA_WIDTH-1:0] fifo_wr_data;
        wire                  fifo_full;
        wire                  fifo_empty;
        wire [4:0]            fifo_count;

        wire                  fifo_rd_en;
        wire [DATA_WIDTH-1:0] fifo_rd_data;

        wire                  m_init_write;
        wire [ADDR_WIDTH-1:0] m_wr_addr;
        wire [7:0]            m_wr_len;
        wire                  m_wr_done;
        wire                  wr_data_req;

        assign fifo_rd_en = wr_data_req;

        // 1. FIFO (16 words)
        dma_fifo #(
            .DATA_WIDTH (DATA_WIDTH),
            .FIFO_DEPTH (16),
            .ADDR_WIDTH (4)
        ) u_fifo (
            .clk        (clk),
            .rst_n      (reset_n),
            .wr_en      (fifo_wr_en),
            .wr_data    (fifo_wr_data),
            .full       (fifo_full),
            .rd_en      (fifo_rd_en),
            .rd_data    (fifo_rd_data),
            .empty      (fifo_empty),
            .fifo_count (fifo_count)
        );

        // 2. DMA Stream-to-Memory Core
        dma_stream_to_mem_core #(
            .DATA_WIDTH   (DATA_WIDTH),
            .ADDR_WIDTH   (ADDR_WIDTH)
        ) u_core (
            .clk          (clk),
            .rst_n        (reset_n),
            .reg_wr_en    (reg_wr_en),
            .reg_addr     (reg_addr),
            .reg_wdata    (reg_wdata),
            .reg_rdata    (reg_rdata),
            .irq          (irq),
            .s_axis_tdata (s_axis_tdata),
            .s_axis_tvalid(s_axis_tvalid),
            .s_axis_tready(s_axis_tready),
            .s_axis_tlast (s_axis_tlast),
            .fifo_wr_en   (fifo_wr_en),
            .fifo_wr_data (fifo_wr_data),
            .fifo_full    (fifo_full),
            .fifo_count   (fifo_count),
            .m_init_write (m_init_write),
            .m_wr_addr    (m_wr_addr),
            .m_wr_len     (m_wr_len),
            .m_wr_done    (m_wr_done)
        );

        // 3. AXI4 Master
        axi_master #(
            .DATA_WIDTH    (DATA_WIDTH),
            .ADDR_WIDTH    (ADDR_WIDTH),
            .ID_WIDTH      (4)
        ) u_master (
            .aclk          (clk),
            .aresetn       (reset_n),
            .init_write    (m_init_write),
            .wr_addr       (m_wr_addr),
            .wr_len        (m_wr_len),
            .wr_data_in    (fifo_rd_data),
            .wr_data_req   (wr_data_req),
            .wr_done       (m_wr_done),

            .init_read     (1'b0),
            .rd_addr       ({ADDR_WIDTH{1'b0}}),
            .rd_len        (8'd0),
            .rd_data_out   (),
            .rd_data_valid (),
            .rd_done       (),

            .m_axi_awid    (m_axi_awid),
            .m_axi_awaddr  (m_axi_awaddr),
            .m_axi_awlen   (m_axi_awlen),
            .m_axi_awsize  (m_axi_awsize),
            .m_axi_awburst (m_axi_awburst),
            .m_axi_awvalid (m_axi_awvalid),
            .m_axi_awready (m_axi_awready),

            .m_axi_wdata   (m_axi_wdata),
            .m_axi_wstrb   (m_axi_wstrb),
            .m_axi_wlast   (m_axi_wlast),
            .m_axi_wvalid  (m_axi_wvalid),
            .m_axi_wready  (m_axi_wready),

            .m_axi_bid     (m_axi_bid),
            .m_axi_bresp   (m_axi_bresp),
            .m_axi_bvalid  (m_axi_bvalid),
            .m_axi_bready  (m_axi_bready),

            .m_axi_arid    (m_axi_arid),
            .m_axi_araddr  (m_axi_araddr),
            .m_axi_arlen   (m_axi_arlen),
            .m_axi_arsize  (m_axi_arsize),
            .m_axi_arburst (m_axi_arburst),
            .m_axi_arvalid (m_axi_arvalid),
            .m_axi_arready (m_axi_arready),

            .m_axi_rid     (m_axi_rid),
            .m_axi_rdata   (m_axi_rdata),
            .m_axi_rresp   (m_axi_rresp),
            .m_axi_rlast   (m_axi_rlast),
            .m_axi_rvalid  (m_axi_rvalid),
            .m_axi_rready  (m_axi_rready)
        );

    endmodule