`timescale 1ns / 1ps

module tb_dma_stream;

    parameter DATA_WIDTH = 32;
    parameter ADDR_WIDTH = 32;
    parameter CLK_PERIOD = 10;

    reg                   clk;
    reg                   reset_n;

    // CPU Control
    reg                   reg_wr_en;
    reg  [3:0]            reg_addr;
    reg  [DATA_WIDTH-1:0] reg_wdata;
    wire [DATA_WIDTH-1:0] reg_rdata;
    wire                  irq;

    // AXI-Stream Input
    reg  [DATA_WIDTH-1:0] s_axis_tdata;
    reg                   s_axis_tvalid;
    wire                  s_axis_tready;
    reg                   s_axis_tlast;

    // Interconnect Wires
    wire [3:0]            m_axi_awid;
    wire [ADDR_WIDTH-1:0] m_axi_awaddr;
    wire [7:0]            m_axi_awlen;
    wire [2:0]            m_axi_awsize;
    wire [1:0]            m_axi_awburst;
    wire                  m_axi_awvalid;
    wire                  m_axi_awready;

    wire [DATA_WIDTH-1:0] m_axi_wdata;
    wire [3:0]            m_axi_wstrb;
    wire                  m_axi_wlast;
    wire                  m_axi_wvalid;
    wire                  m_axi_wready;

    wire [3:0]            m_axi_bid;
    wire [1:0]            m_axi_bresp;
    wire                  m_axi_bvalid;
    wire                  m_axi_bready;

    wire [3:0]            m_axi_arid;
    wire [ADDR_WIDTH-1:0] m_axi_araddr;
    wire [7:0]            m_axi_arlen;
    wire [2:0]            m_axi_arsize;
    wire [1:0]            m_axi_arburst;
    wire                  m_axi_arvalid;
    wire                  m_axi_arready;

    wire [3:0]            m_axi_rid;
    wire [DATA_WIDTH-1:0] m_axi_rdata;
    wire [1:0]            m_axi_rresp;
    wire                  m_axi_rlast;
    wire                  m_axi_rvalid;
    wire                  m_axi_rready;

    reg [DATA_WIDTH-1:0] expected_pixels [0:7];
    integer i;
    integer error_count = 0;

    // 1. INSTANTIATE DUT (dma_stream_top)
    dma_stream_top #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_dut (
        .clk            (clk),
        .reset_n        (reset_n),

        .reg_wr_en      (reg_wr_en),
        .reg_addr       (reg_addr),
        .reg_wdata      (reg_wdata),
        .reg_rdata      (reg_rdata),
        .irq            (irq),

        .s_axis_tdata   (s_axis_tdata),
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tready  (s_axis_tready),
        .s_axis_tlast   (s_axis_tlast),

        .m_axi_awid     (m_axi_awid),
        .m_axi_awaddr   (m_axi_awaddr),
        .m_axi_awlen    (m_axi_awlen),
        .m_axi_awsize   (m_axi_awsize),
        .m_axi_awburst  (m_axi_awburst),
        .m_axi_awvalid  (m_axi_awvalid),
        .m_axi_awready  (m_axi_awready),

        .m_axi_wdata    (m_axi_wdata),
        .m_axi_wstrb    (m_axi_wstrb),
        .m_axi_wlast    (m_axi_wlast),
        .m_axi_wvalid   (m_axi_wvalid),
        .m_axi_wready   (m_axi_wready),

        .m_axi_bid      (m_axi_bid),
        .m_axi_bresp    (m_axi_bresp),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),

        .m_axi_arid     (m_axi_arid),
        .m_axi_araddr   (m_axi_araddr),
        .m_axi_arlen    (m_axi_arlen),
        .m_axi_arsize   (m_axi_arsize),
        .m_axi_arburst  (m_axi_arburst),
        .m_axi_arvalid  (m_axi_arvalid),
        .m_axi_arready  (m_axi_arready),

        .m_axi_rid      (m_axi_rid),
        .m_axi_rdata    (m_axi_rdata),
        .m_axi_rresp    (m_axi_rresp),
        .m_axi_rlast    (m_axi_rlast),
        .m_axi_rvalid   (m_axi_rvalid),
        .m_axi_rready   (m_axi_rready)
    );

    // 2. INSTANTIATE MEMORY MODEL (axi_slave)
    axi_slave #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH),
        .ID_WIDTH   (4)
    ) u_mem (
        .aclk           (clk),
        .aresetn        (reset_n),

        .s_axi_awid     (m_axi_awid),
        .s_axi_awaddr   (m_axi_awaddr),
        .s_axi_awlen    (m_axi_awlen),
        .s_axi_awsize   (m_axi_awsize),
        .s_axi_awburst  (m_axi_awburst),
        .s_axi_awvalid  (m_axi_awvalid),
        .s_axi_awready  (m_axi_awready),

        .s_axi_wdata    (m_axi_wdata),
        .s_axi_wstrb    (m_axi_wstrb),
        .s_axi_wlast    (m_axi_wlast),
        .s_axi_wvalid   (m_axi_wvalid),
        .s_axi_wready   (m_axi_wready),

        .s_axi_bid      (m_axi_bid),
        .s_axi_bresp    (m_axi_bresp),
        .s_axi_bvalid   (m_axi_bvalid),
        .s_axi_bready   (m_axi_bready),

        .s_axi_arid     (m_axi_arid),
        .s_axi_araddr   (m_axi_araddr),
        .s_axi_arlen    (m_axi_arlen),
        .s_axi_arsize   (m_axi_arsize),
        .s_axi_arburst  (m_axi_arburst),
        .s_axi_arvalid  (m_axi_arvalid),
        .s_axi_arready  (m_axi_arready),

        .s_axi_rid      (m_axi_rid),
        .s_axi_rdata    (m_axi_rdata),
        .s_axi_rresp    (m_axi_rresp),
        .s_axi_rlast    (m_axi_rlast),
        .s_axi_rvalid   (m_axi_rvalid),
        .s_axi_rready   (m_axi_rready)
    );

    // 3. 100 MHz CLOCK GENERATOR
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // 4. TASKS
    task cpu_write_reg(input [3:0] addr, input [31:0] data);
    begin
        @(posedge clk);
        reg_wr_en <= 1'b1;
        reg_addr  <= addr;
        reg_wdata <= data;
        @(posedge clk);
        reg_wr_en <= 1'b0;
        reg_wdata <= 32'd0;
    end
    endtask

    task send_stream_pixel(input [31:0] pixel, input last);
    begin
        @(posedge clk);
        s_axis_tvalid <= 1'b1;
        s_axis_tdata  <= pixel;
        s_axis_tlast  <= last;

        while (!s_axis_tready) begin
            @(posedge clk);
        end

        @(posedge clk);
        s_axis_tvalid <= 1'b0;
        s_axis_tlast  <= 1'b0;
    end
    endtask

    // 5. TEST SCENARIO
    initial begin
        $dumpfile("tb_dma_stream.vcd");
        $dumpvars(0, tb_dma_stream);

        $display("\n========================================================");
        $display("   STARTING AXI-STREAM TO MEMORY DMA CONTROLLER TEST   ");
        $display("========================================================");

        reset_n       = 1'b0;
        reg_wr_en     = 1'b0;
        reg_addr      = 4'd0;
        reg_wdata     = 32'd0;
        s_axis_tvalid = 1'b0;
        s_axis_tdata  = 32'd0;
        s_axis_tlast  = 1'b0;

        #(CLK_PERIOD * 5);
        reset_n = 1'b1;
        #(CLK_PERIOD * 2);
        $display("[TIME: %0t] Reset released. System initialized.", $time);

        // CPU: Configure Destination Address = 0x40 (Word 16 in RAM)
        $display("[TIME: %0t] CPU: Setting DDR3 Destination Address = 0x0000_0040", $time);
        cpu_write_reg(4'h4, 32'h0000_0040);

        // CPU: START=1, IRQ_EN=1
        $display("[TIME: %0t] CPU: Writing CTRL register -> START=1, IRQ_EN=1", $time);
        cpu_write_reg(4'hC, 32'h0000_0003);

        #(CLK_PERIOD * 4);

        // Send 8 Pixels
        expected_pixels[0] = 32'hAAAA_0001;
        expected_pixels[1] = 32'hAAAA_0002;
        expected_pixels[2] = 32'hAAAA_0003;
        expected_pixels[3] = 32'hAAAA_0004;
        expected_pixels[4] = 32'hBBBB_0001;
        expected_pixels[5] = 32'hBBBB_0002;
        expected_pixels[6] = 32'hBBBB_0003;
        expected_pixels[7] = 32'hBBBB_0004;

        $display("[TIME: %0t] PC: Starting AXI-Stream pixel transmission...", $time);

        for (i = 0; i < 8; i = i + 1) begin
            send_stream_pixel(expected_pixels[i], (i == 7) ? 1'b1 : 1'b0);
            $display("[TIME: %0t] Streamed Pixel[%0d] = 0x%08X (TLAST = %0b)",
                     $time, i, expected_pixels[i], (i == 7));
        end

        // Wait for IRQ
        $display("[TIME: %0t] Waiting for DMA Hardware Interrupt (IRQ)...", $time);

        fork : wait_for_done
            begin
                @(posedge irq);
                $display("[TIME: %0t] >>> IRQ DETECTED! Full Frame Stored in Memory! <<<", $time);
                disable wait_for_done;
            end
            begin
                #5000;
                $display("[TIME: %0t] ERROR: Timeout! IRQ never asserted!", $time);
                error_count = error_count + 1;
                disable wait_for_done;
            end
        join

        #(CLK_PERIOD * 10);

        // Verify Memory Contents
        $display("\n========================================================");
        $display("          SELF-CHECKING MEMORY VERIFICATION            ");
        $display("========================================================");

        for (i = 0; i < 8; i = i + 1) begin
            if (u_mem.mem[16 + i] !== expected_pixels[i]) begin
                $display("FAIL: Word[%0d] at Addr 0x%08X | Expected: 0x%08X | Got: 0x%08X",
                         16 + i, 32'h0000_0040 + (i * 4), expected_pixels[i], u_mem.mem[16 + i]);
                error_count = error_count + 1;
            end else begin
                $display("PASS: Word[%0d] at Addr 0x%08X | Verified: 0x%08X",
                         16 + i, 32'h0000_0040 + (i * 4), u_mem.mem[16 + i]);
            end
        end

        $display("========================================================");
        if (error_count == 0) begin
            $display("    *** TEST PASSED! ALL 8 PIXELS STORED PERFECTLY! ***  ");
        end else begin
            $display("    *** TEST FAILED! ERRORS ENCOUNTERED: %0d ***         ", error_count);
        end
        $display("========================================================\n");

        $finish;
    end

endmodule
