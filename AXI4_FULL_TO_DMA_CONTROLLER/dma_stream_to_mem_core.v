`timescale 1ns / 1ps
    // ============================================================================
    // MODULE: dma_stream_to_mem_core
    // DESCRIPTION: Peripheral-to-Memory AXI DMA Controller.
    // FEATURES:
    //   - Receives video/pixel stream from Camera via AXI4-Stream interface
    //   - Pushes streaming pixels into internal FIFO buffer
    //   - Uses Watermark Threshold: Waits until FIFO has 4 words, then triggers AXI burst!
    //   - Commands AXI4-Full Master to burst-write pixels into DDR3 RAM (Framebuffer)
    //   - Frame Boundary Detection (s_axis_tlast): Asserts hardware interrupt (irq)
    // ============================================================================
    module dma_stream_to_mem_core #(
        parameter ADDR_WIDTH = 32,          // 32-bit memory address space (covers 4GB)
        parameter DATA_WIDTH = 32           // 32-bit pixel words
    )(
        // ========================================================================
        // 1. GLOBAL SIGNALS
        // ========================================================================
        input  wire                   clk,          // System clock (aclk = 100 MHz)
        input  wire                   rst_n,        // Active-LOW synchronous reset (0 = reset, 1 = run)

        // ========================================================================
        // 2. CPU CONFIGURATION REGISTERS (Slave Control Port)
        // ========================================================================
        input  wire                   reg_wr_en,    // CPU Write strobe
        input  wire [3:0]             reg_addr,     // Register offset: 0x4=DST_ADDR, 0xC=CTRL
        input  wire [DATA_WIDTH-1:0]  reg_wdata,    // 32-bit configuration data from CPU
        output reg  [DATA_WIDTH-1:0]  reg_rdata,    // 32-bit status read back by CPU
        output reg                    irq,          // Hardware Interrupt wire: Pulses 1 when full frame is stored!

        // ========================================================================
        // 3. AXI4-STREAM INPUT INTERFACE (From Camera / USB Stream)
        // ========================================================================
        input  wire [DATA_WIDTH-1:0]  s_axis_tdata, // Incoming 32-bit pixel data from Camera
        input  wire                   s_axis_tvalid,// Camera says: "Pixel data is valid!"
        output wire                   s_axis_tready,// DMA says: "I have room in my FIFO, send pixel!"
        input  wire                   s_axis_tlast, // Camera says: "This is the final pixel of the frame!"

        // ========================================================================
        // 4. FIFO CONTROL INTERFACE (Writing to dma_fifo.v)
        // ========================================================================
        output wire                   fifo_wr_en,   // FIFO Write Enable: Pushes pixel into FIFO
        output wire [DATA_WIDTH-1:0]  fifo_wr_data, // Pixel data going into FIFO
        input  wire                   fifo_full,    // From FIFO: High when FIFO is 100% full
        input  wire [4:0]             fifo_count,   // From FIFO: How many words are currently inside (0 to 16)

        // ========================================================================
        // 5. AXI4-FULL MASTER WRITE COMMAND INTERFACE (Writing to DDR3 RAM)
        // ========================================================================
        output reg                    m_init_write, // 1-cycle pulse to tell Master: "Launch AXI Write Burst!"
        output reg  [ADDR_WIDTH-1:0]  m_wr_addr,    // Memory address in DDR3 RAM where to write
        output reg  [7:0]             m_wr_len,     // Burst length minus 1 (3 means 4 beats)
        input  wire                   m_wr_done     // From Master: "Write burst complete (BRESP received)!"
    );

        // ========================================================================
        // REGISTER FILE FLIP-FLOPS
        // ========================================================================
        reg [ADDR_WIDTH-1:0] dst_addr_reg;  // Offset 0x4: Starting address in DDR3 RAM for the picture
        reg                  ctrl_start;    // Offset 0xC, Bit 0: Write 1 to enable camera capture
        reg                  ctrl_irq_en;   // Offset 0xC, Bit 1: Write 1 to enable hardware interrupt
        reg                  stat_busy;     // Offset 0x10, Bit 0: 1 = DMA actively capturing camera frames
        reg                  stat_done;     // Offset 0x10, Bit 1: 1 = Full frame successfully written to RAM

        // Internal Transfer Tracking Registers:
        reg [ADDR_WIDTH-1:0] cur_dst_addr;  // Steps forward (+16 bytes) after each 4-beat burst into DDR3
        reg                  frame_last_seen;// Latches when s_axis_tlast is seen (end of video frame)

        // ========================================================================
        // AXI-STREAM HANDSHAKE & FIFO WRITE LOGIC
        // ========================================================================
        // DMA is ready to accept pixels whenever the DMA is running (BUSY) and FIFO is NOT full:
        assign s_axis_tready = stat_busy && !fifo_full;

        // Push pixel into FIFO whenever BOTH tvalid and tready are HIGH:
        assign fifo_wr_en    = s_axis_tvalid && s_axis_tready;
        assign fifo_wr_data  = s_axis_tdata;


        // ========================================================================
        // FSM STATE CONSTANTS (4 States)
        // ========================================================================
        localparam S_IDLE       = 2'b00; // Waiting for CPU to write START = 1
        localparam S_COLLECT    = 2'b01; // Pixels filling FIFO; waiting for 4 words or frame end
        localparam S_BURST_WR   = 2'b10; // Triggering AXI Master to write 4 words into DDR3 RAM
        localparam S_DONE       = 2'b11; // Full video frame complete; pulse interrupt to CPU

        reg [1:0] state; // Current state register


        // ========================================================================
        // SECTION A: CPU REGISTER WRITE LOGIC (Slave Port)
        // ========================================================================
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                dst_addr_reg <= {ADDR_WIDTH{1'b0}};
                ctrl_start   <= 1'b0;
                ctrl_irq_en  <= 1'b0;
            end else begin
                // Clear single-cycle START strobe once DMA begins capturing:
                if (ctrl_start)
                    ctrl_start <= 1'b0;

                if (reg_wr_en) begin
                    case (reg_addr)
                        4'h4: dst_addr_reg <= reg_wdata;                  // Offset 0x4: Write Destination Address
                        4'hC: begin                                       // Offset 0xC: Write Control Register
                            ctrl_start  <= reg_wdata[0];                  // Bit 0 = START bit
                            ctrl_irq_en <= reg_wdata[1];                  // Bit 1 = IRQ Enable bit
                        end
                        default: ;
                    endcase
                end
            end
        end


        // ========================================================================
        // SECTION B: CPU REGISTER READBACK LOGIC
        // ========================================================================
        always @(*) begin
            case (reg_addr)
                4'h4: reg_rdata = dst_addr_reg;                                  // Read Destination Address
                4'hC: reg_rdata = {30'd0, ctrl_irq_en, ctrl_start};              // Read Control Register
                4'h8: reg_rdata = {30'd0, stat_done, stat_busy};                // Read Status: Bit 0=BUSY, Bit 1=DONE
                default: reg_rdata = 32'd0;
            endcase
        end


        // ========================================================================
        // SECTION C: PERIPHERAL-TO-MEMORY DMA STATE MACHINE (FSM)
        // ========================================================================
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                state           <= S_IDLE;
                m_init_write    <= 1'b0;
                m_wr_addr       <= {ADDR_WIDTH{1'b0}};
                m_wr_len        <= 8'd0;
                cur_dst_addr    <= {ADDR_WIDTH{1'b0}};
                frame_last_seen <= 1'b0;
                stat_busy       <= 1'b0;
                stat_done       <= 1'b0;
                irq             <= 1'b0;
            end else begin
                // Default pulse clears:
                m_init_write <= 1'b0;
                irq          <= 1'b0;

                // Latch frame end flag whenever camera sends TLAST:
                if (s_axis_tvalid && s_axis_tready && s_axis_tlast) begin
                    frame_last_seen <= 1'b1;
                end

                case (state)

                    // ------------------------------------------------------------
                    // STATE 1: S_IDLE - Waiting for CPU to launch camera capture
                    // ------------------------------------------------------------
                    S_IDLE: begin
                        stat_busy <= 1'b0;

                        if (ctrl_start) begin
                            cur_dst_addr    <= dst_addr_reg; // Load starting memory address (e.g. 0x8000_0000)
                            stat_busy       <= 1'b1;         // Set BUSY = 1
                            stat_done       <= 1'b0;         // Clear DONE = 0
                            frame_last_seen <= 1'b0;         // Reset frame end flag

                            state <= S_COLLECT;              // Begin collecting camera pixels!
                        end
                    end

                    // ------------------------------------------------------------
                    // STATE 2: S_COLLECT - Wait for FIFO to have 4 words (Watermark)
                    // ------------------------------------------------------------
                    S_COLLECT: begin
                        // Condition A: We have at least 4 words in the FIFO -> Burst write to DDR3!
                        if (fifo_count >= 5'd4) begin
                            m_wr_addr    <= cur_dst_addr;   // Target address in DDR3
                            m_wr_len     <= 8'd3;           // 3 means 3 + 1 = 4 beats
                            m_init_write <= 1'b1;           // Trigger AXI Master to write!

                            state <= S_BURST_WR;
                        end
                        // Condition B: Camera finished frame (TLAST seen) and FIFO has leftover words:
                        else if (frame_last_seen && (fifo_count > 0)) begin
                            m_wr_addr    <= cur_dst_addr;
                            m_wr_len     <= fifo_count[3:0] - 1'b1; // Burst whatever is leftover
                            m_init_write <= 1'b1;

                            state <= S_BURST_WR;
                        end
                        // Condition C: Camera finished frame (TLAST seen) and FIFO is completely empty:
                        else if (frame_last_seen && (fifo_count == 0)) begin
                            state <= S_DONE;                // Entire photo frame stored in RAM!
                        end
                    end

                    // ------------------------------------------------------------
                    // STATE 3: S_BURST_WR - Wait for AXI Master to complete write burst
                    // ------------------------------------------------------------
                    S_BURST_WR: begin
                        // Wait for AXI Master to signal that DDR3 confirmed write (BRESP = OKAY):
                        if (m_wr_done) begin
                            // Advance DDR3 address pointer (+16 bytes for 4 words):
                            cur_dst_addr <= cur_dst_addr + ((m_wr_len + 1'b1) << 2);

                            // If frame ended and no more pixels, finish! Else collect more!
                            if (frame_last_seen && (fifo_count == 0))
                                state <= S_DONE;
                            else
                                state <= S_COLLECT;         // Collect next chunk of pixels!
                        end
                    end

                    // ------------------------------------------------------------
                    // STATE 4: S_DONE - Video Frame Complete! Notify CPU
                    // ------------------------------------------------------------
                    S_DONE: begin
                        stat_busy <= 1'b0;                  // Clear BUSY
                        stat_done <= 1'b1;                  // Set DONE = 1

                        // Pulse hardware interrupt if enabled:
                        if (ctrl_irq_en) begin
                            irq <= 1'b1;                    // Notify CPU: "New Video Frame Ready in RAM!"
                        end

                        state <= S_IDLE;                    // Return to IDLE, ready for next frame
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end

    endmodule