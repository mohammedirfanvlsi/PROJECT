`timescale 1ns / 1ps

    // ============================================================================
    // MODULE: AXI4_FULL_MASTER
    // DESCRIPTION: Fully synthesizable AXI4-Full Master module.
    // FEATURES:
    //   - Zero-latency combinational data feeding (Bypasses register bubble)
    //   - Automatic INCR burst address generator
    //   - Strict AXI4 handshake protocol compliance (VALID/READY handshake laws)
    // ============================================================================
    module axi_master #(
        parameter DATA_WIDTH = 32,          // 32-bit data bus (4 bytes per word transfer)
        parameter ADDR_WIDTH = 32,          // 32-bit memory address space (up to 4GB)
        parameter ID_WIDTH   = 4            // 4-bit Transaction ID tags (allows up to 16 outstanding threads)
    )(
        // ========================================================================
        // 1. GLOBAL SYSTEM SIGNALS
        // ========================================================================
        input  wire                   aclk,          // Global Clock (All state transitions happen on rising edge)
        input  wire                   aresetn,       // Global Active-LOW Reset (0 = System Reset, 1 = Normal Run)

        // ========================================================================
        // 2. USER / DMA STREAMING INTERFACE
        // ========================================================================
        // --- Write Control Pins ---
        input  wire                   init_write,    // 1-cycle pulse from user to trigger a new Write transaction
        input  wire [ADDR_WIDTH-1:0]  wr_addr,       // Starting target address in memory to write to
        input  wire [7:0]             wr_len,        // Burst length minus 1 (e.g., 3 means 3 + 1 = 4 beats)
        input  wire [DATA_WIDTH-1:0]  wr_data_in,    // Real-time 32-bit data word coming directly from User/FIFO
        output wire                   wr_data_req,   // Strobe to User/FIFO: "Give me the NEXT word on the next cycle!"
        output reg                    wr_done,       // Pulses HIGH for 1 cycle when the entire write transaction completes

        // --- Read Control Pins ---
        input  wire                   init_read,     // 1-cycle pulse from user to trigger a new Read transaction
        input  wire [ADDR_WIDTH-1:0]  rd_addr,       // Starting target address in memory to read from
        input  wire [7:0]             rd_len,        // Requested read burst length minus 1
        output wire [DATA_WIDTH-1:0]  rd_data_out,   // Real-time 32-bit data word streaming to User/FIFO
        output wire                   rd_data_valid, // Pulses HIGH whenever valid read data is present on rd_data_out
        output reg                    rd_done,       // Pulses HIGH for 1 cycle when entire read burst finishes

        // ========================================================================
        // 3. AXI4 WRITE ADDRESS CHANNEL (AW) - [Master -> Slave]
        // ========================================================================
        output reg  [ID_WIDTH-1:0]    m_axi_awid,    // Write Transaction ID tag (tracks this specific burst)
        output reg  [ADDR_WIDTH-1:0]  m_axi_awaddr,  // Write start address driven to Slave
        output reg  [7:0]             m_axi_awlen,   // Burst length: awlen = number of beats - 1
        output reg  [2:0]             m_axi_awsize,  // 3'b010 = 4 bytes (32 bits) per transfer beat
        output reg  [1:0]             m_axi_awburst, // 2'b01 = INCR mode (Slave auto-increments address)
        output reg                    m_axi_awvalid, // Master: "Write Address is stable and valid on the bus!"
        input  wire                   m_axi_awready, // Slave: "I have captured the Write Address!"

        // ========================================================================
        // 4. AXI4 WRITE DATA CHANNEL (W) - [Master -> Slave]
        // ========================================================================
        output wire [DATA_WIDTH-1:0]  m_axi_wdata,   // Write data bus driven directly from wr_data_in (Zero latency!)
        output reg  [(DATA_WIDTH/8)-1:0] m_axi_wstrb, // Byte enables: 4'b1111 means all 4 bytes of word are valid
        output reg                    m_axi_wlast,   // Master: "This is the FINAL beat of this write burst!"
        output reg                    m_axi_wvalid,  // Master: "Write Data is stable and valid on the bus!"
        input  wire                   m_axi_wready,  // Slave: "I am ready and capturing this data beat!"

        // ========================================================================
        // 5. AXI4 WRITE RESPONSE CHANNEL (B) - [Slave -> Master]
        // ========================================================================
        input  wire [ID_WIDTH-1:0]    m_axi_bid,     // Response ID from Slave (must match m_axi_awid)
        input  wire [1:0]             m_axi_bresp,   // Write status: 2'b00 = OKAY (Write was committed to RAM)
        input  wire                   m_axi_bvalid,  // Slave: "Write is committed, here is your receipt!"
        output reg                    m_axi_bready,  // Master: "I am ready to accept your receipt!"

        // ========================================================================
        // 6. AXI4 READ ADDRESS CHANNEL (AR) - [Master -> Slave]
        // ========================================================================
        output reg  [ID_WIDTH-1:0]    m_axi_arid,    // Read Transaction ID tag
        output reg  [ADDR_WIDTH-1:0]  m_axi_araddr,  // Read starting address driven to Slave
        output reg  [7:0]             m_axi_arlen,   // Number of read beats requested minus 1
        output reg  [2:0]             m_axi_arsize,  // 3'b010 = 4 bytes per read beat
        output reg  [1:0]             m_axi_arburst, // 2'b01 = INCR mode
        output reg                    m_axi_arvalid, // Master: "Read Address is stable and valid!"
        input  wire                   m_axi_arready, // Slave: "I have captured the Read Address!"

        // ========================================================================
        // 7. AXI4 READ DATA CHANNEL (R) - [Slave -> Master]
        // ========================================================================
        input  wire [ID_WIDTH-1:0]    m_axi_rid,     // Read Data ID tag echoed by Slave
        input  wire [DATA_WIDTH-1:0]  m_axi_rdata,   // Read data word payload coming from memory
        input  wire [1:0]             m_axi_rresp,   // Read status: 2'b00 = OKAY
        input  wire                   m_axi_rlast,   // Slave: "This is the final beat of the requested read!"
        input  wire                   m_axi_rvalid,  // Slave: "Read Data is valid on the bus!"
        output reg                    m_axi_rready   // Master: "I am ready to capture this read word!"
    );

        // ========================================================================
        // 8. FSM STATE CONSTANTS
        // ========================================================================
        localparam W_IDLE  = 2'b00; // Waiting for user to pulse init_write
        localparam W_ADDR  = 2'b01; // Driving address on AW channel until Slave asserts AWREADY
        localparam W_DATA  = 2'b10; // Streaming data words on W channel until all beats finish
        localparam W_RESP  = 2'b11; // Waiting for Slave confirmation on B channel (BVALID)

        localparam R_IDLE  = 2'b00; // Waiting for user to pulse init_read
        localparam R_ADDR  = 2'b01; // Driving read address on AR channel until Slave asserts ARREADY
        localparam R_DATA  = 2'b10; // Capturing incoming data words on R channel until RLAST

        reg [1:0] w_state;          // Write State Machine register
        reg [1:0] r_state;          // Read State Machine register

        reg [7:0] w_beat_cnt;       // Counts write beats transferred: 0, 1, 2...
        reg [7:0] r_beat_cnt;       // Counts read beats received: 0, 1, 2...

        // ========================================================================
        // 9. CONTINUOUS ASSIGNMENTS (ZERO-LATENCY STREAMING BYPASS)
        // ========================================================================
        // Why continuous assign?
        // Connecting wr_data_in directly to m_axi_wdata eliminates the 1-cycle flip-flop delay!
        // As soon as User/FIFO updates wr_data_in, m_axi_wdata updates IMMEDIATELY with zero lag!
        assign m_axi_wdata = wr_data_in;

        // Why wr_data_req = wvalid && wready && !wlast?
        // Whenever a beat is accepted by Slave, we immediately tell User/FIFO to load the next word!
        // We stop requesting on the last beat (!m_axi_wlast) because no more words are needed.
        assign wr_data_req = (m_axi_wvalid && m_axi_wready);

        // Pass the incoming memory word directly to the user output port:
        assign rd_data_out   = m_axi_rdata;

        // Flag to user that rd_data_out is valid ONLY when a genuine handshake occurs on R-channel:
        assign rd_data_valid = (m_axi_rvalid && m_axi_rready);


        // ========================================================================
        // SECTION A: WRITE ENGINE FSM (AW, W, B Channels)
        // ========================================================================
        always @(posedge aclk or negedge aresetn) begin
            if (!aresetn) begin
                // Reset state machine and output registers:
                w_state       <= W_IDLE;
                w_beat_cnt    <= 8'd0;
                wr_done       <= 1'b0;

                // Clear AW channel signals:
                m_axi_awid    <= {ID_WIDTH{1'b0}};
                m_axi_awaddr  <= {ADDR_WIDTH{1'b0}};
                m_axi_awlen   <= 8'd0;
                m_axi_awsize  <= 3'b010;  // 4 bytes per beat (32 bits)
                m_axi_awburst <= 2'b01;   // INCR burst type
                m_axi_awvalid <= 1'b0;

                // Clear W channel signals:
                m_axi_wstrb   <= 4'b1111; // Enable all 4 byte lanes
                m_axi_wlast   <= 1'b0;
                m_axi_wvalid  <= 1'b0;

                // Clear B channel signals:
                m_axi_bready  <= 1'b0;
            end else begin
                // wr_done is a single-cycle notification pulse; default to 0:
                wr_done <= 1'b0;

                case (w_state)

                    // ------------------------------------------------------------
                    // STATE 1: W_IDLE - Waiting for user to initiate write
                    // ------------------------------------------------------------
                    W_IDLE: begin
                        w_beat_cnt  <= 8'd0;
                        m_axi_wlast <= 1'b0;

                        if (init_write) begin
                            // Latch user parameters and present address on AW channel:
                            m_axi_awid    <= 4'b0001;     // Transaction ID tag #1
                            m_axi_awaddr  <= wr_addr;     // Target starting address
                            m_axi_awlen   <= wr_len;      // Burst length minus 1
                            m_axi_awsize  <= 3'b010;      // 32-bit transfer
                            m_axi_awburst <= 2'b01;       // INCR burst mode
                            m_axi_awvalid <= 1'b1;        // Assert AWVALID (Address is ready!)

                            w_state       <= W_ADDR;      // Move to Address Handshake state
                        end
                    end

                    // ------------------------------------------------------------
                    // STATE 2: W_ADDR - Complete Address Handshake
                    // ------------------------------------------------------------
                    W_ADDR: begin
                        // When Slave asserts AWREADY, the address handshake is complete!
                        if (m_axi_awvalid && m_axi_awready) begin
                            m_axi_awvalid <= 1'b0;        // Drop AWVALID (Address phase done)
                            m_axi_wvalid  <= 1'b1;        // Immediately raise WVALID for Beat 0!
                            m_axi_wstrb   <= 4'b1111;     // Enable all 4 bytes
                            w_beat_cnt    <= 8'd0;        // Initialize beat counter

                            // If single beat burst (len == 0), flag WLAST immediately:
                            if (m_axi_awlen == 8'd0)
                                m_axi_wlast <= 1'b1;
                            else
                                m_axi_wlast <= 1'b0;

                            w_state <= W_DATA;            // Advance to Data streaming state
                        end
                    end

                    // ------------------------------------------------------------
                    // STATE 3: W_DATA - Stream Data Beats onto AXI Bus
                    // ------------------------------------------------------------
                    W_DATA: begin
                        // Data handshake occurs when BOTH WVALID and WREADY are HIGH:
                        if (m_axi_wvalid && m_axi_wready) begin

                            // Case 1: The beat just accepted was the FINAL beat!
                            if (m_axi_wlast) begin
                                m_axi_wvalid <= 1'b0;     // Drop WVALID (No more data to write)
                                m_axi_wlast  <= 1'b0;     // Clear WLAST
                                m_axi_bready <= 1'b1;     // Assert BREADY (Ready for receipt on B channel)
                                w_state      <= W_RESP;   // Move to Wait Response state
                            end
                            // Case 2: More beats remain in this burst!
                            else begin
                                w_beat_cnt <= w_beat_cnt + 1'b1; // Advance internal counter

                                // When the next beat is going to be the last beat, assert WLAST:
                                if (w_beat_cnt + 1'b1 == m_axi_awlen) begin
                                    m_axi_wlast <= 1'b1;  // Inform Slave: Next beat is the LAST!
                                end
                            end
                        end
                    end

                    // ------------------------------------------------------------
                    // STATE 4: W_RESP - Wait for Write Response Receipt on B-Channel
                    // ------------------------------------------------------------
                    W_RESP: begin
                        // Response handshake occurs when BOTH BVALID and BREADY are HIGH:
                        if (m_axi_bvalid && m_axi_bready) begin
                            m_axi_bready <= 1'b0;         // Handshake complete, drop BREADY
                            wr_done      <= 1'b1;         // Notify user: "Entire write burst finished!"
                            w_state      <= W_IDLE;       // Return to IDLE, ready for next command
                        end
                    end

                    default: w_state <= W_IDLE;
                endcase
            end
        end


        // ========================================================================
        // SECTION B: READ ENGINE FSM (AR and R Channels)
        // ========================================================================
        always @(posedge aclk or negedge aresetn) begin
            if (!aresetn) begin
                // Reset read state machine and outputs:
                r_state       <= R_IDLE;
                r_beat_cnt    <= 8'd0;
                rd_done       <= 1'b0;

                // Clear AR channel:
                m_axi_arid    <= {ID_WIDTH{1'b0}};
                m_axi_araddr  <= {ADDR_WIDTH{1'b0}};
                m_axi_arlen   <= 8'd0;
                m_axi_arsize  <= 3'b010;  // 4 bytes per beat
                m_axi_arburst <= 2'b01;   // INCR burst
                m_axi_arvalid <= 1'b0;

                // Clear R channel:
                m_axi_rready  <= 1'b0;
            end else begin
                // rd_done is a single-cycle notification pulse; default to 0:
                rd_done <= 1'b0;

                case (r_state)

                    // ------------------------------------------------------------
                    // STATE 1: R_IDLE - Waiting for user to initiate read
                    // ------------------------------------------------------------
                    R_IDLE: begin
                        r_beat_cnt <= 8'd0;

                        if (init_read) begin
                            // Latch read parameters and drive AR channel:
                            m_axi_arid    <= 4'b0001;     // Transaction ID tag #1
                            m_axi_araddr  <= rd_addr;     // Target start address
                            m_axi_arlen   <= rd_len;      // Requested burst length
                            m_axi_arsize  <= 3'b010;      // 32-bit transfers
                            m_axi_arburst <= 2'b01;       // INCR burst mode
                            m_axi_arvalid <= 1'b1;        // Assert ARVALID (Read Address is valid!)

                            r_state       <= R_ADDR;      // Move to Address Handshake state
                        end
                    end

                    // ------------------------------------------------------------
                    // STATE 2: R_ADDR - Complete Read Address Handshake
                    // ------------------------------------------------------------
                    R_ADDR: begin
                        // When Slave asserts ARREADY, address handshake is complete:
                        if (m_axi_arvalid && m_axi_arready) begin
                            m_axi_arvalid <= 1'b0;        // Drop ARVALID (Address phase done)
                            m_axi_rready  <= 1'b1;        // Assert RREADY (Master ready to accept read data)
                            r_state       <= R_DATA;      // Move to Data Capture state
                        end
                    end

                    // ------------------------------------------------------------
                    // STATE 3: R_DATA - Stream Read Data Beats from Slave
                    // ------------------------------------------------------------
                    R_DATA: begin
                        // Read handshake occurs when BOTH RVALID and RREADY are HIGH:
                        if (m_axi_rvalid && m_axi_rready) begin

                            // Check if Slave asserted RLAST (Final beat of this read burst):
                            if (m_axi_rlast) begin
                                m_axi_rready <= 1'b0;     // Drop RREADY (Burst complete)
                                rd_done      <= 1'b1;     // Notify user: "Entire read burst finished!"
                                r_state      <= R_IDLE;   // Return to IDLE
                            end else begin
                                r_beat_cnt <= r_beat_cnt + 1'b1; // Advance read beat counter
                            end
                        end
                    end

                    default: r_state <= R_IDLE;
                endcase
            end
        end

    endmodule