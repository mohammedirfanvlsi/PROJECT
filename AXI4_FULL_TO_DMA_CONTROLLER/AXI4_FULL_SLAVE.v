`timescale 1ns / 1ps
 // ============================================================================
    // MODULE: AXI4_FULL_SLAVE
    // DESCRIPTION: Fully synthesizable AXI4-Full Slave memory model with 256-word RAM.
    // FEATURES:
    //   - Byte-masking write support using WSTRB [3:0]
    //   - INCR burst address auto-incrementing (+4 bytes / word)
    //   - Word-aligned memory indexing using address bits [9:2]
    //   - Accurate RLAST assertion on exact final beat with zero extra beats
    // ============================================================================
    module axi_slave #(
        parameter DATA_WIDTH = 32,          // 32-bit data bus (4 bytes per word)
        parameter ADDR_WIDTH = 32,          // 32-bit address bus
        parameter ID_WIDTH   = 4,           // 4-bit Transaction ID tags
        parameter MEM_DEPTH  = 256          // Internal RAM size: 256 memory words (1024 bytes)
    )(
        // ========================================================================
        // 1. GLOBAL SYSTEM SIGNALS
        // ========================================================================
        input  wire                   aclk,          // Global Clock (Rising-edge active)
        input  wire                   aresetn,       // Global Reset (Active-LOW: 0=Reset, 1=Run)

        // ========================================================================
        // 2. AXI4 WRITE ADDRESS CHANNEL (AW) - [Master -> Slave]
        // ========================================================================
        input  wire [ID_WIDTH-1:0]    s_axi_awid,    // Write Transaction ID from Master
        input  wire [ADDR_WIDTH-1:0]  s_axi_awaddr,  // Write start address from Master
        input  wire [7:0]             s_axi_awlen,   // Burst length (number of beats minus 1)
        input  wire [2:0]             s_axi_awsize,  // Burst size (3'b010 = 4 bytes per beat)
        input  wire [1:0]             s_axi_awburst, // Burst type (2'b01 = INCR)
        input  wire                   s_axi_awvalid, // Master: "Write Address is valid!"
        output reg                    s_axi_awready, // Slave: "I am ready to capture Write Address!"

        // ========================================================================
        // 3. AXI4 WRITE DATA CHANNEL (W) - [Master -> Slave]
        // ========================================================================
        input  wire [DATA_WIDTH-1:0]  s_axi_wdata,   // Write data payload from Master
        input  wire [(DATA_WIDTH/8)-1:0] s_axi_wstrb, // Byte enables (e.g., 4'b1111)
        input  wire                   s_axi_wlast,   // Master: "This is the last beat of burst!"
        input  wire                   s_axi_wvalid,  // Master: "Write Data is valid!"
        output reg                    s_axi_wready,  // Slave: "I am ready to capture Write Data!"

        // ========================================================================
        // 4. AXI4 WRITE RESPONSE CHANNEL (B) - [Slave -> Master]
        // ========================================================================
        output reg  [ID_WIDTH-1:0]    s_axi_bid,     // Response ID (echoes s_axi_awid back)
        output reg  [1:0]             s_axi_bresp,   // Write status: 2'b00 = OKAY (Success!)
        output reg                    s_axi_bvalid,  // Slave: "Write is committed, here is receipt!"
        input  wire                   s_axi_bready,  // Master: "I acknowledge receipt!"

        // ========================================================================
        // 5. AXI4 READ ADDRESS CHANNEL (AR) - [Master -> Slave]
        // ========================================================================
        input  wire [ID_WIDTH-1:0]    s_axi_arid,    // Read Transaction ID from Master
        input  wire [ADDR_WIDTH-1:0]  s_axi_araddr,  // Read start address from Master
        input  wire [7:0]             s_axi_arlen,   // Requested burst length
        input  wire [2:0]             s_axi_arsize,  // 3'b010 = 4 bytes per beat
        input  wire [1:0]             s_axi_arburst, // 2'b01 = INCR
        input  wire                   s_axi_arvalid, // Master: "Read Address is valid!"
        output reg                    s_axi_arready, // Slave: "I am ready to capture Read Address!"

        // ========================================================================
        // 6. AXI4 READ DATA CHANNEL (R) - [Slave -> Master]
        // ========================================================================
        output reg  [ID_WIDTH-1:0]    s_axi_rid,     // Read Data ID (echoes s_axi_arid back)
        output reg  [DATA_WIDTH-1:0]  s_axi_rdata,   // Read data payload going to Master
        output reg  [1:0]             s_axi_rresp,   // Read status: 2'b00 = OKAY
        output reg                    s_axi_rlast,   // Slave: "This is the final beat of read burst!"
        output reg                    s_axi_rvalid,  // Slave: "Read Data is valid on the bus!"
        input  wire                   s_axi_rready   // Master: "I am ready to capture Read Data!"
    );

        // ========================================================================
        // INTERNAL STORAGE (The 256-word physical RAM array)
        // ========================================================================
        reg [DATA_WIDTH-1:0] mem [0:MEM_DEPTH-1];    // 256 rows, each holding 32 bits (1 word)

        // ========================================================================
        // STATE MACHINE CONSTANTS
        // ========================================================================
        localparam S_W_IDLE = 2'b00; // Waiting for Master to send Write Address (AWVALID)
        localparam S_W_DATA = 2'b01; // Capturing Write Data beats and storing into RAM
        localparam S_W_RESP = 2'b10; // Returning final OKAY receipt on B-channel

        localparam S_R_IDLE = 2'b00; // Waiting for Master to send Read Address (ARVALID)
        localparam S_R_DATA = 2'b01; // Streaming words from RAM onto R-channel

        reg [1:0] s_wstate;          // Slave Write State register
        reg [1:0] s_rstate;          // Slave Read State register

        // Internal Address Registers:
        reg [ADDR_WIDTH-1:0] wr_addr_reg; // Holds current write byte address
        reg [ADDR_WIDTH-1:0] rd_addr_reg; // Holds current read byte address
        reg [7:0]            rd_len_reg;  // Latched burst length for read
        reg [7:0]            rd_beat_cnt; // Counts read beats delivered: 0, 1, 2...


        // ========================================================================
        // SECTION A: SLAVE WRITE ENGINE (Handles AW, W, B channels)
        // ========================================================================
        always @(posedge aclk or negedge aresetn) begin
            if (!aresetn) begin
                // Reset write state and output registers:
                s_wstate       <= S_W_IDLE;
                s_axi_awready  <= 1'b0;
                s_axi_wready   <= 1'b0;
                s_axi_bvalid   <= 1'b0;
                s_axi_bresp    <= 2'b00;
                s_axi_bid      <= {ID_WIDTH{1'b0}};
                wr_addr_reg    <= {ADDR_WIDTH{1'b0}};
            end else begin
                case (s_wstate)

                    // ------------------------------------------------------------
                    // STATE 1: S_W_IDLE - Wait for Write Address from Master
                    // ------------------------------------------------------------
                    S_W_IDLE: begin
                        s_axi_bvalid <= 1'b0; // Ensure response receipt is LOW

                        // When Master asserts AWVALID, capture the address:
                        if (s_axi_awvalid) begin
                            s_axi_awready <= 1'b1;         // Assert AWREADY (Address handshake!)
                            wr_addr_reg   <= s_axi_awaddr; // Latch starting address
                            s_axi_bid     <= s_axi_awid;   // Latch ID tag to return later on B-channel
                            s_axi_wready  <= 1'b1;         // Immediately assert WREADY for Beat 0!
                            s_wstate      <= S_W_DATA;     // Move to Data Capture state
                        end else begin
                            s_axi_awready <= 1'b0;
                        end
                    end

                    // ------------------------------------------------------------
                    // STATE 2: S_W_DATA - Capture Data Beats and Write into RAM
                    // ------------------------------------------------------------
                    S_W_DATA: begin
                        s_axi_awready <= 1'b0; // Drop AWREADY (Address phase complete)

                        // Data handshake occurs when BOTH WVALID and WREADY are HIGH:
                        if (s_axi_wvalid && s_axi_wready) begin

                            // --- BYTE MASKING USING WSTRB ---
                            // Bits [9:2] select the 32-bit RAM row (0 to 255):
                            if (s_axi_wstrb[0]) mem[wr_addr_reg[9:2]][7:0]   <= s_axi_wdata[7:0];   // Byte 0
                            if (s_axi_wstrb[1]) mem[wr_addr_reg[9:2]][15:8]  <= s_axi_wdata[15:8];  // Byte 1
                            if (s_axi_wstrb[2]) mem[wr_addr_reg[9:2]][23:16] <= s_axi_wdata[23:16]; // Byte 2
                            if (s_axi_wstrb[3]) mem[wr_addr_reg[9:2]][31:24] <= s_axi_wdata[31:24]; // Byte 3

                            // INCR Auto-increment address: Add 4 bytes (1 word) for next beat!
                            wr_addr_reg <= wr_addr_reg + 32'd4;

                            // Check if Master signaled that this was the FINAL beat:
                            if (s_axi_wlast) begin
                                s_axi_wready <= 1'b0;      // Burst finished, drop WREADY
                                s_axi_bvalid <= 1'b1;      // Assert BVALID (Receipt ready on B channel!)
                                s_axi_bresp  <= 2'b00;     // 2'b00 = OKAY (Success!)
                                s_wstate     <= S_W_RESP;  // Move to Response state
                            end
                        end
                    end

                    // ------------------------------------------------------------
                    // STATE 3: S_W_RESP - Deliver Response Receipt to Master
                    // ------------------------------------------------------------
                    S_W_RESP: begin
                        // Response handshake occurs when BOTH BVALID and BREADY are HIGH:
                        if (s_axi_bvalid && s_axi_bready) begin
                            s_axi_bvalid <= 1'b0;          // Drop BVALID (Transaction complete!)
                            s_wstate     <= S_W_IDLE;      // Return to IDLE, ready for next write
                        end
                    end

                    default: s_wstate <= S_W_IDLE;
                endcase
            end
        end


        // ========================================================================
        // SECTION B: SLAVE READ ENGINE (Handles AR and R channels)
        // ========================================================================
        always @(posedge aclk or negedge aresetn) begin
            if (!aresetn) begin
                // Reset read state and output registers:
                s_rstate      <= S_R_IDLE;
                s_axi_arready <= 1'b0;
                s_axi_rvalid  <= 1'b0;
                s_axi_rlast   <= 1'b0;
                s_axi_rdata   <= {DATA_WIDTH{1'b0}};
                s_axi_rresp   <= 2'b00;
                s_axi_rid     <= {ID_WIDTH{1'b0}};
                rd_addr_reg   <= {ADDR_WIDTH{1'b0}};
                rd_len_reg    <= 8'd0;
                rd_beat_cnt   <= 8'd0;
            end else begin
                case (s_rstate)

                    // ------------------------------------------------------------
                    // STATE 1: S_R_IDLE - Wait for Read Address from Master
                    // ------------------------------------------------------------
                    S_R_IDLE: begin
                        s_axi_rvalid <= 1'b0;
                        s_axi_rlast  <= 1'b0;
                        rd_beat_cnt  <= 8'd0;

                        // When Master asserts ARVALID, accept the read address:
                        if (s_axi_arvalid) begin
                            s_axi_arready <= 1'b1;         // Assert ARREADY (Address handshake!)
                            rd_addr_reg   <= s_axi_araddr; // Latch starting address
                            rd_len_reg    <= s_axi_arlen;  // Latch burst length
                            s_axi_rid     <= s_axi_arid;   // Echo back Transaction ID tag

                            // PRE-FETCH BEAT 0 IMMEDIATELY:
                            // Fetch Word 0 directly from RAM so it is ready on the very first cycle!
                            s_axi_rdata   <= mem[s_axi_araddr[9:2]];
                            s_axi_rvalid  <= 1'b1;         // Assert RVALID
                            s_axi_rresp   <= 2'b00;        // 2'b00 = OKAY

                            // If burst length is 0 (single-word read), assert RLAST immediately:
                            if (s_axi_arlen == 8'd0)
                                s_axi_rlast <= 1'b1;
                            else
                                s_axi_rlast <= 1'b0;

                            s_rstate      <= S_R_DATA;     // Move to Data Streaming state
                        end else begin
                            s_axi_arready <= 1'b0;
                        end
                    end

                    // ------------------------------------------------------------
                    // STATE 2: S_R_DATA - Stream Read Data Beats from RAM to Master
                    // ------------------------------------------------------------
                    S_R_DATA: begin
                        s_axi_arready <= 1'b0; // Drop ARREADY (Address phase done)

                        // Read handshake occurs when BOTH RVALID and RREADY are HIGH:
                        if (s_axi_rvalid && s_axi_rready) begin

                            // If the beat just accepted was flagged with RLAST, the burst is DONE:
                            if (s_axi_rlast) begin
                                s_axi_rvalid <= 1'b0;      // Drop RVALID (No more data!)
                                s_axi_rlast  <= 1'b0;      // Drop RLAST
                                s_rstate     <= S_R_IDLE;  // Read burst complete! Return to IDLE
                            end
                            // More beats left in the burst:
                            else begin
                                rd_beat_cnt <= rd_beat_cnt + 1'b1; // Advance beat counter
                                rd_addr_reg <= rd_addr_reg + 32'd4; // Auto-increment address (+4 bytes)

                                // PRE-FETCH NEXT WORD:
                                // Fetch word from the next address (rd_addr_reg + 4) for the upcoming cycle!
                                s_axi_rdata <= mem[(rd_addr_reg + 32'd4) >> 2];

                                // Flag RLAST when the next beat is going to be the final beat:
                                if (rd_beat_cnt + 1'b1 == rd_len_reg) begin
                                    s_axi_rlast <= 1'b1;   // Assert RLAST on the final beat!
                                end
                            end
                        end
                    end

                    default: s_rstate <= S_R_IDLE;
                endcase
            end
        end

    endmodule