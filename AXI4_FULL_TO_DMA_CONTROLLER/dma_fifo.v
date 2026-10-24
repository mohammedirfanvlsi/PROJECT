`timescale 1ns / 1ps
 // ============================================================================
    // MODULE: dma_fifo
    // DESCRIPTION: 32-bit Synchronous FIFO (First-In, First-Out) buffer.
    // FEATURES:
    //   - Depth: 16 words (holds up to 16 32-bit words)
    //   - Single clock domain (aclk) - Zero metastability risk
    //   - First-Word Fall-Through (FWFT): Data is immediately available on output
    //   - 5-bit pointer math using the extra MSB bit for FULL/EMPTY detection
    // ============================================================================
    module dma_fifo #(
        parameter DATA_WIDTH = 32,          // Each word is 32 bits wide (4 bytes)
        parameter FIFO_DEPTH = 16,          // Total storage capacity: 16 words
        parameter ADDR_WIDTH = 4            // 4 bits needed to address 16 rows (2^4 = 16)
    )(
        // ========================================================================
        // 1. GLOBAL SIGNALS
        // ========================================================================
        input  wire                   clk,        // System Clock (aclk = 100 MHz)
        input  wire                   rst_n,      // Active-LOW synchronous reset (0 = reset, 1 = run)

        // ========================================================================
        // 2. WRITE INTERFACE (Connected to AXI Read Engine to receive incoming data)
        // ========================================================================
        input  wire                   wr_en,      // Write Enable: Pulses HIGH when a new word arrives
        input  wire [DATA_WIDTH-1:0]  wr_data,    // Incoming 32-bit data word from AXI Master
        output wire                   full,       // Status Flag: Goes HIGH when all 16 seats are occupied

        // ========================================================================
        // 3. READ INTERFACE (Connected to AXI Write Engine to supply outgoing data)
        // ========================================================================
        input  wire                   rd_en,      // Read Enable: Pulses HIGH when AXI Master takes a word
        output wire [DATA_WIDTH-1:0]  rd_data,    // Outgoing 32-bit data word streaming to AXI Master
        output wire                   empty,      // Status Flag: Goes HIGH when FIFO has 0 words

        // ========================================================================
        // 4. STATUS / OCCUPANCY COUNTER
        // ========================================================================
        output wire [ADDR_WIDTH:0]    fifo_count  // 5-bit counter: Tells DMA FSM exactly how many words are inside (0 to 16)
    );

        // ========================================================================
        // INTERNAL MEMORY STORAGE (16 rows, each holding 32 bits)
        // ========================================================================
        reg [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1]; // The physical 16-row RAM array inside the FIFO

        // ========================================================================
        // 5-BIT POINTER REGISTERS (Using the Extra Bit trick!)
        // ========================================================================
        // Notice ADDR_WIDTH is 4, but pointers are [4:0] (5 bits)!
        // Bits [3:0] = Selects physical memory row 0 to 15
        // Bit  [4]   = Lap counter (0 or 1) to distinguish FULL from EMPTY
        reg [ADDR_WIDTH:0] wr_ptr; // Write pointer register
        reg [ADDR_WIDTH:0] rd_ptr; // Read pointer register


        // ========================================================================
        // SECTION A: WRITE LOGIC (Storing data into FIFO)
        // ========================================================================
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                // On reset, clear write pointer back to seat 0:
                wr_ptr <= {(ADDR_WIDTH+1){1'b0}};
            end else begin
                // Only write if write enable is HIGH and FIFO is NOT completely full:
                if (wr_en && !full) begin
                    // Store the 32-bit word into the memory seat selected by lower 4 bits:
                    mem[wr_ptr[ADDR_WIDTH-1:0]] <= wr_data;

                    // Advance write pointer to the next seat:
                    wr_ptr <= wr_ptr + 1'b1;
                end
            end
        end


        // ========================================================================
        // SECTION B: READ LOGIC (Advancing the read pointer)
        // ========================================================================
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                // On reset, clear read pointer back to seat 0:
                rd_ptr <= {(ADDR_WIDTH+1){1'b0}};
            end else begin
                // Only advance read pointer if read enable is HIGH and FIFO is NOT empty:
                if (rd_en && !empty) begin
                    // Move read pointer forward to next word:
                    rd_ptr <= rd_ptr + 1'b1;
                end
            end
        end


        // ========================================================================
        // SECTION C: FIRST-WORD FALL-THROUGH (FWFT) DATA OUTPUT
        // ========================================================================
        // Why continuous assign?
        // In First-Word Fall-Through, the word at the current rd_ptr is ALWAYS
        // visible on rd_data wires immediately! No waiting for a clock cycle!
        // When the AXI Master is ready, the data is already waiting on the wire!
        assign rd_data = mem[rd_ptr[ADDR_WIDTH-1:0]];


        // ========================================================================
        // SECTION D: FULL & EMPTY FLAG LOGIC (The MSB Bit Math)
        // ========================================================================
        // EMPTY: Both pointers point to the exact same seat on the exact same lap!
        assign empty = (wr_ptr == rd_ptr);

        // FULL: Lower 4 bits match (same seat), but MSB bit 4 is DIFFERENT (1 lap ahead)!
        assign full  = (wr_ptr[ADDR_WIDTH-1:0] == rd_ptr[ADDR_WIDTH-1:0]) &&
                       (wr_ptr[ADDR_WIDTH]     != rd_ptr[ADDR_WIDTH]);


        // ========================================================================
        // SECTION E: FIFO OCCUPANCY COUNT
        // ========================================================================
        // Subtracting rd_ptr from wr_ptr gives the exact number of words inside!
        // Examples:
        // - Empty: wr_ptr(0) - rd_ptr(0) = 0 words
        // - 4 words stored: wr_ptr(4) - rd_ptr(0) = 4 words
        // - Full: wr_ptr(16 = 5'b10000) - rd_ptr(0 = 5'b00000) = 16 words!
        assign fifo_count = wr_ptr - rd_ptr;

    endmodule