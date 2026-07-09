// ========================================================================
// Module Name: i2c_byte_cntr (I2C Byte-Level Controller)
// Description:
//   This module handles parallel-to-serial serialization (for Writes) and
//   serial-to-parallel deserialization (for Reads) of 8-bit bytes.
//   It coordinates byte-level protocol steps by sending bit-level commands
//   to the Bit Controller.
// ========================================================================

module i2c_byte_cntr (
    input  wire        clk,        // System Clock (PCLK)
    input  wire        rst_n,      // Active-low asynchronous reset
    input  wire [7:0]  din,        // Data byte to transmit (loaded from TXR register)
    input  wire        ack_in,     // ACK value to send when reading from slave (0=ACK, 1=NACK)
    input  wire        start,      // Start Command flag (cr[7] & ctr[7])
    input  wire        write,      // Write Command flag (cr[4] & ctr[7])
    input  wire        read,       // Read Command flag (cr[5] & ctr[7])
    input  wire        stop,       // Stop Command flag (cr[6] & ctr[7])
    output reg  [7:0]  dout,       // Data byte received from I2C bus (sent to RXR register)
    output reg         cmd_ack,    // Byte transfer complete pulse (clears cr command bits)
    output reg         ack_out,    // Latch for the 9th ACK bit from Slave (sent to sr[7])
    output reg         i2c_busy,   // Bus Busy flag (sent to sr[6]/sr[1] status bits)
    output reg  [3:0]  bit_cmd,    // Command sent to Bit Controller (START/STOP/Read Bit/Write Bit)
    output reg         bit_din,    // Individual data bit sent to Bit Controller
    input  wire        bit_ack,    // Single-bit transfer complete signal from Bit Controller
    input  wire        bit_dout    // Bit received from the Bit Controller during reads
);

    // ------------------------------------------------------------------------
    // FSM States for Byte-Level Operations
    // ------------------------------------------------------------------------
    localparam state_idle  = 3'd0; // Idle, waiting for CPU commands
    localparam state_start = 3'd1; // Triggering I2C START condition
    localparam state_data  = 3'd2; // Serializing/Deserializing 8 bits of data
    localparam state_ack   = 3'd3; // Processing the 9th ACK/NACK bit
    localparam state_stop  = 3'd4; // Triggering I2C STOP condition
    localparam state_done  = 3'd5; // Final pulse to clean up cr register

    reg [2:0] state;               // FSM current state register
    reg [2:0] bit_counter;         // Counts bits from 7 down to 0
    reg [7:0] shift_reg;           // Shift register to hold serialized/deserialized data

    // ------------------------------------------------------------------------
    // Byte-Level State Machine
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            // Reset state registers and outputs
            state       <= state_idle;
            dout        <= 8'h00;
            bit_cmd     <= 4'b0000;
            bit_din     <= 1'b0;
            ack_out     <= 1'b0;
            cmd_ack     <= 1'b0;
            i2c_busy    <= 1'b0;
            bit_counter <= 3'd0;
            shift_reg   <= 8'h00;
        end else begin
            cmd_ack <= 1'b0;       // Default: Acknowledge pulse is Low

            case(state)
                // -------------------------------------------------------------
                // STATE_IDLE: Wait for CPU to issue command (START, WRITE, READ, STOP)
                // We check !cmd_ack to prevent re-triggering on same acknowledge cycle!
                // -------------------------------------------------------------
                state_idle : begin 
                    if((start || write || read || stop) && !cmd_ack) begin
                        i2c_busy  <= 1'b1;         // Lock the bus
                        shift_reg <= din;          // Load data to shift register
                        
                        if (start) begin
                            state   <= state_start;
                            bit_cmd <= 4'b0001;    // Command Bit Ctrl to do START
                        end else begin
                            state       <= state_data; // Go straight to wiggling data
                            bit_counter <= 3'd7;       // Set bit count to 8 bits
                            if(write) begin
                                bit_cmd <= 4'b0100;    // Command Bit Ctrl to WRITE
                                bit_din <= din[7];     // Drive MSB (Bit 7) first
                            end else begin
                                bit_cmd <= 4'b1000;    // Command Bit Ctrl to READ
                            end 
                        end
                    end else begin
                        // Release bus and clear command outputs when idle
                        i2c_busy <= 1'b0;
                        bit_cmd  <= 4'b0000;
                    end
                end

                // -------------------------------------------------------------
                // STATE_START: Wait for Bit Controller to complete START wiggles
                // -------------------------------------------------------------
                state_start : begin
                    if(bit_ack) begin
                        state       <= state_data; // Proceed to serializing data
                        bit_counter <= 3'd7;
                        if(write) begin
                            bit_din <= shift_reg[7]; // Place MSB
                            bit_cmd <= 4'b0100;      // WRITE bit
                        end else begin 
                            bit_cmd <= 4'b1000;      // READ bit
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE_DATA: Shift 8 bits onto the bus one bit at a time
                // -------------------------------------------------------------
                state_data : begin
                    if(bit_ack) begin
                        if(read) begin
                            // During read, shift in the bit from Bit Controller
                            shift_reg <= (shift_reg << 1) | bit_dout;
                        end else begin
                            // During write, shift the register left
                            shift_reg <= shift_reg << 1;
                        end
                        
                        if (bit_counter == 3'd0) begin
                            // All 8 bits completed! Transition to ACK phase
                            state <= state_ack;
                            if(write) begin
                                bit_cmd <= 4'b1000; // WRITE completed, we must READ ACK from slave
                            end else begin
                                bit_cmd <= 4'b0100; // READ completed, we must WRITE ACK to slave
                                bit_din <= ack_in;  // Drive specified ACK/NACK value
                            end
                        end else begin
                            // Keep wiggling the remaining bits
                            bit_counter <= bit_counter - 3'd1;
                            if(write) begin
                                bit_cmd <= 4'b0100;
                                bit_din <= shift_reg[6]; // Grab next bit (using non-blocking shift value)
                            end else begin
                                bit_cmd <= 4'b1000;
                            end
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE_ACK: Manage 9th ACK bit and transition to STOP or DONE
                // -------------------------------------------------------------
                state_ack : begin
                    if(bit_ack) begin
                        if(write) begin
                            ack_out <= bit_dout; // Latch ACK bit returned from slave
                        end
                        if(read) begin
                            dout <= shift_reg;   // Latch complete byte for CPU read
                        end
                        
                        if(stop) begin
                            state   <= state_stop;
                            bit_cmd <= 4'b0010;  // Command Bit Ctrl to do STOP
                        end else begin
                            state   <= state_done; // No stop commanded, end transaction
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE_STOP: Wait for Bit Controller to complete STOP wiggles
                // -------------------------------------------------------------
                state_stop : begin
                    if(bit_ack) begin
                        state <= state_done;      // Stop finished, exit
                    end
                end

                // -------------------------------------------------------------
                // STATE_DONE: Pulse acknowledge to clear CPU Command Register
                // -------------------------------------------------------------
                state_done : begin
                    cmd_ack <= 1'b1;              // Clear commands in Top Module
                    state   <= state_idle;        // Return to Idle
                end

                default : state <= state_idle;
            endcase
        end
    end
endmodule
