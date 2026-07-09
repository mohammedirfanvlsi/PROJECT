module i2c_slave #(
    parameter [6:0] MY_ADDR = 7'h5A // The unique 7-bit address of this slave device
)(
    input  wire       clk,        // Main system clock (50 MHz)
    input  wire       rst_n,      // Active-low reset
    inout  wire       sda,        // Physical Data wire
    inout  wire       scl,        // Physical Clock wire
    output reg  [7:0] data_out,   // Data received by the slave
    output reg        data_valid, // Pulses 1 when slave has a complete, new byte
    input  wire [7:0] data_in     // Data the slave will send if master reads from it
);

    // Synchronizers to safely bring external asynchronous SCL/SDA into our 50MHz clock domain
    // We use a 3-bit shift register.
    reg [2:0] scl_sync;
    reg [2:0] sda_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_sync <= 3'b111; // Default I2C state is HIGH (1) due to pull-ups
            sda_sync <= 3'b111; // Default I2C state is HIGH (1) due to pull-ups
        end else begin
            // Every clock tick, shift left and grab the raw physical pin value at bit 0
            scl_sync <= (scl_sync << 1) | scl;
            sda_sync <= (sda_sync << 1) | sda;
        end
    end

    // Edge Detection using the synchronizer
    // If it was 0 two clocks ago (bit[2]) and is 1 one clock ago (bit[1]), it's a rising edge!
    wire scl_rising  = (scl_sync[2] == 1'b0 && scl_sync[1] == 1'b1);
    // If it was 1 two clocks ago and is 0 one clock ago, it's a falling edge!
    wire scl_falling = (scl_sync[2] == 1'b1 && scl_sync[1] == 1'b0);

    // I2C Start Condition: SDA falls (1 to 0) WHILE SCL is stable HIGH
    wire start_detect = (scl_sync[1] == 1'b1) && (sda_sync[2] == 1'b1 && sda_sync[1] == 1'b0);
    // I2C Stop Condition: SDA rises (0 to 1) WHILE SCL is stable HIGH
    wire stop_detect  = (scl_sync[1] == 1'b1) && (sda_sync[2] == 1'b0 && sda_sync[1] == 1'b1);

    // Open-drain output control for Slave
    reg sda_out;
    assign sda = (sda_out == 1'b0) ? 1'b0 : 1'bz; // Only pull to 0 or let go

    // Slave State Machine States
    localparam IDLE       = 3'd0; // Waiting for Start condition
    localparam GET_ADDR   = 3'd1; // Receiving 8 bits (7-bit addr + 1-bit rw)
    localparam ACK_ADDR   = 3'd2; // Acknowledging if the address matches MY_ADDR
    localparam RECV_DATA  = 3'd3; // Receiving data byte from master
    localparam ACK_DATA   = 3'd4; // Acknowledging received data byte
    localparam SEND_DATA  = 3'd5; // Sending data byte to master
    localparam WAIT_ACK   = 3'd6; // Waiting for master to acknowledge sent data

    reg [2:0] state;     // Current state
    reg [3:0] bit_count; // Bit counter (0 to 8)
    reg [7:0] shift_reg; // Shift register to assemble incoming bits or send outgoing bits
    reg       rw_bit;    // Remembers if the master asked to Read (1) or Write (0)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            sda_out    <= 1'b1; // Let go of SDA
            bit_count  <= 0;
            data_valid <= 0;
        end else begin
            
            data_valid <= 0; // Default: no valid data

            // Independent check: If we detect a START condition at ANY time, reset and get ready!
            // This is important because a master can generate a "Repeated Start" without a Stop.
            if (start_detect) begin
                state     <= GET_ADDR;
                bit_count <= 0;
                sda_out   <= 1'b1; // Let go of SDA so master can talk
            end 
            // Independent check: If we detect a STOP condition at ANY time, go to IDLE
            else if (stop_detect) begin
                state   <= IDLE;
                sda_out <= 1'b1;
            end 
            else begin // Normal state machine operation
                case (state)
                    IDLE: begin
                        sda_out <= 1'b1; // Do nothing, just wait
                    end

                    GET_ADDR: begin
                        if (scl_rising) begin
                            // On SCL rising edge, data is stable. Read SDA and shift it in!
                            shift_reg <= (shift_reg << 1) | sda_sync[1];
                            bit_count <= bit_count + 1; 
                        end
                        else if (scl_falling) begin
                            // On SCL falling edge, check if we finished 8 bits
                            if (bit_count == 8) begin
                                // Top 7 bits are address. Bottom bit is R/W.
                                if (shift_reg[7:1] == MY_ADDR) begin // Is it talking to ME?
                                    sda_out <= 1'b0; // YES! Pull SDA low to ACK!
                                    rw_bit  <= shift_reg[0]; // Save R/W direction
                                    state   <= ACK_ADDR;
                                end else begin
                                    sda_out <= 1'b1; // NO. Not me. Ignore it.
                                    state   <= IDLE;
                                end
                            end
                        end
                    end

                    ACK_ADDR: begin
                        if (scl_falling) begin // After the 9th clock pulse ends
                            bit_count <= 0; // Reset counter for data transfer
                            
                            if (rw_bit == 1'b0) begin // Master wants to WRITE to us
                                sda_out <= 1'b1; // Let go of SDA so Master can send data
                                state   <= RECV_DATA;
                            end else begin // Master wants to READ from us
                                sda_out   <= data_in[7]; // Output our first bit (MSB) immediately
                                shift_reg <= data_in << 1; // Shift so next bit is ready
                                bit_count <= 1; // We just placed 1 bit
                                state     <= SEND_DATA;
                            end
                        end
                    end

                    RECV_DATA: begin
                        if (scl_rising) begin // Data is stable, read it
                            shift_reg <= (shift_reg << 1) | sda_sync[1]; // Make byte
                            bit_count <= bit_count + 1;
                        end
                        else if (scl_falling) begin
                            if (bit_count == 8) begin // We got all 8 bits!
                                sda_out    <= 1'b0;          // Send ACK (pull SDA low)
                                data_out   <= shift_reg;     // Output the complete byte
                                data_valid <= 1'b1;          // Tell system "New Data is ready!"
                                state      <= ACK_DATA;
                            end
                        end
                    end

                    ACK_DATA: begin
                        if (scl_falling) begin // After sending ACK on 9th pulse
                            sda_out   <= 1'b1; // Let go of SDA again
                            bit_count <= 0;
                            state     <= RECV_DATA; // Get ready for the next byte
                        end
                    end

                    SEND_DATA: begin
                        if (scl_falling) begin // Master dropped clock, safe to change data
                            if (bit_count == 8) begin // We sent all 8 bits
                                sda_out <= 1'b1; // Let go of SDA so Master can ACK
                                state   <= WAIT_ACK;
                            end else begin
                                sda_out   <= shift_reg[7]; // Output next bit
                                shift_reg <= shift_reg << 1; // Shift to prepare for next time
                                bit_count <= bit_count + 1;
                            end
                        end
                    end

                    WAIT_ACK: begin
                        if (scl_falling) begin // After Master's ACK/NACK pulse
                            state   <= IDLE; // Back to IDLE
                            sda_out <= 1'b1; 
                        end
                    end
                endcase
            end
        end
    end
endmodule
