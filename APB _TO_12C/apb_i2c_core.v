// ========================================================================
// Module Name: apb_i2c_core (Top-Level APB-to-I2C Bridge Wrapper)
// Description:
//   This is the top-level module of the design. It acts as an APB Slave,
//   exposing a set of 5 control/data registers to the CPU.
//   It connects the Clock Generator, Byte Controller, and Bit Controller
//   to drive physical SCL/SDA tristate bidirectional lines.
// ========================================================================

module apb_i2c_core (
    // ------------------------------------------------------------------------
    // APB Bus Interface Ports
    // ------------------------------------------------------------------------
    input  wire        PCLK,      // System clock (drives CPU bus and internal FSMs)
    input  wire        PRESETn,   // Active-low synchronous system reset
    input  wire [31:0] PADDR,     // Address Bus from CPU to select registers
    input  wire        PSEL,      // Select signal indicating bridge is targeted
    input  wire        PENABLE,   // Enable signal indicating second phase of transfer
    input  wire        PWRITE,    // Write control (1 = Write, 0 = Read)
    input  wire [31:0] PWDATA,    // 32-bit Write Data from CPU (bottom 8 bits mapped)
    output reg  [31:0] PRDATA,    // 32-bit Read Data to CPU (registers read-back)
    output wire        PREADY,    // Ready signal. Set to 1'b1 (no wait states needed)

    // ------------------------------------------------------------------------
    // Physical Bidirectional I2C Pins
    // ------------------------------------------------------------------------
    inout  wire        scl,       // Serial Clock line (bidirectional)
    inout  wire        sda        // Serial Data line (bidirectional)
);

    // ========================================================================
    // 1. Register Declarations (OpenCores Spec Page 7)
    // ========================================================================
    reg [7:0] prer_lo; // Address 0x00: Prescale Low-byte (clock divisor)
    reg [7:0] prer_hi; // Address 0x01: Prescale High-byte (clock divisor)
    reg [7:0] ctr;     // Address 0x02: Control Register (Bit 7 is Core Enable)

    // Transmit register (txr) is Write-only. Receive register (rxr) is Read-only.
    reg [7:0] txr;     // Address 0x03 (Write): Transmit register (data to serialize)
    wire [7:0] rxr;    // Address 0x03 (Read): Receive register (data deserialized)

    // Command register (cr) is Write-only. Status register (sr) is Read-only.
    reg [7:0] cr;      // Address 0x04 (Write): Command register (STA, STO, RD, WR)
    wire [7:0] sr;     // Address 0x04 (Read): Status register (Busy, RxACK, TIP)

    // ========================================================================
    // 2. Internal Wires connecting the Sub-modules
    // ========================================================================
    wire        clk_en;        // 4x clock tick from Clock Gen to Bit Controller
    wire [3:0]  bit_cmd;       // Command from Byte Controller to Bit Controller
    wire        bit_din;       // Data bit from Byte Controller to Bit Controller
    wire        bit_dout;      // Data bit from Bit Controller to Byte Controller
    wire        bit_ack;       // Bit-transfer acknowledge from Bit Controller
    
    wire        scl_oen;       // SCL output enable from Bit Controller (0=drive, 1=float)
    wire        sda_oen;       // SDA output enable from Bit Controller (0=drive, 1=float)
    wire        scl_in;        // Physical SCL level sampled back from pin
    wire        sda_in;        // Physical SDA level sampled back from pin

    wire [7:0]  byte_dout;     // Completed byte from Byte Controller to RXR register
    wire        byte_ack;      // Completed byte pulse (resets cr and interrupts CPU)
    wire        byte_ack_out;  // Latch for 9th ACK bit read from slave
    wire        i2c_busy;      // Bus Busy status flag from Byte Controller

    // ========================================================================
    // 3. APB Read/Write Decode Logic
    // ========================================================================
    assign PREADY = 1'b1;                                 // Registers are fast, ready immediately
    wire apb_write = PSEL & PENABLE & PWRITE & PREADY;    // Asserted during Write access phase
    wire apb_read  = PSEL & PENABLE & ~PWRITE & PREADY;   // Asserted during Read access phase

    // ========================================================================
    // 4. APB Register Write Logic (CPU -> Registers)
    // ========================================================================
    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            // Reset values defined by OpenCores Spec
            prer_lo <= 8'hFF;
            prer_hi <= 8'hFF;
            ctr     <= 8'h00;
            txr     <= 8'h00;
            cr      <= 8'h00;
        end else begin
            if (apb_write) begin
                // Decode CPU write address
                case (PADDR[2:0])
                    3'h0: prer_lo <= PWDATA[7:0];
                    3'h1: prer_hi <= PWDATA[7:0];
                    3'h2: ctr     <= PWDATA[7:0];
                    3'h3: txr     <= PWDATA[7:0];
                    3'h4: cr      <= PWDATA[7:0]; // Trigger command (STA, STO, RD, WR)
                    default: ;
                endcase
            end else begin
                // AUTO-CLEAR LOGIC:
                // When Byte Controller completes the transfer, it pulses `byte_ack`.
                // We must instantly clear the commands (STA, STO, RD, WR) to `0`
                // to prevent the FSM from immediately starting again.
                if (byte_ack) begin
                    cr[7:4] <= 4'b0000;
                end
            end
        end
    end

    // ========================================================================
    // 5. APB Register Read Logic (Registers -> CPU)
    // ========================================================================
    always @(*) begin
        PRDATA = 32'h0000_0000; // Default data bus value
        if (apb_read) begin
            // Decode CPU read address and put value on APB data bus
            case (PADDR[2:0])
                3'h0: PRDATA[7:0] = prer_lo;
                3'h1: PRDATA[7:0] = prer_hi;
                3'h2: PRDATA[7:0] = ctr;
                3'h3: PRDATA[7:0] = rxr; // Read received serial data
                3'h4: PRDATA[7:0] = sr;  // Read bridge status flags
                default: PRDATA[7:0] = 8'h00;
            endcase
        end
    end

    // ========================================================================
    // 6. Connect Status (SR) and Receive (RXR) Wires (Spec Page 9)
    // ========================================================================
    assign rxr = byte_dout; // Link Receive register directly to Byte Controller output

    // Status bits mapping:
    //   sr[7] = RxACK (0 = slave ACK, 1 = slave NACK)
    //   sr[6] = Busy (1 = bus active, 0 = idle)
    //   sr[5:2] = Reserved (0000)
    //   sr[1] = TIP (Transfer in Progress: 1 when sending/receiving)
    //   sr[0] = IF (Interrupt Flag: asserted for 1 cycle when transfer completes)
    assign sr[7]   = byte_ack_out; 
    assign sr[6]   = i2c_busy;     
    assign sr[5:2] = 4'b0000;      
    assign sr[1]   = i2c_busy;     
    assign sr[0]   = byte_ack;     

    // ========================================================================
    // 7. Physical Bidirectional Buffers (Tristate Logic)
    //   Uses Open-Drain structure to prevent short circuits on the I2C bus.
    // ========================================================================
    // If output enable is Low (0), pull the pin to ground (1'b0).
    // If output enable is High (1), release to high-impedance (1'bz) and let pull-ups pull it High.
    assign scl = (scl_oen == 1'b0) ? 1'b0 : 1'bz;
    assign sda = (sda_oen == 1'b0) ? 1'b0 : 1'bz;

    // Sample physical voltages back into our controller feedback wires
    assign scl_in = scl;
    assign sda_in = sda;

    // ========================================================================
    // 8. Module Instantiations
    // ========================================================================

    // A. Clock Generator (handles clock division and stretching)
    i2c_clk_gen u_clock_gen (
        .clk      (PCLK),
        .rst_n    (PRESETn),
        .prescale ({prer_hi, prer_lo}), // Concatenate LO and HI into 16-bit factor
        .scl_out  (scl_oen),
        .scl_in   (scl_in),
        .clk_en   (clk_en)
    );

    // B. Bit Controller (executes individual bits over SCL/SDA lines)
    i2c_bit_cntr u_bit_ctrl (
        .clk      (PCLK),
        .rst_n    (PRESETn),
        .clk_en   (clk_en),
        .cmd      (bit_cmd),
        .din      (bit_din),
        .dout     (bit_dout),
        .cmd_ack  (bit_ack),
        .scl_oen  (scl_oen),
        .sda_oen  (sda_oen),
        .scl_in   (scl_in),
        .sda_in   (sda_in)
    );

    // C. Byte Controller (serializes/deserializes bytes, governs transfer FSM)
    i2c_byte_cntr u_byte_ctrl (
        .clk      (PCLK),
        .rst_n    (PRESETn),
        .start    (cr[7] & ctr[7]), // Core must be enabled (ctr[7]=1) to start
        .stop     (cr[6] & ctr[7]),
        .read     (cr[5] & ctr[7]),
        .write    (cr[4] & ctr[7]),
        .ack_in   (cr[3]),
        .din      (txr),
        .dout     (byte_dout),
        .cmd_ack  (byte_ack),
        .ack_out  (byte_ack_out),
        .i2c_busy (i2c_busy),
        .bit_cmd  (bit_cmd),
        .bit_din  (bit_din),
        .bit_dout (bit_dout),
        .bit_ack  (bit_ack)
    );

endmodule