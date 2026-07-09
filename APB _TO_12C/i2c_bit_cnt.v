// ========================================================================
// Module Name: i2c_bit_cntr (I2C Bit-Level Controller)
// Description:
//   This module handles individual bit operations (START, STOP, Write Bit, Read Bit)
//   on the physical I2C pins using a 4-phase State Machine (A, B, C, D)
//   synchronized by clk_en.
// ========================================================================

module i2c_bit_cntr (
    input  wire        clk,        // System clock (PCLK)
    input  wire        rst_n,      // Active-low asynchronous reset
    input  wire        clk_en,     // Timing tick from the Clock Generator
    input  wire [3:0]  cmd,        // Command bus from the Byte Controller
    input  wire        din,        // Data bit to send (from Byte Controller shift reg)
    output reg         dout,       // Received data bit to send back to Byte Controller
    output reg         cmd_ack,    // Command Acknowledge pulse (1 cycle) to Byte Controller
    output reg         scl_oen,    // SCL Output Enable Negated (0 = drive Low, 1 = release/float)
    output reg         sda_oen,    // SDA Output Enable Negated (0 = drive Low, 1 = release/float)
    input  wire        scl_in,     // Feedback sample of SCL physical wire
    input  wire        sda_in      // Feedback sample of SDA physical wire
);

    // ------------------------------------------------------------------------
    // FSM States representing the 4 clock phases of an I2C bit
    // ------------------------------------------------------------------------
    localparam state_idle = 3'd0;  // Idle state, waiting for commands
    localparam state_A    = 3'd1;  // Phase A: Setup phase (SCL = Low, SDA wiggles)
    localparam state_B    = 3'd2;  // Phase B: Delay phase (SCL = Low)
    localparam state_c    = 3'd3;  // Phase C: Sample phase (SCL = High, sample SDA)
    localparam state_d    = 3'd4;  // Phase D: Hold phase (SCL = High, transition back to Low)

    reg [2:0] state;               // FSM current state register
    reg [3:0] active_cmd;          // Captures cmd to prevent CPU changes from corrupting FSM mid-bit

    // ------------------------------------------------------------------------
    // 4-Phase Bit-Level State Machine Block
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            // Reset clears states and outputs (lines released to 1'bz default)
            state      <= state_idle;
            active_cmd <= 4'b0000;
            cmd_ack    <= 1'b0;
            scl_oen    <= 1'b1;    // Release SCL
            sda_oen    <= 1'b1;    // Release SDA
        end else begin
            cmd_ack <= 1'b0;       // Default: Acknowledge pulse is Low

            case (state)
                // -------------------------------------------------------------
                // STATE_IDLE: Wait for the Byte Controller to send a command
                // Check !cmd_ack to prevent starting again on the same ack cycle!
                // -------------------------------------------------------------
                state_idle : begin
                    if(cmd != 4'b0000 && !cmd_ack) begin
                        active_cmd <= cmd;       // Capture the command
                        state      <= state_A;   // Jump to Phase A
                    end
                end

                // -------------------------------------------------------------
                // STATE_A (Phase A - Data Setup): 
                //   Pull SCL Low, drive SDA with appropriate value based on command.
                // -------------------------------------------------------------
                state_A : begin
                    if(clk_en) begin
                        state   <= state_B;      // Proceed to Phase B
                        scl_oen <= 1'b0;         // SCL goes Low

                        case(active_cmd)
                            4'b0001 : sda_oen <= 1'b1; // START: Keep SDA High (released)
                            4'b0010 : sda_oen <= 1'b0; // STOP: Pull SDA Low
                            4'b0100 : sda_oen <= din;  // WRITE: Put data bit on SDA
                            4'b1000 : sda_oen <= 1'b1; // READ: Release SDA so slave can drive it
                            default : ;
                        endcase
                    end
                end

                // -------------------------------------------------------------
                // STATE_B (Phase B - Clock Delay):
                //   Keep SCL Low. Complete START/STOP setup times.
                // -------------------------------------------------------------
                state_B : begin
                    if(clk_en) begin
                        state   <= state_c;      // Proceed to Phase C (Sample)
                        scl_oen <= 1'b1;         // Release SCL to go High

                        case(active_cmd)
                            4'b0001 : sda_oen <= 1'b0; // START: Pull SDA Low while SCL is High
                            4'b0010 : sda_oen <= 1'b0; // STOP: Keep SDA Low
                            default : ;
                        endcase
                    end
                end

                // -------------------------------------------------------------
                // STATE_C (Phase C - Clock High / Sample):
                //   Keep SCL High. Sample SDA on SCL rising edge.
                // -------------------------------------------------------------
                state_c : begin
                    if(clk_en) begin
                        state   <= state_d;      // Proceed to Phase D (Hold)
                        scl_oen <= 1'b1;         // SCL remains High

                        case(active_cmd)
                            4'b0010 : sda_oen <= 1'b1; // STOP: Release SDA to float High while SCL is High
                            4'b1000 : dout    <= sda_in; // READ: Sample the state of the SDA physical wire
                            default : ;
                        endcase
                    end
                end

                // -------------------------------------------------------------
                // STATE_D (Phase D - Data Hold):
                //   SCL remains High. Terminate the bit on clk_en.
                // -------------------------------------------------------------
                state_d : begin
                    if(clk_en) begin
                        state   <= state_idle;   // Return to Idle
                        cmd_ack <= 1'b1;         // Assert Acknowledge pulse (finished 1 bit)
                        scl_oen <= 1'b0;         // SCL pulled Low to terminate the bit
                    end
                end
                
                default : state <= state_idle;
            endcase
        end
    end
endmodule
