// ========================================================================
// Module Name: i2c_clk_gen (I2C Clock Generator)
// Description:
//   This module divides the high-speed system clock (PCLK) down to a slower
//   timing tick (clk_en) that dictates the speed of the I2C transfer.
//   It also handles Clock Stretching by monitoring SCL pin feedback.
// ========================================================================

module i2c_clk_gen ( 
    input  wire        clk,        // System Clock input (e.g. 50MHz APB PCLK)
    input  wire        rst_n,      // Active-low asynchronous system reset
    output reg         clk_en,     // Output tick pulse (asserted for 1 cycle when timer reaches 0)
    input  wire [15:0] prescale,   // 16-bit clock division factor loaded from PRER register
    input  wire        scl_out,    // SCL command output driven by the master Bit Controller
    input  wire        scl_in      // Actual voltage level sampled back from SCL pin
);

    reg [15:0] counter;            // Down-counter register to keep track of clock ticks

    // ------------------------------------------------------------------------
    // Clock Stretching Detection Logic:
    //   If the master releases SCL to let it float High (scl_out == 1'b1),
    //   but the SCL line reads back as Low (scl_in == 1'b0), it means the external
    //   I2C Slave is holding the SCL line Low (stretching the clock).
    // ------------------------------------------------------------------------
    wire clk_stretch_active = (scl_out == 1'b1) && (scl_in == 1'b0);

    // ------------------------------------------------------------------------
    // Clock Division & Timing Tick Generation Block
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            // Asynchronous reset clears the down-counter and clock enable pulse
            counter <= 16'b0;
            clk_en <= 1'b0;
        end else begin
            if (clk_stretch_active) begin
                // If a slave is stretching the clock, stall the timer.
                // We keep the counter at its current value and hold clk_en at 0.
                counter <= counter;
                clk_en <= 1'b0;
            end else if(counter == 0) begin
                // When down-counter reaches 0, reload it with the prescale factor
                // and assert clk_en for 1 clock cycle to trigger the Bit Controller FSM.
                counter <= prescale;
                clk_en <= 1'b1;
            end else begin
                // Normal countdown mode. Decrement the counter and keep clk_en Low.
                counter <= counter - 16'd1;
                clk_en <= 1'b0;
            end
        end
    end
endmodule
