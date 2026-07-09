 module apb_i2c_tb;

        // ------------------------------------------------------------------------
        // 1. APB Wires (CPU driving signals)
        // ------------------------------------------------------------------------
        reg         clk;            // System Clock (50MHz)
        reg         rst_n;          // System Reset (Active Low)
        reg  [31:0] paddr;          // Address we want to access
        reg         psel;           // Select chip
        reg         penable;        // Enable phase
        reg         pwrite;         // 1 = Write, 0 = Read
        reg  [31:0] pwdata;         // Data we send to Master
        wire [31:0] prdata;         // Data we read from Master (used for Status)
        wire        pready;         // Master ready signal

        // ------------------------------------------------------------------------
        // 2. Physical I2C Wires
        // ------------------------------------------------------------------------
        wire        scl;
        wire        sda;

        // Simulate physical pull-up resistors on the I2C wires
        pullup(scl);
        pullup(sda);

        // ------------------------------------------------------------------------
        // 3. Instantiate the APB-to-I2C Master
        // ------------------------------------------------------------------------
        apb_i2c_core u_master (
            .PCLK     (clk),
            .PRESETn  (rst_n),
            .PADDR    (paddr),
            .PSEL     (psel),
            .PENABLE  (penable),
            .PWRITE   (pwrite),
            .PWDATA   (pwdata),
            .PRDATA   (prdata),
            .PREADY   (pready),
            .scl      (scl),
            .sda      (sda)
        );

        // ------------------------------------------------------------------------
        // 4. Instantiate the I2C Slave (Address = 7'h5A)
        // ------------------------------------------------------------------------
        wire [7:0]  slave_rx_data;
        wire        slave_rx_valid;

        i2c_slave #(
            .MY_ADDR  (7'h5A)
        ) u_slave (
            .clk        (clk),
            .rst_n      (rst_n),
            .scl        (scl),
            .sda        (sda),
            .data_out   (slave_rx_data),
            .data_valid (slave_rx_valid),
            .data_in    (8'h00)
        );

        // ------------------------------------------------------------------------
        // 5. Clock Generator (50 MHz = 20ns period)
        // ------------------------------------------------------------------------


        always begin
            #10 clk = ~clk;          // Toggle clock every 10ns
        end

        // ------------------------------------------------------------------------
        // 6. APB Write Helper Task
        // ------------------------------------------------------------------------
        task apb_write(input [31:0] addr, input [7:0] data);
            begin
                @(posedge clk);
                paddr   = addr;
                pwdata  = {24'h000000, data};
                pwrite  = 1'b1;
                psel    = 1'b1;
                penable = 1'b0;      // Setup Phase

                @(posedge clk);
                penable = 1'b1;      // Access Phase

                @(posedge clk);
                psel    = 1'b0;
                penable = 1'b0;
            end
        endtask

        // ------------------------------------------------------------------------
        // 7. APB Read Helper Task (Used to read the Status Register!)
        // ------------------------------------------------------------------------
        task apb_read(input [31:0] addr, output [7:0] data_out);
            begin
                @(posedge clk);
                paddr   = addr;
                pwrite  = 1'b0;      // Set to READ mode
                psel    = 1'b1;
                penable = 1'b0;      // Setup Phase

                @(posedge clk);
                penable = 1'b1;      // Access Phase

                @(posedge clk);
                data_out = prdata[7:0]; // Capture the read data from Master
                psel    = 1'b0;
                penable = 1'b0;
            end
        endtask

        // ------------------------------------------------------------------------
        // 8. Main Test Sequence
        // ------------------------------------------------------------------------
        reg [7:0] status_val;        // Temporary variable to hold read status
        reg       data_received;     // Latch for slave_rx_valid

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                data_received <= 1'b0;
            end else if (slave_rx_valid) begin
                data_received <= 1'b1;
            end
        end

        initial begin
            // A. Reset the system
            clk     = 1'b0;
            rst_n   = 1'b0;          // Press Reset
            psel    = 1'b0;
            penable = 1'b0;
            pwrite  = 1'b0;
            paddr   = 32'h0;
            pwdata  = 32'h0;
            status_val = 8'h00;

            #100;
            rst_n = 1'b1;            // Release Reset
            #40;

            // B. Configure Clock Speed (Prescale = 99 / 8'h63)
            $display("[CPU] Configuring Prescale to 99...");
            apb_write(32'h0, 8'h63); // Write to prer_lo (Address 0)
            apb_write(32'h1, 8'h00); // Write to prer_hi (Address 1)

            // C. Enable the core (Write 8'h80 to Control Register)
            $display("[CPU] Turning ON the core (Control Register)...");
            apb_write(32'h2, 8'h80);

            // ====================================================================
            // TRANSACTION STEP 1: SEND ADDRESS (8'hB4)
            // ====================================================================
            $display("[CPU] Loading Slave Address 7'h5A + Write bit (8'hB4)...");
            apb_write(32'h3, 8'hB4); // Write to Transmit register (txr)

            $display("[CPU] Triggering START + WRITE command...");
            apb_write(32'h4, 8'h90); // Command: STA=1, WR=1 (binary 1001_0000 = 8'h90)

            // D. Poll Status Register until TIP (bit 1) becomes 0.
            // This replaces the lazy #150000 delay!
            status_val = 8'h02;      // Set TIP bit to 1 manually to enter loop
            while (status_val[1] == 1'b1) begin
                #1000;               // Wait 1 microsecond before checking again
                apb_read(32'h4, status_val); // Read Status Register (Address 4)
                $display("[CPU Polling] Address Transfer active... Status register reads: %b (Busy=%b, TIP=%b)", status_val, status_val[6], status_val[1]);
            end
            $display("[CPU] Address sent! Status register reads: %b", status_val);

            // Check if the Slave responded with ACK (Status Bit 7 should be 0)
            if (status_val[7] == 1'b0) begin
                $display("[CPU] Success! Slave acknowledged the Address.");
            end else begin
                $display("[CPU] Error! Slave NACKed the Address. Stopping.");
                $finish;
            end

            // ====================================================================
            // TRANSACTION STEP 2: SEND DATA (8'hAA)
            // ====================================================================
            $display("[CPU] Loading Data byte (8'hAA) into TXR...");
            apb_write(32'h3, 8'hAA); // Write to txr

            $display("[CPU] Triggering WRITE + STOP command...");
            apb_write(32'h4, 8'h50); // Command: WR=1, STO=1 (binary 0101_0000 = 8'h50)

            // E. Poll Status Register again until TIP (bit 1) becomes 0
            status_val = 8'h02;      // Reset TIP bit to 1 to enter loop
            while (status_val[1] == 1'b1) begin
                #1000;
                apb_read(32'h4, status_val); // Read Status Register
                $display("[CPU Polling] Data Transfer active... Status register reads: %b (Busy=%b, TIP=%b)", status_val, status_val[6], status_val[1]);
            end
            $display("[CPU] Data sent! Status register reads: %b", status_val);

            // Check if the Slave acknowledged the data
            if (status_val[7] == 1'b0) begin
                $display("[CPU] Success! Slave acknowledged the Data.");
            end else begin
                $display("[CPU] Error! Slave NACKed the Data.");
            end

            // F. Verification Check
            #1000;
            if (data_received) begin
                $display("[RESULT] SUCCESS! Slave received byte: %h", slave_rx_data);
            end else begin
                $display("[RESULT] FAILED! Slave did not receive data.");
            end

            $finish;
        end

    endmodule
