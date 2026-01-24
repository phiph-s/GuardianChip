// MFRC522 SPI Read with explicit clock cycles
module ulx3s_test_top (
    input wire clk_25mhz, btn_fire1,
    output reg spi_sclk, spi_cs_0, spi_cs_1, spi_mosi,
    input wire spi_miso,
    output wire uart_txd, input wire uart_rxd,
    output wire mode, busy, hard_fault, unlock,
    output wire [7:0] led
);
    reg [24:0] timer;
    reg [5:0] spi_bit_count;  // 0-15 for 16 bits
    reg [8:0] spi_clk_count;  // Count for 500kHz SPI clock
    reg [15:0] tx_shift;
    reg [15:0] rx_shift;
    reg [15:0] rx_data;  // Latched data after transfer complete
    reg [2:0] state;  // 0=idle, 1=delay, 2=transfer, 3=done
    reg [1:0] test_cycle;  // 0-3 for different register reads
    
    localparam IDLE = 0, DELAY = 1, TRANSFER = 2, DONE = 3;
    
    always @(posedge clk_25mhz) begin
        timer <= timer + 1;
        
        case (state)
            IDLE: begin
                spi_cs_0 <= 1;
                spi_sclk <= 0;
                spi_mosi <= 0;
                
                if (timer == 25'd10_000_000) begin
                    state <= DELAY;
                    spi_cs_0 <= 0;
                    // Cycle through different registers
                    case (test_cycle)
                        0: tx_shift <= 16'hEE00;  // Read Version (0x37)
                        1: tx_shift <= 16'hE600;  // Read ComIEnReg (0x33)
                        2: tx_shift <= 16'hFC00;  // Read TxControlReg (0x3E)
                        3: tx_shift <= 16'hEE00;  // Read Version again
                    endcase
                    spi_clk_count <= 0;
                    test_cycle <= test_cycle + 1;
                end
            end
            
            DELAY: begin
                // Wait 4us after CS activation
                if (spi_clk_count < 100) begin
                    spi_clk_count <= spi_clk_count + 1;
                end else begin
                    state <= TRANSFER;
                    spi_clk_count <= 0;
                    spi_bit_count <= 0;
                    spi_sclk <= 0;
                end
            end
            
            TRANSFER: begin
                spi_clk_count <= spi_clk_count + 1;
                
                if (spi_clk_count == 12) begin
                    // Half period - rising edge
                    spi_sclk <= 1;
                    // Sample MISO
                    rx_shift <= {rx_shift[14:0], spi_miso};
                    
                end else if (spi_clk_count == 24) begin
                    // Full period - falling edge
                    spi_sclk <= 0;
                    spi_clk_count <= 0;
                    // Output next bit
                    spi_mosi <= tx_shift[15];
                    tx_shift <= {tx_shift[14:0], 1'b0};
                    
                    spi_bit_count <= spi_bit_count + 1;
                    if (spi_bit_count == 15) begin
                        state <= DONE;
                    end
                end
            end
            
            DONE: begin
                spi_cs_0 <= 1;
                spi_sclk <= 0;
                // Latch the received data
                rx_data <= rx_shift;
                // Stay here until next timer trigger
                if (timer == 25'd10_000_000) begin
                    state <= DELAY;
                    spi_cs_0 <= 0;
                    // Test different registers with known values
                    case (test_cycle)
                        0: tx_shift <= 16'h8000;  // Read Register 0x00 (Reserved)
                        1: tx_shift <= 16'hEE00;  // Read Register 0x37 (Version)
                        2: tx_shift <= 16'h9200;  // Read Register 0x09 (CollReg)
                        3: tx_shift <= 16'hAA00;  // Read Register 0x15 (ModeReg)
                    endcase
                    spi_clk_count <= 0;
                    test_cycle <= test_cycle + 1;
                end
            end
        endcase
    end
    
    assign spi_cs_1 = 1'b1;
    
    wire [7:0] byte_high = rx_data[15:8];  // First byte received (latched)
    wire [7:0] byte_low = rx_data[7:0];    // Second byte received (latched)
    
    // Show BOTH bytes on LEDs - press button to switch
    wire [7:0] display_byte = btn_fire1 ? byte_high : byte_low;
    
    assign led = display_byte;
    
    assign mode = (state == TRANSFER);
    assign busy = (state != IDLE);
    assign hard_fault = 0;
    assign unlock = btn_fire1;  // Show which byte we're displaying
    assign uart_txd = 1'b1;
endmodule
