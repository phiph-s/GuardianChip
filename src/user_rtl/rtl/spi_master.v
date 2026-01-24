// Simple SPI Master
// Single-byte transactions with automatic CS control
// Supports multiple CS lines
// SPI Mode 0 (CPOL=0, CPHA=0)

module spi_master #(
    parameter CLKS_PER_HALF_BIT = 2,  // SPI clock divider
    parameter CS_INACTIVE_CLKS = 10   // CS inactive time
)(
    input wire clk,
    input wire rst_n,
    
    // Command interface
    input wire cmd_valid,           // Start transaction
    output reg cmd_ready,           // Ready for new transaction
    input wire [7:0] tx_data,       // Data to send
    output reg [7:0] rx_data,       // Data received
    output reg cmd_done,            // Transaction complete (1 cycle pulse)
    
    // SPI interface
    output reg spi_cs_n,            // Chip select (active low)
    output reg spi_sclk,            // SPI clock
    output reg spi_mosi,            // Master out
    input wire spi_miso             // Master in
);

    // State machine
    localparam ST_IDLE       = 3'd0;
    localparam ST_CS_SETUP   = 3'd1;
    localparam ST_TRANSFER   = 3'd2;
    localparam ST_CS_HOLD    = 3'd3;
    
    reg [2:0] state;
    reg [7:0] bit_counter;
    reg [7:0] clk_counter;
    reg [7:0] tx_shift;
    reg [7:0] rx_shift;
    reg [7:0] cs_counter;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            cmd_ready <= 1'b1;
            cmd_done <= 1'b0;
            spi_cs_n <= 1'b1;
            spi_sclk <= 1'b0;
            spi_mosi <= 1'b0;
            rx_data <= 8'h00;
            bit_counter <= 0;
            clk_counter <= 0;
            tx_shift <= 0;
            rx_shift <= 0;
            cs_counter <= 0;
        end else begin
            // Default: clear pulse signals
            cmd_done <= 1'b0;
            
            case (state)
                ST_IDLE: begin
                    cmd_ready <= 1'b1;
                    spi_cs_n <= 1'b1;
                    spi_sclk <= 1'b0;
                    
                    if (cmd_valid && cmd_ready) begin
                        cmd_ready <= 1'b0;
                        tx_shift <= tx_data;
                        bit_counter <= 0;
                        clk_counter <= 0;
                        cs_counter <= 0;
                        state <= ST_CS_SETUP;
                    end
                end
                
                ST_CS_SETUP: begin
                    // Assert CS and wait a bit
                    spi_cs_n <= 1'b0;
                    if (cs_counter < 2) begin
                        cs_counter <= cs_counter + 1;
                    end else begin
                        state <= ST_TRANSFER;
                        clk_counter <= 0;
                    end
                end
                
                ST_TRANSFER: begin
                    // SPI transfer - 8 bits
                    if (clk_counter < CLKS_PER_HALF_BIT - 1) begin
                        clk_counter <= clk_counter + 1;
                    end else begin
                        clk_counter <= 0;
                        spi_sclk <= ~spi_sclk;
                        
                        if (!spi_sclk) begin
                            // Rising edge - output data
                            spi_mosi <= tx_shift[7];
                        end else begin
                            // Falling edge - sample data, shift
                            rx_shift <= {rx_shift[6:0], spi_miso};
                            tx_shift <= {tx_shift[6:0], 1'b0};
                            bit_counter <= bit_counter + 1;
                            
                            if (bit_counter == 7) begin
                                // All bits done
                                rx_data <= {rx_shift[6:0], spi_miso};
                                state <= ST_CS_HOLD;
                                cs_counter <= 0;
                                spi_sclk <= 1'b0;
                            end
                        end
                    end
                end
                
                ST_CS_HOLD: begin
                    // Deassert CS and hold
                    spi_cs_n <= 1'b1;
                    if (cs_counter < CS_INACTIVE_CLKS) begin
                        cs_counter <= cs_counter + 1;
                    end else begin
                        cmd_done <= 1'b1;
                        state <= ST_IDLE;
                    end
                end
                
                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
