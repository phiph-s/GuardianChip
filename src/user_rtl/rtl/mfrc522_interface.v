// MFRC522 RFID Reader Interface Module
// Supports register read/write and FIFO operations
// SPI Mode 0 (CPOL=0, CPHA=0)

module mfrc522_interface #(
    parameter CLKS_PER_HALF_BIT = 2,  // SPI clock divider
    parameter MAX_BYTES_PER_CS = 2,   // Max bytes per transaction
    parameter CS_INACTIVE_CLKS = 10   // CS inactive clocks
)(
    input wire clk,
    input wire rst_n,
    
    // Command interface
    input wire cmd_valid,           // Command valid
    output reg cmd_ready,           // Ready for command
    input wire cmd_is_write,        // 1=write, 0=read
    input wire [5:0] cmd_addr,      // Register address (6 bits)
    input wire [7:0] cmd_wdata,     // Write data
    output reg [7:0] cmd_rdata,     // Read data
    output reg cmd_done,            // Command complete
    
    // SPI interface
    output wire spi_cs_n,           // Chip select (active low)
    output wire spi_sclk,           // SPI clock
    output wire spi_mosi,           // Master out slave in
    input wire spi_miso             // Master in slave out
);

    // MFRC522 SPI Protocol:
    // Byte 0: Address byte
    //   Bit 7: 1=read, 0=write
    //   Bit 6-1: Address (6 bits)
    //   Bit 0: Always 0
    // Byte 1: Data byte (write) or dummy+data (read)
    
    // State machine
    localparam ST_IDLE        = 2'b00;
    localparam ST_START_XFER  = 2'b01;
    localparam ST_SEND_DATA   = 2'b10;
    localparam ST_DONE        = 2'b11;
    
    reg [1:0] state;
    reg current_is_write;
    reg [5:0] current_addr;
    reg [7:0] current_wdata;
    reg bytes_sent;
    
    // SPI Master signals
    reg [1:0] spi_tx_count;
    reg [7:0] spi_tx_byte;
    reg spi_tx_dv;
    wire spi_tx_ready;
    wire [7:0] spi_rx_byte;
    wire [1:0] spi_rx_count;
    wire spi_rx_dv;
    
    // SPI Master instance from ip/spi-master
    SPI_Master_With_Single_CS #(
        .SPI_MODE(0),
        .CLKS_PER_HALF_BIT(CLKS_PER_HALF_BIT),
        .MAX_BYTES_PER_CS(MAX_BYTES_PER_CS),
        .CS_INACTIVE_CLKS(CS_INACTIVE_CLKS)
    ) spi_inst (
        .i_Rst_L(rst_n),
        .i_Clk(clk),
        .i_TX_Count(spi_tx_count),
        .i_TX_Byte(spi_tx_byte),
        .i_TX_DV(spi_tx_dv),
        .o_TX_Ready(spi_tx_ready),
        .o_RX_Count(spi_rx_count),
        .o_RX_DV(spi_rx_dv),
        .o_RX_Byte(spi_rx_byte),
        .o_SPI_Clk(spi_sclk),
        .i_SPI_MISO(spi_miso),
        .o_SPI_MOSI(spi_mosi),
        .o_SPI_CS_n(spi_cs_n)
    );
    
    // Main state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            cmd_ready <= 1;
            cmd_done <= 0;
            cmd_rdata <= 0;
            spi_tx_dv <= 0;
            spi_tx_byte <= 0;
            spi_tx_count <= 0;
            current_is_write <= 0;
            current_addr <= 0;
            current_wdata <= 0;
            bytes_sent <= 0;
        end else begin
            // Default: clear pulses
            cmd_done <= 0;
            spi_tx_dv <= 0;
            
            case (state)
                ST_IDLE: begin
                    cmd_ready <= 1;
                    
                    if (cmd_valid && cmd_ready) begin
                        cmd_ready <= 0;
                        current_is_write <= cmd_is_write;
                        current_addr <= cmd_addr;
                        current_wdata <= cmd_wdata;
                        bytes_sent <= 0;
                        state <= ST_START_XFER;
                    end
                end
                
                ST_START_XFER: begin
                    // Start 2-byte transaction
                    spi_tx_count <= 2'd2;
                    
                    // Build and send address byte: [R/W][A5:A0][0]
                    // Note: MFRC522 expects 1 for Read, 0 for Write
                    spi_tx_byte <= {~current_is_write, current_addr, 1'b0};
                    spi_tx_dv <= 1;
                    bytes_sent <= 0;
                    state <= ST_SEND_DATA;
                end
                
                ST_SEND_DATA: begin
                    // Wait for SPI master to be ready after first byte
                    if (spi_tx_ready && !spi_tx_dv) begin
                        if (bytes_sent == 0) begin
                            // Send second byte
                            bytes_sent <= 1;
                            if (current_is_write) begin
                                spi_tx_byte <= current_wdata;
                            end else begin
                                spi_tx_byte <= 8'h00;  // Dummy for read
                            end
                            spi_tx_dv <= 1;
                        end else begin
                            // All bytes sent, wait for completion
                            state <= ST_DONE;
                        end
                    end
                    
                    // Capture read data when received (RX count is 1 when 2nd byte arrives)
                    // Even during write, SPI is full-duplex so we receive data
                    if (spi_rx_dv && spi_rx_count == 2'd1) begin
                        cmd_rdata <= spi_rx_byte;
                    end
                end
                
                ST_DONE: begin
                    cmd_done <= 1;
                    state <= ST_IDLE;
                end
                
                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
