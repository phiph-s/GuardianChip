// MFRC522 Interface using Simple SPI Master
// 2-byte transactions: address + data
// SPI Mode 0 (CPOL=0, CPHA=0)

module mfrc522_interface_simple #(
    parameter CLKS_PER_HALF_BIT = 2,
    parameter CS_INACTIVE_CLKS = 10
)(
    input wire clk,
    input wire rst_n,
    
    // Command interface
    input wire cmd_valid,
    output reg cmd_ready,
    input wire cmd_is_write,
    input wire [5:0] cmd_addr,
    input wire [7:0] cmd_wdata,
    output reg [7:0] cmd_rdata,
    output reg cmd_done,
    
    // SPI interface
    output wire spi_cs_n,
    output wire spi_sclk,
    output wire spi_mosi,
    input wire spi_miso
);

    // State machine
    localparam ST_IDLE = 2'd0;
    localparam ST_ADDR = 2'd1;
    localparam ST_DATA = 2'd2;
    localparam ST_DONE = 2'd3;
    
    reg [1:0] state;
    reg current_is_write;
    reg [5:0] current_addr;
    reg [7:0] current_wdata;
    
    // SPI signals
    reg spi_cmd_valid;
    wire spi_cmd_ready;
    reg [7:0] spi_tx_data;
    wire [7:0] spi_rx_data;
    wire spi_cmd_done;
    
    // Simple SPI Master
    spi_master #(
        .CLKS_PER_HALF_BIT(CLKS_PER_HALF_BIT),
        .CS_INACTIVE_CLKS(CS_INACTIVE_CLKS)
    ) spi_master (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_valid(spi_cmd_valid),
        .cmd_ready(spi_cmd_ready),
        .tx_data(spi_tx_data),
        .rx_data(spi_rx_data),
        .cmd_done(spi_cmd_done),
        .spi_cs_n(spi_cs_n),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso)
    );
    
    // Main state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            cmd_ready <= 1'b1;
            cmd_done <= 1'b0;
            cmd_rdata <= 8'h00;
            spi_cmd_valid <= 1'b0;
            spi_tx_data <= 8'h00;
            current_is_write <= 1'b0;
            current_addr <= 6'h00;
            current_wdata <= 8'h00;
        end else begin
            // Default: clear pulses
            cmd_done <= 1'b0;
            spi_cmd_valid <= 1'b0;
            
            case (state)
                ST_IDLE: begin
                    cmd_ready <= 1'b1;
                    if (cmd_valid && cmd_ready) begin
                        cmd_ready <= 1'b0;
                        current_is_write <= cmd_is_write;
                        current_addr <= cmd_addr;
                        current_wdata <= cmd_wdata;
                        
                        // Send address byte
                        // MFRC522: Bit 7 = 1 for read, 0 for write
                        spi_tx_data <= {~cmd_is_write, cmd_addr, 1'b0};
                        spi_cmd_valid <= 1'b1;
                        state <= ST_ADDR;
                    end
                end
                
                ST_ADDR: begin
                    if (spi_cmd_done) begin
                        // Send data byte
                        if (current_is_write) begin
                            spi_tx_data <= current_wdata;
                        end else begin
                            spi_tx_data <= 8'h00;  // Dummy for read
                        end
                        spi_cmd_valid <= 1'b1;
                        state <= ST_DATA;
                    end
                end
                
                ST_DATA: begin
                    if (spi_cmd_done) begin
                        // Capture received data
                        cmd_rdata <= spi_rx_data;
                        state <= ST_DONE;
                    end
                end
                
                ST_DONE: begin
                    cmd_done <= 1'b1;
                    state <= ST_IDLE;
                end
                
                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
