// AT25010 EEPROM Interface Module
// 1Kbit (128 x 8) SPI Serial EEPROM
// Supports standard AT25010 command set

module at25010_interface #(
    parameter CLKS_PER_HALF_BIT = 2,  // SPI clock divider
    parameter MAX_BYTES_PER_CS = 3,   // Max bytes per transaction
    parameter CS_INACTIVE_CLKS = 10   // CS inactive clocks
)(
    input wire clk,
    input wire rst_n,
    
    // Command interface
    input wire cmd_valid,           // Command valid
    output reg cmd_ready,           // Ready for command
    input wire [2:0] cmd_type,      // Command type
    input wire [6:0] cmd_addr,      // Address (7 bits for 128 bytes)
    input wire [7:0] cmd_wdata,     // Write data
    output reg [7:0] cmd_rdata,     // Read data
    output reg cmd_done,            // Command complete
    output reg cmd_error,           // Command error
    
    // SPI interface
    output wire spi_cs_n,           // Chip select (active low)
    output wire spi_sclk,           // SPI clock
    output wire spi_mosi,           // Master out slave in
    input wire spi_miso             // Master in slave out
);

    // Command types
    localparam CMD_WREN  = 3'b000;  // Write Enable
    localparam CMD_WRDI  = 3'b001;  // Write Disable
    localparam CMD_RDSR  = 3'b010;  // Read Status Register
    localparam CMD_WRSR  = 3'b011;  // Write Status Register
    localparam CMD_READ  = 3'b100;  // Read Data
    localparam CMD_WRITE = 3'b101;  // Write Data
    
    // AT25010 instruction opcodes
    localparam OP_WREN  = 8'h06;
    localparam OP_WRDI  = 8'h04;
    localparam OP_RDSR  = 8'h05;
    localparam OP_WRSR  = 8'h01;
    localparam OP_READ  = 8'h03;
    localparam OP_WRITE = 8'h02;
    
    // State machine
    localparam ST_IDLE         = 3'd0;
    localparam ST_START_XFER   = 3'd1;
    localparam ST_SEND_BYTES   = 3'd2;
    localparam ST_WAIT_DONE    = 3'd3;
    localparam ST_WRITE_DELAY  = 3'd4;
    localparam ST_DONE         = 3'd5;
    localparam ST_ERROR        = 3'd6;
    
    reg [2:0] state;
    reg [2:0] current_cmd;
    reg [6:0] current_addr;
    reg [7:0] current_wdata;
    reg [15:0] write_delay_counter;  // Write cycle delay
    reg [1:0] byte_count;            // Bytes to send counter
    reg [1:0] bytes_sent;            // Bytes sent counter
    
    // SPI Master signals
    reg [1:0] spi_tx_count;
    reg [7:0] spi_tx_byte;
    reg spi_tx_dv;
    wire spi_tx_ready;
    wire [7:0] spi_rx_byte;
    wire [1:0] spi_rx_count;
    wire spi_rx_dv;
    
    // SPI Master instance from ip/spi-master (Mode 0: CPOL=0, CPHA=0)
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
            cmd_error <= 0;
            cmd_rdata <= 0;
            spi_tx_dv <= 0;
            spi_tx_byte <= 0;
            spi_tx_count <= 0;
            current_cmd <= 0;
            current_addr <= 0;
            current_wdata <= 0;
            byte_count <= 0;
            bytes_sent <= 0;
            write_delay_counter <= 0;
        end else begin
            // Default: clear pulses
            cmd_done <= 0;
            spi_tx_dv <= 0;
            
            case (state)
                ST_IDLE: begin
                    cmd_ready <= 1;
                    
                    if (cmd_valid && cmd_ready) begin
                        cmd_ready <= 0;
                        current_cmd <= cmd_type;
                        current_addr <= cmd_addr;
                        current_wdata <= cmd_wdata;
                        bytes_sent <= 0;
                        
                        // Determine byte count for transaction
                        case (cmd_type)
                            CMD_WREN, CMD_WRDI: byte_count <= 2'd1;  // 1 byte: opcode
                            CMD_RDSR, CMD_WRSR: byte_count <= 2'd2;  // 2 bytes: opcode + data
                            CMD_READ, CMD_WRITE: byte_count <= 2'd3; // 3 bytes: opcode + addr + data
                            default: byte_count <= 2'd1;
                        endcase
                        
                        state <= ST_START_XFER;
                    end
                end
                
                ST_START_XFER: begin
                    // Start SPI transaction with byte count
                    spi_tx_count <= byte_count;
                    
                    // Load first byte (opcode)
                    case (current_cmd)
                        CMD_WREN:  spi_tx_byte <= OP_WREN;
                        CMD_WRDI:  spi_tx_byte <= OP_WRDI;
                        CMD_RDSR:  spi_tx_byte <= OP_RDSR;
                        CMD_WRSR:  spi_tx_byte <= OP_WRSR;
                        CMD_READ:  spi_tx_byte <= OP_READ;
                        CMD_WRITE: spi_tx_byte <= OP_WRITE;
                        default:   spi_tx_byte <= 8'h00;
                    endcase
                    
                    spi_tx_dv <= 1;
                    bytes_sent <= 1;
                    state <= ST_SEND_BYTES;
                end
                
                ST_SEND_BYTES: begin
                    // Send remaining bytes when SPI is ready
                    if (spi_tx_ready && bytes_sent < byte_count) begin
                        bytes_sent <= bytes_sent + 1;
                        
                        // Determine what to send based on byte position
                        if (bytes_sent == 1) begin
                            // Second byte
                            case (current_cmd)
                                CMD_READ, CMD_WRITE: begin
                                    spi_tx_byte <= {1'b0, current_addr};
                                end
                                CMD_WRSR: begin
                                    spi_tx_byte <= current_wdata;
                                end
                                CMD_RDSR: begin
                                    spi_tx_byte <= 8'h00;  // Dummy
                                end
                                default: spi_tx_byte <= 8'h00;
                            endcase
                        end else if (bytes_sent == 2) begin
                            // Third byte
                            case (current_cmd)
                                CMD_WRITE: begin
                                    spi_tx_byte <= current_wdata;
                                end
                                CMD_READ: begin
                                    spi_tx_byte <= 8'h00;  // Dummy
                                end
                                default: spi_tx_byte <= 8'h00;
                            endcase
                        end else begin
                            spi_tx_byte <= 8'h00;
                        end
                        
                        spi_tx_dv <= 1;
                    end else if (bytes_sent >= byte_count) begin
                        state <= ST_WAIT_DONE;
                    end
                end
                
                ST_WAIT_DONE: begin
                    // Wait for all bytes to complete and capture read data
                    if (spi_rx_dv) begin
                        // Capture read data from appropriate byte (RX count lags by 1)
                        case (current_cmd)
                            CMD_RDSR: begin
                                if (spi_rx_count == 2'd1) begin  // Second byte
                                    cmd_rdata <= spi_rx_byte;
                                end
                            end
                            CMD_READ: begin
                                if (spi_rx_count == 2'd2) begin  // Third byte
                                    cmd_rdata <= spi_rx_byte;
                                end
                            end
                        endcase
                    end
                    
                    // When TX is ready again, transaction is complete
                    if (spi_tx_ready && !spi_tx_dv) begin
                        // Check if write operation needs delay
                        if (current_cmd == CMD_WRITE || current_cmd == CMD_WRSR) begin
                            write_delay_counter <= 16'd250;
                            state <= ST_WRITE_DELAY;
                        end else begin
                            state <= ST_DONE;
                        end
                    end
                end
                
                ST_WRITE_DELAY: begin
                    // Wait for self-timed write cycle to complete
                    if (write_delay_counter == 0) begin
                        state <= ST_DONE;
                    end else begin
                        write_delay_counter <= write_delay_counter - 1;
                    end
                end
                
                ST_DONE: begin
                    cmd_done <= 1;
                    state <= ST_IDLE;
                end
                
                ST_ERROR: begin
                    cmd_error <= 1;
                    state <= ST_IDLE;
                end
                
                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
