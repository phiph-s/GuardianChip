module ulx3s_test_top (
    input  wire        clk_25mhz,
    input  wire [6:0]  btn,
    output reg         spi_sclk,
    output reg         spi_cs_0,
    output wire        spi_cs_1,
    output reg         spi_mosi,
    input  wire        spi_miso,
    output wire        uart_txd,
    input  wire        uart_rxd,
    output wire        mode,
    output wire        busy,
    output wire        hard_fault,
    output wire        unlock,
    output wire [7:0]  led
);

    localparam integer START_PERIOD_TICKS = 25_000_000; // ~1s
    localparam integer CS_SETUP_TICKS     = 100;        // ~4us
    localparam integer HALF_SCLK_TICKS    = 25;         // ~500kHz SCLK

    localparam [2:0] IDLE=3'd0, CS_SETUP=3'd1, TRANSFER=3'd2, DONE=3'd3;

    // MFRC522 commands (read)
    localparam [7:0] CMD_VERSION   = 8'hEE; // reg 0x37
    localparam [7:0] CMD_COMIEN    = 8'hE6; // reg 0x33
    localparam [7:0] CMD_TXCONTROL = 8'hFC; // reg 0x3E

    // Setze das auf 0, wenn du sicher bist, dass Bit-Reverse NICHT nötig ist
    localparam USE_BIT_REVERSE = 1;

    reg [31:0] timer = 0;
    reg [2:0]  state = IDLE;

    reg [23:0] tx_shift = 0;
    reg [23:0] rx_shift = 0;
    reg [23:0] rx_data  = 0;

    reg [5:0]  bit_count = 0;  // 0..23
    reg [7:0]  cs_setup_cnt = 0;
    reg [7:0]  half_cnt = 0;

    reg [2:0]  test_cycle = 0;

    // Bit-Reverse-Funktion
    function [7:0] reverse_bits;
        input [7:0] data;
        integer i;
        begin
            for (i = 0; i < 8; i = i + 1)
                reverse_bits[i] = data[7-i];
        end
    endfunction

    // wähle Command BYTE direkt (ohne cmd_byte-Race)
    wire [7:0] sel_cmd =
        (test_cycle == 3'd0) ? CMD_VERSION :
        (test_cycle == 3'd1) ? CMD_COMIEN  :
        (test_cycle == 3'd2) ? CMD_TXCONTROL :
                               CMD_VERSION;

    always @(posedge clk_25mhz) begin
        timer <= timer + 1;

        case (state)
            IDLE: begin
                spi_cs_0 <= 1'b1;
                spi_sclk <= 1'b0;
                spi_mosi <= 1'b0;

                if (timer >= START_PERIOD_TICKS-1) begin
                    timer <= 0;

                    // frame: [command][dummy][dummy]
                    tx_shift <= {sel_cmd, 16'h0000};
                    rx_shift <= 24'd0;

                    test_cycle <= test_cycle + 1;

                    spi_cs_0 <= 1'b0;
                    cs_setup_cnt <= 0;
                    state <= CS_SETUP;
                end
            end

            CS_SETUP: begin
                if (cs_setup_cnt < CS_SETUP_TICKS-1) begin
                    cs_setup_cnt <= cs_setup_cnt + 1;
                end else begin
                    spi_sclk  <= 1'b0;
                    half_cnt  <= 0;
                    bit_count <= 0;

                    // MOSI preload: erstes Bit vor der ersten rising edge stabil machen
                    spi_mosi  <= tx_shift[23];
                    tx_shift  <= {tx_shift[22:0], 1'b0};

                    state <= TRANSFER;
                end
            end

            TRANSFER: begin
                if (half_cnt < HALF_SCLK_TICKS-1) begin
                    half_cnt <= half_cnt + 1;
                end else begin
                    half_cnt <= 0;

                    if (spi_sclk == 1'b0) begin
                        // rising edge: sample MISO (Mode 0)
                        spi_sclk <= 1'b1;
                        rx_shift <= {rx_shift[22:0], spi_miso};
                    end else begin
                        // falling edge: shift next MOSI
                        spi_sclk <= 1'b0;

                        bit_count <= bit_count + 1;

                        if (bit_count == 6'd23) begin
                            state <= DONE;
                        end else begin
                            spi_mosi <= tx_shift[23];
                            tx_shift <= {tx_shift[22:0], 1'b0};
                        end
                    end
                end
            end

            DONE: begin
                spi_cs_0 <= 1'b1;
                spi_sclk <= 1'b0;
                spi_mosi <= 1'b0;

                rx_data <= rx_shift;

                state <= IDLE;
            end

            default: state <= IDLE;
        endcase
    end

    assign spi_cs_1 = 1'b1;
    assign uart_txd = 1'b1;

    wire [7:0] byte0 = rx_data[23:16];
    wire [7:0] byte1 = rx_data[15:8];
    wire [7:0] byte2 = rx_data[7:0];

    wire [7:0] byte0_r = USE_BIT_REVERSE ? reverse_bits(byte0) : byte0;
    wire [7:0] byte1_r = USE_BIT_REVERSE ? reverse_bits(byte1) : byte1;
    wire [7:0] byte2_r = USE_BIT_REVERSE ? reverse_bits(byte2) : byte2;

    // Default: byte1_r (oft Registerwert), Buttons zum Prüfen:
    // btn[2] -> byte0_r, btn[1] -> byte1_r, sonst -> byte2_r
    wire [7:0] display_byte = btn[2] ? byte0_r : (btn[1] ? byte1_r : byte2_r);
    assign led = display_byte;

    assign mode = (state == TRANSFER);
    assign busy = (state != IDLE);
    assign hard_fault = btn[2];
    assign unlock = btn[0];

endmodule
