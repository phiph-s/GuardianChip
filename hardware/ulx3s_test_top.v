module ulx3s_test_top (
    input  wire        clk_25mhz,
    input  wire [6:0]  btn,

    // Shared SPI bus
    output wire        spi_sclk,
    output wire        spi_cs_0,   // MFRC522
    output wire        spi_cs_1,   // EEPROM
    output wire        spi_mosi,
    input  wire        spi_miso,

    // misc IO
    output wire        uart_txd,
    input  wire        uart_rxd,
    output wire        mode,

    // status
    output wire        busy,
    output wire        hard_fault,
    output wire        unlock,

    output wire [7:0]  led
);

    wire rst = btn[0];

    // ------------------------------------------------------------------------
    // PLL: 25 MHz -> 32 MHz (MFRC522 logic expects 32 MHz)
    // ------------------------------------------------------------------------
    wire clk_32mhz;
    wire pll_lock;

    // ECP5 PLL (same config as before)
    EHXPLLL #(
        .PLLRST_ENA("ENABLED"),
        .INTFB_WAKE("DISABLED"),
        .STDBY_ENABLE("DISABLED"),
        .DPHASE_SOURCE("DISABLED"),
        .OUTDIVIDER_MUXA("DIVA"),
        .OUTDIVIDER_MUXB("DIVB"),
        .OUTDIVIDER_MUXC("DIVC"),
        .OUTDIVIDER_MUXD("DIVD"),
        .CLKI_DIV(25),
        .CLKFB_DIV(32),
        .CLKOP_DIV(1),
        .CLKOP_ENABLE("ENABLED"),
        .CLKOP_CPHASE(0),
        .CLKOP_FPHASE(0)
    ) pll_i (
        .CLKI(clk_25mhz),
        .CLKFB(clk_32mhz),
        .PHASESEL0(1'b0), .PHASESEL1(1'b0),
        .PHASEDIR(1'b0), .PHASESTEP(1'b0), .PHASELOADREG(1'b0),
        .STDBY(1'b0),
        .PLLWAKESYNC(1'b0),
        .RST(rst),
        .ENCLKOP(1'b1), .ENCLKOS(1'b0), .ENCLKOS2(1'b0), .ENCLKOS3(1'b0),
        .CLKOP(clk_32mhz),
        .CLKOS(), .CLKOS2(), .CLKOS3(),
        .LOCK(pll_lock)
    );

    // ------------------------------------------------------------------------
    // Shared SPI master (Mode 0)
    // ------------------------------------------------------------------------
    wire       spi_xfer_active;
    wire       spi_xfer_done;
    wire [7:0] spi_rx_byte;

    wire       spi_start_xfer;
    wire [7:0] spi_tx_byte;

    spi_master #(
        .CLK_HZ(32_000_000),
        .SPI_HZ(4_000_000)
    ) u_spi (
        .clk(clk_32mhz),
        .rst(rst | ~pll_lock),

        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),

        .start_xfer(spi_start_xfer),
        .tx_byte(spi_tx_byte),
        .xfer_active(spi_xfer_active),
        .xfer_done(spi_xfer_done),
        .rx_byte(spi_rx_byte)
    );

    // ------------------------------------------------------------------------
    // MFRC522 detector core (chip-select + protocol FSM)
    // ------------------------------------------------------------------------
    wire [7:0] dbg_state_unused;
    wire [7:0] dbg_prev_state_unused;

    nfc_card_detector #(
        .CLK_HZ(32_000_000),
        .SPI_HZ(4_000_000)
    ) u_nfc (
        .clk(clk_32mhz),
        .rst(rst | ~pll_lock),

        .spi_cs_0(spi_cs_0),

        .start_xfer(spi_start_xfer),
        .tx_byte(spi_tx_byte),
        .xfer_active(spi_xfer_active),
        .xfer_done(spi_xfer_done),
        .rx_byte(spi_rx_byte),

        .busy(busy),
        .hard_fault(hard_fault),
        .card_seen(),
        .unlock(unlock),

        .dbg_state(dbg_state_unused),
        .dbg_prev_state(dbg_prev_state_unused)
    );

    // TODO: EEPROM chip-select + SPI transactions
    assign spi_cs_1 = 1'b1;

    // UART idle high (unused)
    assign uart_txd = 1'b1;
    assign mode     = 1'b0;

    // Minimal LED status
    // [7]=hard_fault, [6]=unlock, [5]=busy, [4]=pll_lock, [3:0]=buttons[4:1]
    assign led = {hard_fault, unlock, busy, pll_lock, btn[4:1]};

endmodule