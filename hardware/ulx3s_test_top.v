module ulx3s_test_top (
    input  wire        clk_25mhz,
    input  wire [6:0]  btn,
    output wire        spi_sclk,
    output wire        spi_cs_0,
    output wire        spi_cs_1,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire        uart_txd,
    input  wire        uart_rxd,
    output wire        mode,
    output wire        busy,
    output wire        hard_fault,
    output wire        unlock,
    output wire [7:0]  led
);

    // buttons: assume btn[0] is reset (adjust if you want)
    wire rst = btn[0];

    // ------------------------------------------------------------------------
    // PLL: 25 MHz -> 32 MHz (then /8 in SPI engine gives exact 4 MHz)
    // Fout = Fin * CLKFB_DIV / (CLKI_DIV * CLKOP_DIV)
    // 32 = 25 * 32 / (25 * 1)
    // ------------------------------------------------------------------------
    wire clk_32mhz;
    wire pll_lock;
    wire [7:0] dbg_state;

    // EHXPLLL primitive (ECP5). If your build flow needs different params,
    // tell me which toolchain and I’ll adapt it.
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
        .ENCLKOP(1'b0), .ENCLKOS(1'b0), .ENCLKOS2(1'b0), .ENCLKOS3(1'b0),
        .CLKOP(clk_32mhz),
        .CLKOS(), .CLKOS2(), .CLKOS3(),
        .LOCK(pll_lock)
    );

    // ------------------------------------------------------------------------
    // NFC detector
    // ------------------------------------------------------------------------
    nfc_card_detector #(
        .CLK_HZ(32_000_000),
        .SPI_HZ(4_000_000)
    ) nfc_i (
        .clk(clk_32mhz),
        .rst(~pll_lock | rst),
        .spi_sclk(spi_sclk),
        .spi_cs_0(spi_cs_0),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .busy(busy),
        .hard_fault(hard_fault),
        .card_seen(),    // internal only; reflected on LEDs below via unlock/busy
        .unlock(unlock),
        .dbg_state(dbg_state)
    );

    // Do not touch the other SPI device
    assign spi_cs_1 = 1'b1;

    // UART idle high
    assign uart_txd = 1'b1;

    // your extra outputs
    assign mode = 1'b0;

    assign led = hard_fault ? dbg_prev_state : dbg_state;

endmodule
