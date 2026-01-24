module nfc_detector_wrapper (
    input wire clk,
    input wire rst_n,
    
    output wire card_detected,
    output wire [31:0] card_uid,
    output wire card_ready,
    output wire start_auth,
    output wire detection_error,
    output wire [7:0] error_code,
    
    output wire spi_cs_n,
    output wire spi_sclk,
    output wire spi_mosi,
    input wire spi_miso
);

    // Internal signals connecting detector and interface
    wire nfc_cmd_valid;
    wire nfc_cmd_ready;
    wire nfc_cmd_write;
    wire [5:0] nfc_cmd_addr;
    wire [7:0] nfc_cmd_wdata;
    wire [7:0] nfc_cmd_rdata;
    wire nfc_cmd_done;

    nfc_card_detector #(
        .POLL_INTERVAL(10000)  // Fast polling for simulation (10k cycles @ 100MHz = 100us)
    ) u_detector (
        .clk(clk),
        .rst_n(rst_n),
        .card_detected(card_detected),
        .card_uid(card_uid),
        .card_ready(card_ready),
        .start_auth(start_auth),
        .nfc_cmd_valid(nfc_cmd_valid),
        .nfc_cmd_ready(nfc_cmd_ready),
        .nfc_cmd_write(nfc_cmd_write),
        .nfc_cmd_addr(nfc_cmd_addr),
        .nfc_cmd_wdata(nfc_cmd_wdata),
        .nfc_cmd_rdata(nfc_cmd_rdata),
        .nfc_cmd_done(nfc_cmd_done),
        .detection_error(detection_error),
        .error_code(error_code)
    );

    mfrc522_interface #(
        .CLKS_PER_HALF_BIT(2)
    ) u_interface (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_valid(nfc_cmd_valid),
        .cmd_ready(nfc_cmd_ready),
        .cmd_is_write(nfc_cmd_write),
        .cmd_addr(nfc_cmd_addr),
        .cmd_wdata(nfc_cmd_wdata),
        .cmd_rdata(nfc_cmd_rdata),
        .cmd_done(nfc_cmd_done),
        .spi_cs_n(spi_cs_n),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso)
    );

endmodule
