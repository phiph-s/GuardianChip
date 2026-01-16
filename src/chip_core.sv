// SPDX-FileCopyrightText: © 2025 XXX Authors
// SPDX-License-Identifier: Apache-2.0

`default_nettype none

module chip_core #(
    parameter NUM_INPUT_PADS,
    parameter NUM_OUTPUT_PADS,
    parameter NUM_BIDIR_PADS,
    parameter NUM_ANALOG_PADS
)(
    input  logic clk,
    input  logic rst_n,

    input  wire [NUM_INPUT_PADS-1:0]   input_in,
    output wire [NUM_OUTPUT_PADS-1:0]  output_out,

    input  wire [NUM_BIDIR_PADS-1:0]   bidir_in,
    output wire [NUM_BIDIR_PADS-1:0]   bidir_out,
    output wire [NUM_BIDIR_PADS-1:0]   bidir_oe,

    inout  wire [NUM_ANALOG_PADS-1:0]  analog
);

    // -------------------------
    // QFN-24 index mapping
    // -------------------------
    // input_in[0]  = uart_clk   (pin 3)  optional
    // input_in[1]  = uart_rx    (pin 5)  optional
    // input_in[2]  = spi_miso   (pin 15) required
    //
    // output_out[0] = uart_tx      (pin 6)  optional
    // output_out[1] = spi_sclk     (pin 17)
    // output_out[2] = spi_mosi     (pin 16)
    // output_out[3] = cs_0         (pin 14)
    // output_out[4] = cs_1         (pin 13)
    // output_out[5] = status_busy  (pin 22)
    // output_out[6] = status_fault (pin 21)
    // output_out[7] = status_unlock(pin 20)
    //
    // bidir_in/out/oe[0] = user_io_0 (pin 4)
    // bidir_in/out/oe[1] = user_io_1 (pin 7)
    // bidir_in/out/oe[2] = user_io_2 (pin 8)
    // bidir_in/out/oe[3] = user_io_3 (pin 9)
    // bidir_in/out/oe[4] = user_io_4 (pin 10)

    // -------------------------
    // Core wires
    // -------------------------
    logic nfc_irq;

    logic nfc_cs_n, nfc_sclk, nfc_mosi;
    logic eeprom_cs_n, eeprom_sclk, eeprom_mosi;

    logic spi_sclk_bus, spi_mosi_bus;
    logic spi_miso_bus;

    logic door_unlock;
    logic status_unlock, status_fault, status_busy;

    // Inputs from pins
    assign spi_miso_bus = (NUM_INPUT_PADS > 2) ? input_in[2] : 1'b0;

    // Use user_io_0 as NFC IRQ input
    assign nfc_irq = (NUM_BIDIR_PADS > 0) ? bidir_in[0] : 1'b0;

    // -------------------------
    // Shared SPI bus muxing
    // -------------------------
    // Choose which peripheral is actively driving the SPI bus based on CS (active low).
    always_comb begin
        spi_sclk_bus = 1'b0;
        spi_mosi_bus = 1'b0;

        if (!nfc_cs_n) begin
            spi_sclk_bus = nfc_sclk;
            spi_mosi_bus = nfc_mosi
;
        end else if (!eeprom_cs_n) begin
            spi_sclk_bus = eeprom_sclk;
            spi_mosi_bus = eeprom_mosi;
        end
    end

    // -------------------------
    // Output pad driving
    // -------------------------
    // Default all outputs to 0, then assign used bits.
    wire [NUM_OUTPUT_PADS-1:0] out_vec;
    assign out_vec = '0;

    // Safe default for uart_tx (unused): drive '1' (idle-high) if present.
    assign output_out[0] = (NUM_OUTPUT_PADS > 0) ? 1'b1 : 1'b0;

    assign output_out[1] = (NUM_OUTPUT_PADS > 1) ? spi_sclk_bus : 1'b0;
    assign output_out[2] = (NUM_OUTPUT_PADS > 2) ? spi_mosi_bus : 1'b0;
    assign output_out[3] = (NUM_OUTPUT_PADS > 3) ? nfc_cs_n     : 1'b0; // cs_0
    assign output_out[4] = (NUM_OUTPUT_PADS > 4) ? eeprom_cs_n  : 1'b0; // cs_1
    assign output_out[5] = (NUM_OUTPUT_PADS > 5) ? status_busy  : 1'b0;
    assign output_out[6] = (NUM_OUTPUT_PADS > 6) ? status_fault : 1'b0;
    assign output_out[7] = (NUM_OUTPUT_PADS > 7) ? status_unlock: 1'b0;

    // -------------------------
    // Bidir pad directions & values
    // -------------------------
    // user_io_0 is input (nfc_irq)
    // user_io_1 is output (door_unlock)
    // others remain inputs
    
    // Build bidir_out properly
    logic [NUM_BIDIR_PADS-1:0] bidir_out_vec;
    logic [NUM_BIDIR_PADS-1:0] bidir_oe_vec;
    
    generate
        for (genvar i = 0; i < NUM_BIDIR_PADS; i++) begin : gen_bidir
            if (i == 1) begin
                // user_io_1 is output (door_unlock)
                assign bidir_out_vec[i] = door_unlock;
                assign bidir_oe_vec[i] = 1'b1;
            end else begin
                // All other bidirs are inputs (or unused)
                assign bidir_out_vec[i] = 1'b0;
                assign bidir_oe_vec[i] = 1'b0;
            end
        end
    endgenerate
    
    assign bidir_out = bidir_out_vec;
    assign bidir_oe = bidir_oe_vec;

    // -------------------------
    // Instantiate your main core
    // -------------------------
    main_core u_main_core (
        .clk            (clk),
        .rst_n          (rst_n),

        .nfc_irq        (nfc_irq),

        .nfc_spi_miso   (spi_miso_bus),
        .eeprom_spi_miso(spi_miso_bus),

        .nfc_spi_cs_n   (nfc_cs_n),
        .nfc_spi_sclk   (nfc_sclk),
        .nfc_spi_mosi   (nfc_mosi),

        .eeprom_spi_cs_n(eeprom_cs_n),
        .eeprom_spi_sclk(eeprom_sclk),
        .eeprom_spi_mosi(eeprom_mosi),

        .door_unlock    (door_unlock),

        .status_unlock  (status_unlock),
        .status_fault   (status_fault),
        .status_busy    (status_busy)
    );

    // Analog pads currently unused
    // (Leave 'analog' untouched; it's an inout bus on purpose.)

endmodule

`default_nettype wire

