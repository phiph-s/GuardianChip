// SPDX-FileCopyrightText: © 2025 LibreLane Template Contributors
// SPDX-License-Identifier: Apache-2.0

`default_nettype none

module chip_top #(
    // Fixed to match your QFN-24 pinout intent:
    // - 2x core supplies (VDD/VSS)   -> pins 11/12 and 19/18 in your list
    // - 1x IO supplies (IOVDD/IOVSS) -> pins 24/23
    parameter int NUM_VDD_PADS   = 2,
    parameter int NUM_VSS_PADS   = 2,
    parameter int NUM_IOVDD_PADS = 1,
    parameter int NUM_IOVSS_PADS = 1,

    // Signal pads used by chip_core mapping
    parameter int NUM_INPUT_PADS  = 3,  // uart_clk, uart_rx, spi_miso
    parameter int NUM_OUTPUT_PADS = 8,  // uart_tx, spi_sclk, spi_mosi, cs0, cs1, status_busy/fault/unlock
    parameter int NUM_BIDIR_PADS  = 5,  // user_io_0..4

    // Keep chip_core interface happy without exposing analog pads to the package
    parameter int NUM_ANALOG_PADS = 1
)(
    `ifdef USE_POWER_PINS
    inout wire IOVDD,
    inout wire IOVSS,
    inout wire VDD,
    inout wire VSS,
    `endif

    // Dedicated pads
    inout wire clk_PAD,      // Pin 2: sys_clk
    inout wire rst_PAD,      // Pin 1: rst (see polarity note below)

    // Buses for the remaining pads
    inout wire [NUM_INPUT_PADS-1:0]  input_PAD,
    inout wire [NUM_OUTPUT_PADS-1:0] output_PAD,
    inout wire [NUM_BIDIR_PADS-1:0]  bidir_PAD
);

    // -------------------------
    // Internal signals to core
    // -------------------------
    wire clk_PAD2CORE;

    wire rst_raw;       // raw value coming from the pad
    wire rst_n_PAD2CORE; // what chip_core expects (active-low)

    wire [NUM_INPUT_PADS-1:0]  input_PAD2CORE;
    wire [NUM_OUTPUT_PADS-1:0] output_CORE2PAD;
    wire [NUM_BIDIR_PADS-1:0]  bidir_PAD2CORE;
    wire [NUM_BIDIR_PADS-1:0]  bidir_CORE2PAD;
    wire [NUM_BIDIR_PADS-1:0]  bidir_CORE2PAD_OE;

    // No analog pads on package; keep an internal dummy bus only
    wire [NUM_ANALOG_PADS-1:0] analog_dummy;

    // -------------------------
    // Supply pad instances (named for config.yaml PAD_* lists)
    // -------------------------
    // IO supplies (pins 24/23)
    (* keep *) (* keep *) sg13g2_IOPadIOVdd iovdd_pad_0 (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS)
        `endif
    );

    (* keep *) (* keep *) sg13g2_IOPadIOVss iovss_pad_0 (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS)
        `endif
    );

    // Core supplies (pins 11/12 and 19/18)
    (* keep *) (* keep *) sg13g2_IOPadVdd vdd_pad_0 (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS)
        `endif
    );

    (* keep *) (* keep *) sg13g2_IOPadVss vss_pad_0 (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS)
        `endif
    );

    (* keep *) (* keep *) sg13g2_IOPadVdd vdd_pad_1 (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS)
        `endif
    );

    (* keep *) (* keep *) sg13g2_IOPadVss vss_pad_1 (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS)
        `endif
    );

    // -------------------------
    // Signal pad instances (flat names, no [] anywhere)
    // -------------------------

    // Pin 2: sys_clk
    (* keep *) sg13g2_IOPadIn sys_clk_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .p2c(clk_PAD2CORE),
        .pad(clk_PAD)
    );

    // Pin 1: rst (raw)
    (* keep *) sg13g2_IOPadIn rst_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .p2c(rst_raw),
        .pad(rst_PAD)
    );

    // Reset polarity selection:
    // - If external pin "rst" is ACTIVE-HIGH: keep this inversion (recommended with your naming "rst")
    // - If external pin "rst" is ACTIVE-LOW: change to "assign rst_n_PAD2CORE = rst_raw;"
    assign rst_n_PAD2CORE = ~rst_raw;

    // Inputs:
    // Pin 3: uart_clk -> input_PAD[0]
    (* keep *) sg13g2_IOPadIn uart_clk_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .p2c(input_PAD2CORE[0]),
        .pad(input_PAD[0])
    );

    // Pin 5: uart_rx -> input_PAD[1]
    (* keep *) sg13g2_IOPadIn uart_rx_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .p2c(input_PAD2CORE[1]),
        .pad(input_PAD[1])
    );

    // Pin 15: spi_miso -> input_PAD[2]
    (* keep *) sg13g2_IOPadIn spi_miso_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .p2c(input_PAD2CORE[2]),
        .pad(input_PAD[2])
    );

    // Outputs (use same drive-strength cell as your template unless you want smaller):
    // Pin 6: uart_tx -> output_PAD[0]
    (* keep *) sg13g2_IOPadOut30mA uart_tx_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p(output_CORE2PAD[0]),
        .pad(output_PAD[0])
    );

    // Pin 17: spi_sclk -> output_PAD[1]
    (* keep *) sg13g2_IOPadOut30mA spi_sclk_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p(output_CORE2PAD[1]),
        .pad(output_PAD[1])
    );

    // Pin 16: spi_mosi -> output_PAD[2]
    (* keep *) sg13g2_IOPadOut30mA spi_mosi_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p(output_CORE2PAD[2]),
        .pad(output_PAD[2])
    );

    // Pin 14: cs_0 -> output_PAD[3]
    (* keep *) sg13g2_IOPadOut30mA cs_0_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p(output_CORE2PAD[3]),
        .pad(output_PAD[3])
    );

    // Pin 13: cs_1 -> output_PAD[4]
    (* keep *) sg13g2_IOPadOut30mA cs_1_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p(output_CORE2PAD[4]),
        .pad(output_PAD[4])
    );

    // Pin 22: status_busy -> output_PAD[5]
    (* keep *) sg13g2_IOPadOut30mA status_busy_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p(output_CORE2PAD[5]),
        .pad(output_PAD[5])
    );

    // Pin 21: status_fault -> output_PAD[6]
    (* keep *) sg13g2_IOPadOut30mA status_fault_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p(output_CORE2PAD[6]),
        .pad(output_PAD[6])
    );

    // Pin 20: status_unlock -> output_PAD[7]
    (* keep *) sg13g2_IOPadOut30mA status_unlock_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p(output_CORE2PAD[7]),
        .pad(output_PAD[7])
    );

    // Bidirectional user IO (tri-state):
    // Pin 4:  user_io_0 -> bidir_PAD[0]
    // Pins 7–10: user_io_1..4 -> bidir_PAD[1..4]
    (* keep *) sg13g2_IOPadInOut30mA user_io_0_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p   (bidir_CORE2PAD[0]),
        .c2p_en(bidir_CORE2PAD_OE[0]),
        .p2c   (bidir_PAD2CORE[0]),
        .pad   (bidir_PAD[0])
    );

    (* keep *) sg13g2_IOPadInOut30mA user_io_1_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p   (bidir_CORE2PAD[1]),
        .c2p_en(bidir_CORE2PAD_OE[1]),
        .p2c   (bidir_PAD2CORE[1]),
        .pad   (bidir_PAD[1])
    );

    (* keep *) sg13g2_IOPadInOut30mA user_io_2_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p   (bidir_CORE2PAD[2]),
        .c2p_en(bidir_CORE2PAD_OE[2]),
        .p2c   (bidir_PAD2CORE[2]),
        .pad   (bidir_PAD[2])
    );

    (* keep *) sg13g2_IOPadInOut30mA user_io_3_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p   (bidir_CORE2PAD[3]),
        .c2p_en(bidir_CORE2PAD_OE[3]),
        .p2c   (bidir_PAD2CORE[3]),
        .pad   (bidir_PAD[3])
    );

    (* keep *) sg13g2_IOPadInOut30mA user_io_4_pad (
        `ifdef USE_POWER_PINS
        .iovdd(IOVDD), .iovss(IOVSS), .vdd(VDD), .vss(VSS),
        `endif
        .c2p   (bidir_CORE2PAD[4]),
        .c2p_en(bidir_CORE2PAD_OE[4]),
        .p2c   (bidir_PAD2CORE[4]),
        .pad   (bidir_PAD[4])
    );

    // -------------------------
    // Core
    // -------------------------
    (* keep *) chip_core #(
        .NUM_INPUT_PADS (NUM_INPUT_PADS),
        .NUM_OUTPUT_PADS(NUM_OUTPUT_PADS),
        .NUM_BIDIR_PADS (NUM_BIDIR_PADS),
        .NUM_ANALOG_PADS(NUM_ANALOG_PADS)
    ) i_chip_core (
        .clk        (clk_PAD2CORE),
        .rst_n      (rst_n_PAD2CORE),
        .input_in   (input_PAD2CORE),
        .output_out (output_CORE2PAD),
        .bidir_in   (bidir_PAD2CORE),
        .bidir_out  (bidir_CORE2PAD),
        .bidir_oe   (bidir_CORE2PAD_OE),
        .analog     (analog_dummy)
    );

endmodule

`default_nettype wire

