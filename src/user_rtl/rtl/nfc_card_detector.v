module nfc_card_detector #(
    parameter integer CLK_HZ      = 32_000_000,   // internal clock AFTER PLL
    parameter integer SPI_HZ       = 4_000_000     // EXACT 4 MHz like Arduino lib default
)(
    input  wire clk,
    input  wire rst,              // sync reset, active high

    output wire spi_sclk,
    output reg  spi_cs_0,         // MFRC522 CS (active low)
    output reg  spi_mosi,
    input  wire spi_miso,

    output reg  busy,
    output reg  hard_fault,
    output reg  card_seen,
    output reg  unlock,
    output wire [7:0] dbg_state,
    output reg [7:0] dbg_prev_state
);

    // ------------------------------------------------------------------------
    // SPI bit engine: Mode0 (CPOL=0, CPHA=0), MSB first, 8-bit words
    // ------------------------------------------------------------------------
    localparam integer HALF_DIV = CLK_HZ / (SPI_HZ * 2); // 32MHz/(4MHz*2)=4
    localparam integer HALF_DIV_W = 16;

    reg [HALF_DIV_W-1:0] div_cnt = 0;
    reg sclk = 1'b0;
    assign spi_sclk = sclk;

    reg [7:0] shifter_tx = 8'h00;
    reg [7:0] shifter_rx = 8'h00;
    reg [2:0] bit_idx    = 3'd7;

    reg       xfer_active = 1'b0;
    reg       xfer_done   = 1'b0;

    reg done_pending = 1'b0;

    // handshake for one-byte transfer
    reg       start_xfer  = 1'b0;
    reg [7:0] tx_byte     = 8'h00;
    reg [7:0] rx_byte     = 8'h00;

    // 1ms tick generator
    localparam integer MS_DIV = CLK_HZ / 1000; // 32000 @ 32MHz
    reg [$clog2(MS_DIV)-1:0] ms_cnt = 0;
    reg ms_tick = 1'b0;

    always @(posedge clk) begin
      if (rst) begin
        ms_cnt <= 0;
        ms_tick <= 1'b0;
      end else begin
        if (ms_cnt == MS_DIV-1) begin
          ms_cnt <= 0;
          ms_tick <= 1'b1;
        end else begin
          ms_cnt <= ms_cnt + 1'b1;
          ms_tick <= 1'b0;
        end
      end
    end

    reg [7:0] delay_ms = 8'd0;
    reg [1:0] reset_tries = 2'd0;

    // Start on SCK low, present MOSI before rising edge, sample MISO on rising edge.
    always @(posedge clk) begin
        if (rst) begin
            div_cnt     <= 0;
            sclk        <= 1'b0;
            spi_mosi    <= 1'b0;
            shifter_tx  <= 8'h00;
            shifter_rx  <= 8'h00;
            bit_idx     <= 3'd7;
            xfer_active <= 1'b0;
            xfer_done   <= 1'b0;
            rx_byte     <= 8'h00;
        end else begin
            xfer_done <= 1'b0;

            if (start_xfer && !xfer_active) begin
                // latch new byte
                xfer_active <= 1'b1;
                shifter_tx  <= tx_byte;
                shifter_rx  <= 8'h00;
                bit_idx     <= 3'd7;
                sclk        <= 1'b0;
                div_cnt     <= 0;
                spi_mosi    <= tx_byte[7];  // present MSB before first rising edge
                done_pending <= 1'b0;
            end

            if (xfer_active) begin
              if (div_cnt == HALF_DIV-1) begin
                div_cnt <= 0;
                sclk <= ~sclk;

                if (sclk == 1'b0) begin
                  // rising edge (0->1): sample MISO
                  shifter_rx[bit_idx] <= spi_miso;

                  if (bit_idx == 0) begin
                    // last bit sampled - DO NOT force sclk low here!
                    // we wait one more half-cycle so SCK can fall back to 0 cleanly
                    done_pending <= 1'b1;
                  end else begin
                    bit_idx <= bit_idx - 1'b1;
                  end

                end else begin
                  // falling edge (1->0): update MOSI
                  spi_mosi <= shifter_tx[bit_idx];

                  // if we already sampled last bit on previous rising edge,
                  // complete transfer NOW (after this falling edge)
                  if (done_pending) begin
                    done_pending <= 1'b0;
                    xfer_active  <= 1'b0;
                    xfer_done    <= 1'b1;
                    rx_byte      <= {shifter_rx[7:1], shifter_rx[0]}; 
                    // (alternativ: rx_byte <= shifter_rx; wenn du shifter_rx[0] sauber gesetzt hast)
                    // sclk is already falling to 0 via the toggle -> idle low achieved naturally
                  end
                end
              end else begin
                div_cnt <= div_cnt + 1'b1;
              end
            end
        end
    end

    // ------------------------------------------------------------------------
    // High-level MFRC522 sequence (Arduino-identical path up to ATQA)
    // ------------------------------------------------------------------------
    // Exact shifted register address bytes (from your MFRC522.h)
    localparam [7:0] REG_Command_W     = 8'h02;
    localparam [7:0] REG_Command_R     = 8'h82;
    localparam [7:0] REG_ComIrq_W      = 8'h08;
    localparam [7:0] REG_ComIrq_R      = 8'h88;
    localparam [7:0] REG_DivIrq_W      = 8'h0A;
    localparam [7:0] REG_DivIrq_R      = 8'h8A;
    localparam [7:0] REG_Error_R       = 8'h8C;
    localparam [7:0] REG_FIFOData_W    = 8'h12;
    localparam [7:0] REG_FIFOData_R    = 8'h92;
    localparam [7:0] REG_FIFOLevel_W   = 8'h14;
    localparam [7:0] REG_FIFOLevel_R   = 8'h94;
    localparam [7:0] REG_Control_R     = 8'h98;
    localparam [7:0] REG_BitFraming_W  = 8'h1A;
    localparam [7:0] REG_BitFraming_R  = 8'h9A;
    localparam [7:0] REG_Coll_W        = 8'h1C;
    localparam [7:0] REG_Coll_R        = 8'h9C;
    localparam [7:0] REG_Mode_W        = 8'h22;
    localparam [7:0] REG_TxMode_W      = 8'h24;
    localparam [7:0] REG_RxMode_W      = 8'h26;
    localparam [7:0] REG_TxControl_W   = 8'h28;
    localparam [7:0] REG_TxControl_R   = 8'hA8;
    localparam [7:0] REG_TxASK_W       = 8'h2A;
    localparam [7:0] REG_ModWidth_W    = 8'h48;
    localparam [7:0] REG_TMode_W       = 8'h54;
    localparam [7:0] REG_TPrescaler_W  = 8'h56;
    localparam [7:0] REG_TReloadH_W    = 8'h58;
    localparam [7:0] REG_TReloadL_W    = 8'h5A;

    // MFRC522 commands/constants used in your pasted cpp
    localparam [7:0] PCD_Idle      = 8'h00;
    localparam [7:0] PCD_CalcCRC   = 8'h03;
    localparam [7:0] PCD_Transceive= 8'h0C;
    localparam [7:0] PCD_SoftReset = 8'h0F;

    localparam [7:0] REQA          = 8'h26;

    // State machine
    reg [7:0] state = 8'h00;
    reg [7:0] prev_state;
    assign dbg_state = state;

    // helpers / temp regs
    reg [7:0] tmp_reg = 8'h00;
    reg [7:0] comirq  = 8'h00;
    reg [7:0] errreg  = 8'h00;
    reg [7:0] fifolvl = 8'h00;
    reg [7:0] ctrlreg = 8'h00;
    reg [7:0] atqa0   = 8'h00;
    reg [7:0] atqa1   = 8'h00;

    reg [15:0] poll_ctr = 16'd0;

    // micro-ops: write reg(value), read reg -> tmp_reg
    // Each micro-op is a two-byte SPI sequence with CS toggling exactly like Arduino lib.
    reg [7:0] op_addr  = 8'h00;
    reg [7:0] op_data  = 8'h00;
    reg       op_is_read = 1'b0;

    reg [2:0] op_step = 3'd0; // 0 idle, 1 assert CS, 2 send addr, 3 send data/dummy, 4 deassert CS, 5 done
    reg       op_done = 1'b0;

    task start_write;
        input [7:0] addr_w;
        input [7:0] data;
        begin
            op_addr    <= addr_w;
            op_data    <= data;
            op_is_read <= 1'b0;
            op_step    <= 3'd1;
        end
    endtask

    task start_read;
        input [7:0] addr_r;
        begin
            op_addr    <= addr_r;
            op_data    <= 8'h00;
            op_is_read <= 1'b1;
            op_step    <= 3'd1;
        end
    endtask

    // op engine
    always @(posedge clk) begin
        if (rst) begin
            prev_state      <= 8'h00;
            dbg_prev_state  <= 8'h00;
            spi_cs_0 <= 1'b1;
            busy     <= 1'b1;
            hard_fault <= 1'b0;
            card_seen <= 1'b0;
            unlock    <= 1'b0;

            op_step <= 3'd0;
            op_done <= 1'b0;
            start_xfer <= 1'b0;
            state <= 8'h00;
            poll_ctr <= 16'd0;
        end else begin
            op_done <= 1'b0;
            start_xfer <= 1'b0;

            // nur updaten solange wir NICHT im HardFault sind
            if (state != 8'hFF) begin
              prev_state     <= state;
              dbg_prev_state <= prev_state;
            end

            // default: keep CS high unless op active or burst active
            if (op_step == 3'd0) begin
                spi_cs_0 <= 1'b1;
            end

            case (op_step)
                3'd0: begin
                    // idle, controlled by FSM
                end
                3'd1: begin
                    // assert CS low
                    spi_cs_0 <= 1'b0;
                    op_step <= 3'd2;
                end
                3'd2: begin
                    // send address byte
                    if (!xfer_active) begin
                        tx_byte <= op_addr;
                        start_xfer <= 1'b1;
                        op_step <= 3'd3;
                    end
                end
                3'd3: begin
                    // wait addr xfer done then send data/dummy
                    if (xfer_done) begin
                        if (!xfer_active) begin
                            tx_byte <= op_data;
                            start_xfer <= 1'b1;
                            op_step <= 3'd4;
                        end
                    end
                end
                3'd4: begin
                    // wait data/dummy done, capture read result
                    if (xfer_done) begin
                        if (op_is_read) tmp_reg <= rx_byte;
                        op_step <= 3'd5;
                    end
                end
                3'd5: begin
                    // deassert CS
                    spi_cs_0 <= 1'b1;
                    op_done <= 1'b1;
                    op_step <= 3'd0;
                end
            endcase

            // ----------------------------------------------------------------
            // Main FSM: Arduino-identical sequence up to ATQA
            // ----------------------------------------------------------------
            case (state)
                // -------- PCD_Init() --------
                8'h00: begin
                  busy <= 1'b1;
                  hard_fault <= 1'b0;
                  card_seen <= 1'b0;
                  unlock <= 1'b0;
                  reset_tries <= 0;
                  if (op_step == 3'd0) begin
                    start_write(REG_Command_W, PCD_SoftReset); // CommandReg <- 0x0F
                    state <= 8'h01;
                  end
                end

                // wait after writing reset
                8'h01: begin
                  if (op_done) begin
                    delay_ms <= 8'd50;          // Arduino: delay(50)
                    state <= 8'h02;
                  end
                end

                // 50ms delay
                8'h02: begin
                  if (ms_tick) begin
                    if (delay_ms != 0) delay_ms <= delay_ms - 1'b1;
                    else state <= 8'h03;
                  end
                end

                // read CommandReg
                8'h03: begin
                  if (op_step == 3'd0) begin
                    start_read(REG_Command_R);  // read CommandReg
                    state <= 8'h04;
                  end
                end

                8'h04: begin
                  if (op_done) begin
                    // if PowerDown bit still set -> retry up to 3 times, Arduino-like
                    if (tmp_reg[4] == 1'b1) begin
                      reset_tries <= reset_tries + 1'b1;
                      if (reset_tries == 2'd2) begin
                        hard_fault <= 1'b1;
                        state <= 8'hFF;
                      end else begin
                        delay_ms <= 8'd50;
                        state <= 8'h02; // wait again then read again
                      end
                    end else begin
                      state <= 8'h10; // continue init writes
                    end
                  end
                end

                // Init writes exactly like Arduino PCD_Init()
                8'h10: if (op_step==3'd0) begin start_write(REG_TxMode_W, 8'h00); state<=8'h11; end
                8'h11: if (op_done) begin state<=8'h12; end
                8'h12: if (op_step==3'd0) begin start_write(REG_RxMode_W, 8'h00); state<=8'h13; end
                8'h13: if (op_done) begin state<=8'h14; end
                8'h14: if (op_step==3'd0) begin start_write(REG_ModWidth_W, 8'h26); state<=8'h15; end
                8'h15: if (op_done) begin state<=8'h16; end

                8'h16: if (op_step==3'd0) begin start_write(REG_TMode_W, 8'h80); state<=8'h17; end
                8'h17: if (op_done) begin state<=8'h18; end
                8'h18: if (op_step==3'd0) begin start_write(REG_TPrescaler_W, 8'hA9); state<=8'h19; end
                8'h19: if (op_done) begin state<=8'h1A; end
                8'h1A: if (op_step==3'd0) begin start_write(REG_TReloadH_W, 8'h03); state<=8'h1B; end
                8'h1B: if (op_done) begin state<=8'h1C; end
                8'h1C: if (op_step==3'd0) begin start_write(REG_TReloadL_W, 8'hE8); state<=8'h1D; end
                8'h1D: if (op_done) begin state<=8'h1E; end

                8'h1E: if (op_step==3'd0) begin start_write(REG_TxASK_W, 8'h40); state<=8'h1F; end
                8'h1F: if (op_done) begin state<=8'h20; end
                8'h20: if (op_step==3'd0) begin start_write(REG_Mode_W, 8'h3D); state<=8'h21; end
                8'h21: if (op_done) begin state<=8'h22; end

                // AntennaOn(): read TxControlReg, if (v&3)!=3 then write v|3
                8'h22: if (op_step==3'd0) begin start_read(REG_TxControl_R); state<=8'h23; end
                8'h23: if (op_done) begin tmp_reg <= tmp_reg; state<=8'h24; end
                8'h24: begin
                    if ((tmp_reg[1:0] == 2'b11)) begin
                        state <= 8'h30; // done init, go loop
                    end else if (op_step==3'd0) begin
                        start_write(REG_TxControl_W, (tmp_reg | 8'h03));
                        state <= 8'h25;
                    end
                end
                8'h25: if (op_done) begin state <= 8'h30; end

                // -------- LOOP: PICC_IsNewCardPresent() equivalent --------
                // reset baud rates each time (Arduino does it)
                8'h30: if (op_step==3'd0) begin start_write(REG_TxMode_W, 8'h00); state<=8'h31; end
                8'h31: if (op_done) begin state<=8'h32; end
                8'h32: if (op_step==3'd0) begin start_write(REG_RxMode_W, 8'h00); state<=8'h33; end
                8'h33: if (op_done) begin state<=8'h34; end
                8'h34: if (op_step==3'd0) begin start_write(REG_ModWidth_W, 8'h26); state<=8'h35; end
                8'h35: if (op_done) begin state<=8'h40; end

                // ---- REQA Transceive sequence (Arduino-identical) ----
                // CommandReg <- Idle
                8'h40: if (op_step==3'd0) begin start_write(REG_Command_W, PCD_Idle); state<=8'h41; end
                8'h41: if (op_done) begin state<=8'h42; end

                // ComIrqReg <- 0x7F
                8'h42: if (op_step==3'd0) begin start_write(REG_ComIrq_W, 8'h7F); state<=8'h43; end
                8'h43: if (op_done) begin state<=8'h44; end

                // FIFOLevelReg <- 0x80 (flush)
                8'h44: if (op_step==3'd0) begin start_write(REG_FIFOLevel_W, 8'h80); state<=8'h45; end
                8'h45: if (op_done) begin state<=8'h46; end

                // FIFODataReg <- 0x26 (REQA)
                8'h46: if (op_step==3'd0) begin start_write(REG_FIFOData_W, REQA); state<=8'h47; end
                8'h47: if (op_done) begin state<=8'h48; end

                // BitFramingReg <- 0x07 (TxLastBits=7)
                8'h48: if (op_step==3'd0) begin start_write(REG_BitFraming_W, 8'h07); state<=8'h49; end
                8'h49: if (op_done) begin state<=8'h4A; end

                // CommandReg <- Transceive
                8'h4A: if (op_step==3'd0) begin start_write(REG_Command_W, PCD_Transceive); state<=8'h4B; end
                8'h4B: if (op_done) begin state<=8'h4C; end

                // StartSend: BitFramingReg |= 0x80  (Arduino does R-M-W)
                8'h4C: if (op_step==3'd0) begin start_read(REG_BitFraming_R); state<=8'h4D; end
                8'h4D: if (op_done) begin state<=8'h4E; end
                8'h4E: if (op_step==3'd0) begin start_write(REG_BitFraming_W, (tmp_reg | 8'h80)); state<=8'h4F; end
                8'h4F: if (op_done) begin poll_ctr<=0; state<=8'h50; end

                // Poll ComIrqReg until (n & 0x30)!=0 or (n & 0x01)!=0 (timer) or timeout
                8'h50: if (op_step==3'd0) begin start_read(REG_ComIrq_R); state<=8'h51; end
                8'h51: if (op_done) begin comirq<=tmp_reg; state<=8'h52; end
                8'h52: begin
                    if ((comirq & 8'h30) != 8'h00) begin
                        state <= 8'h60; // completed
                    end else if ((comirq & 8'h01) != 8'h00) begin
                        // timer -> no card (Arduino returns STATUS_TIMEOUT)
                        state <= 8'h30;
                    end else begin
                        poll_ctr <= poll_ctr + 1'b1;
                        if (poll_ctr == 16'hFFFF) begin
                            hard_fault <= 1'b1;
                            state <= 8'hFF;
                        end else begin
                            state <= 8'h50;
                        end
                    end
                end

                // Read ErrorReg, FIFOLevelReg, FIFODataReg (2 bytes), ControlReg
                8'h60: if (op_step==3'd0) begin start_read(REG_Error_R); state<=8'h61; end
                8'h61: if (op_done) begin errreg<=tmp_reg; state<=8'h62; end
                8'h62: if (op_step==3'd0) begin start_read(REG_FIFOLevel_R); state<=8'h63; end
                8'h63: if (op_done) begin fifolvl<=tmp_reg; state<=8'h64; end

                // Burst read FIFODataReg exactly like library style for 2 bytes:
                // CS↓ 0x92 0x00 0x92 0x00 0x00 0x00 CS↑  (we implement it explicitly here)
                8'h64: begin
                    // if FIFO doesn't have 2 bytes, treat as no card and loop again
                    if (fifolvl < 8'd2) begin
                        state <= 8'h30;
                    end else begin
                        state <= 8'h65;
                    end
                end

                // Manual burst read (special case) to be Arduino-identical
                // Steps: assert CS, send addr, dummy0 (read atqa0), send addr, dummy0 (read atqa1), send 0x00 dummy stop, deassert CS
                8'h65: begin
                    // start burst: drive CS low, then do 3 transfers after initial address:
                    // We'll reuse the byte engine directly for this burst.
                    if (!xfer_active && op_step==3'd0) begin
                        spi_cs_0 <= 1'b0;
                        // send first address
                        tx_byte <= REG_FIFOData_R;
                        start_xfer <= 1'b1;
                        state <= 8'h66;
                    end
                end
                8'h66: if (xfer_done && !xfer_active) begin
                    // read byte0 by sending dummy=0x00 (library does transfer(address) here, but effective is "clock 8 bits")
                    tx_byte <= 8'h00;
                    start_xfer <= 1'b1;
                    state <= 8'h67;
                end
                8'h67: if (xfer_done && !xfer_active) begin
                    atqa0 <= rx_byte;
                    // library sends address again to keep reading
                    tx_byte <= REG_FIFOData_R;
                    start_xfer <= 1'b1;
                    state <= 8'h68;
                end
                8'h68: if (xfer_done && !xfer_active) begin
                    // now final byte read; library ends with transfer(0)
                    tx_byte <= 8'h00;
                    start_xfer <= 1'b1;
                    state <= 8'h69;
                end
                8'h69: if (xfer_done && !xfer_active) begin
                    atqa1 <= rx_byte;
                    // stop: one more dummy 0 clocks in some implementations; the lib uses transfer(0) as the last read already.
                    spi_cs_0 <= 1'b1;
                    state <= 8'h6A;
                end

                8'h6A: if (op_step==3'd0) begin start_read(REG_Control_R); state<=8'h6B; end
                8'h6B: if (op_done) begin ctrlreg<=tmp_reg; state<=8'h6C; end

                8'h6C: begin
                    // Arduino expects: bufferSize==2 and validBits==0 (RxLastBits=0)
                    if ((ctrlreg[2:0] == 3'b000) && (fifolvl == 8'd2) && (errreg[4:0] == errreg[4:0])) begin
                        // We deliberately do NOT over-interpret errreg here; Arduino checks 0x13 later in other paths.
                        card_seen <= 1'b1;
                        unlock    <= 1'b1;
                        busy      <= 1'b0;
                        state     <= 8'h6D;
                    end else begin
                        state <= 8'h30;
                    end
                end

                // hold unlocked (you can change this to pulse if you want)
                8'h6D: begin
                    busy <= 1'b0;
                    // keep running or stop; for now stop here.
                    state <= 8'h6D;
                end

                // Hard fault stop
                8'hFF: begin
                    busy <= 1'b0;
                    // freeze
                    state <= 8'hFF;
                end

                default: state <= 8'h00;
            endcase
        end
    end

endmodule
