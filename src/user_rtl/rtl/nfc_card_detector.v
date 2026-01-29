module nfc_card_detector #(
    parameter integer CLK_HZ      = 32_000_000,   // internal clock AFTER PLL
    parameter integer SPI_HZ       = 4_000_000     // EXACT 4 MHz like Arduino lib default
)(
    input  wire clk,
    input  wire rst,              // sync reset, active high
output reg  spi_cs_0,         // MFRC522 CS (active low)

    // SPI byte-transfer interface (drives external spi_master, Mode 0)
    output reg        start_xfer,
    output reg  [7:0] tx_byte,
    input  wire       xfer_active,
    input  wire       xfer_done,
    input  wire [7:0] rx_byte,
output reg  busy,
    output reg  hard_fault,
    output reg  card_seen,
    output reg  unlock,
    output wire [7:0] dbg_state,
    output reg [7:0] dbg_prev_state,
    output reg [7:0] max_state,    // highest state reached (for debug)
    output reg [31:0] card_uid,   // 4-byte UID output
    output reg [7:0] dbg_uid_bcc,  // received BCC for debug
    output reg [7:0] dbg_calc_bcc, // calculated BCC for debug
    output wire [7:0] dbg_uid0,    // uid0 register for debug
    output wire [7:0] dbg_uid1,    // uid1 register for debug
    output wire [7:0] dbg_uid2,    // uid2 register for debug
    output wire [7:0] dbg_uid3,    // uid3 register for debug
    output wire [7:0] dbg_comirq   // comirq register for debug
);

    // ------------------------------------------------------------------------
    // SPI bit engine: Mode0 (CPOL=0, CPHA=0), MSB first, 8-bit words
    // ------------------------------------------------------------------------
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
    localparam [7:0] REG_CRCResultL_R  = 8'hC4;  // CRCResultRegL read = 0x22<<1 | 0x80
    localparam [7:0] REG_CRCResultH_R  = 8'hC2;  // CRCResultRegH read = 0x21<<1 | 0x80
    localparam [7:0] REG_ModWidth_W    = 8'h48;
    localparam [7:0] REG_TMode_W       = 8'h54;
    localparam [7:0] REG_TPrescaler_W  = 8'h56;
    localparam [7:0] REG_TReloadH_W    = 8'h58;
    localparam [7:0] REG_TReloadL_W    = 8'h5A;
    localparam [7:0] REG_Version_R     = 8'hEE;  // VersionReg = 0x37<<1 | 0x80 = 0x6E | 0x80 = 0xEE

    // MFRC522 commands/constants used in your pasted cpp
    localparam [7:0] PCD_Idle      = 8'h00;
    localparam [7:0] PCD_CalcCRC   = 8'h03;
    localparam [7:0] PCD_Transceive= 8'h0C;
    localparam [7:0] PCD_SoftReset = 8'h0F;

    localparam [7:0] REQA          = 8'h26;
    localparam [7:0] PICC_CMD_SEL_CL1 = 8'h93;  // Anti collision/Select, Cascade Level 1

    // State machine
    reg [7:0] state = 8'h00;
    reg [7:0] prev_state;
    assign dbg_state = state;
    assign dbg_uid0 = uid0;
    assign dbg_uid1 = uid1;
    assign dbg_uid2 = uid2;
    assign dbg_uid3 = uid3;
    assign dbg_comirq = comirq;

    // helpers / temp regs
    reg [7:0] tmp_reg = 8'h00;
    reg [7:0] comirq  = 8'h00;
    reg [7:0] errreg  = 8'h00;
    reg [7:0] fifolvl = 8'h00;
    reg [7:0] ctrlreg = 8'h00;
    reg [7:0] atqa0   = 8'h00;
    reg [7:0] atqa1   = 8'h00;

    // UID storage (anticollision response: UID0-3 + BCC)
    reg [7:0] uid0 = 8'h00;
    reg [7:0] uid1 = 8'h00;
    reg [7:0] uid2 = 8'h00;
    reg [7:0] uid3 = 8'h00;
    reg [7:0] uid_bcc = 8'h00;
    
    // CRC result storage
    reg [7:0] crc_lo = 8'h00;
    reg [7:0] crc_hi = 8'h00;
    
    // SAK storage
    reg [7:0] sak = 8'h00;
    
    // Burst read counter
    reg [3:0] burst_cnt = 4'd0;

    reg [11:0] unlock_ms = 12'd0;

    reg [15:0] poll_ctr = 16'd0;

    // LAYR APDU protocol helpers
    reg [4:0] apdu_state = 5'd0;
    reg [3:0] apdu_idx = 4'd0;
    reg [4:0] resp_idx = 5'd0;
    reg [7:0] sel_sw1 = 8'h00;
    reg [7:0] sel_sw2 = 8'h00;
    reg [7:0] auth_sw1 = 8'h00;
    reg [7:0] auth_sw2 = 8'h00;
    reg       pattern_ok = 1'b1;
    reg [127:0] auth_pattern = 128'h0;

    localparam [4:0] APDU_CFG_TXMODE      = 5'd0;
    localparam [4:0] APDU_CFG_RXMODE      = 5'd1;
    localparam [4:0] APDU_SEL_IDLE        = 5'd2;
    localparam [4:0] APDU_SEL_IRQ_CLEAR   = 5'd3;
    localparam [4:0] APDU_SEL_FIFO_FLUSH  = 5'd4;
    localparam [4:0] APDU_SEL_FIFO_WRITE  = 5'd5;
    localparam [4:0] APDU_SEL_BITFRAMING  = 5'd6;
    localparam [4:0] APDU_SEL_CMD         = 5'd7;
    localparam [4:0] APDU_SEL_STARTSEND   = 5'd8;
    localparam [4:0] APDU_SEL_POLL        = 5'd9;
    localparam [4:0] APDU_SEL_ERR         = 5'd10;
    localparam [4:0] APDU_SEL_LEN         = 5'd11;
    localparam [4:0] APDU_SEL_READ        = 5'd12;
    localparam [4:0] APDU_SEL_STATUS      = 5'd13;
    localparam [4:0] APDU_AUTH_IDLE       = 5'd14;
    localparam [4:0] APDU_AUTH_IRQ_CLEAR  = 5'd15;
    localparam [4:0] APDU_AUTH_FIFO_FLUSH = 5'd16;
    localparam [4:0] APDU_AUTH_FIFO_WRITE = 5'd17;
    localparam [4:0] APDU_AUTH_BITFRAMING = 5'd18;
    localparam [4:0] APDU_AUTH_CMD        = 5'd19;
    localparam [4:0] APDU_AUTH_STARTSEND  = 5'd20;
    localparam [4:0] APDU_AUTH_POLL       = 5'd21;
    localparam [4:0] APDU_AUTH_ERR        = 5'd22;
    localparam [4:0] APDU_AUTH_LEN        = 5'd23;
    localparam [4:0] APDU_AUTH_READ       = 5'd24;
    localparam [4:0] APDU_AUTH_STATUS     = 5'd25;

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

    function [7:0] select_apdu_byte;
        input [3:0] idx;
        begin
            case (idx)
                4'd0:  select_apdu_byte = 8'h00;
                4'd1:  select_apdu_byte = 8'hA4;
                4'd2:  select_apdu_byte = 8'h04;
                4'd3:  select_apdu_byte = 8'h00;
                4'd4:  select_apdu_byte = 8'h06;
                4'd5:  select_apdu_byte = 8'hF0;
                4'd6:  select_apdu_byte = 8'hBA;
                4'd7:  select_apdu_byte = 8'hAA;
                4'd8:  select_apdu_byte = 8'hAA;
                4'd9:  select_apdu_byte = 8'hAD;
                4'd10: select_apdu_byte = 8'h01;
                default: select_apdu_byte = 8'h00;
            endcase
        end
    endfunction

    function [7:0] auth_init_byte;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: auth_init_byte = 8'h80;
                3'd1: auth_init_byte = 8'h10;
                3'd2: auth_init_byte = 8'h00;
                3'd3: auth_init_byte = 8'h00;
                default: auth_init_byte = 8'h00;
            endcase
        end
    endfunction

    // op engine
    always @(posedge clk) begin
        if (rst) begin
            prev_state      <= 8'h00;
            dbg_prev_state  <= 8'h00;
            max_state       <= 8'h00;
            spi_cs_0 <= 1'b1;
            busy     <= 1'b1;
            hard_fault <= 1'b0;
            card_seen <= 1'b0;
            unlock    <= 1'b0;
            card_uid  <= 32'h0;

            op_step <= 3'd0;
            op_done <= 1'b0;
            start_xfer <= 1'b0;
            state <= 8'h00;
            poll_ctr <= 16'd0;
            burst_cnt <= 4'd0;
            uid0 <= 8'h00;
            uid1 <= 8'h00;
            uid2 <= 8'h00;
            uid3 <= 8'h00;
            uid_bcc <= 8'h00;
            crc_lo <= 8'h00;
            crc_hi <= 8'h00;
            sak <= 8'h00;
            dbg_uid_bcc <= 8'h00;
            dbg_calc_bcc <= 8'h00;
            apdu_state <= APDU_CFG_TXMODE;
            apdu_idx <= 4'd0;
            resp_idx <= 5'd0;
            sel_sw1 <= 8'h00;
            sel_sw2 <= 8'h00;
            auth_sw1 <= 8'h00;
            auth_sw2 <= 8'h00;
            pattern_ok <= 1'b1;
            auth_pattern <= 128'h0;
        end else begin
            op_done <= 1'b0;
            start_xfer <= 1'b0;

            // Track maximum state reached (for debugging)
            if (state > max_state) begin
                max_state <= state;
            end

            // nur updaten solange wir NICHT im HardFault sind
            if (state != 8'hFF) begin
              prev_state     <= state;
              dbg_prev_state <= prev_state;
            end

            // default: keep CS high unless op active or burst active
            // WICHTIG: Während des manuellen Burst-Reads (States 0x65-0x69, 0x8F-0x95)
            // darf CS NICHT automatisch high gesetzt werden!
            if (op_step == 3'd0 && 
                !((state >= 8'h65 && state <= 8'h69) ||   // ATQA burst read
                  (state >= 8'h8F && state <= 8'h95))) begin  // UID burst read
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
                  // Initialize uid registers to 0 (will be overwritten by UID read)
                  uid0 <= 8'h00;
                  uid1 <= 8'h00;
                  uid2 <= 8'h00;
                  uid3 <= 8'h00;
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
                    uid0 <= tmp_reg;  // Store CommandReg read in uid0 for debug (b2)
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
                // First read Version register to verify SPI communication
                8'h10: if (op_step==3'd0) begin start_read(REG_Version_R); state<=8'h0D; end
                8'h0D: if (op_done) begin 
                    uid3 <= tmp_reg;  // Store version in uid3 for debug display
                    state<=8'h0E; 
                end
                8'h0E: if (op_step==3'd0) begin start_write(REG_TxMode_W, 8'h00); state<=8'h11; end
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
                    // After valid ATQA, proceed to PICC_Select (anticollision)
                    if ((ctrlreg[2:0] == 3'b000) && (fifolvl == 8'd2)) begin
                        card_seen <= 1'b1;
                        // Continue to PICC_Select: clear CollReg bit 7 first (Arduino: PCD_ClearRegisterBitMask(CollReg, 0x80))
                        state <= 8'h70;
                    end else begin
                        state <= 8'h30;
                    end
                end

                // ======== PICC_Select() - Anticollision/Select sequence ========
                // Step 1: Read CollReg
                8'h70: if (op_step==3'd0) begin start_read(REG_Coll_R); state<=8'h71; end
                8'h71: if (op_done) begin state<=8'h72; end
                // Step 2: Write CollReg with bit 7 cleared (ValuesAfterColl=1)
                8'h72: if (op_step==3'd0) begin start_write(REG_Coll_W, tmp_reg & 8'h7F); state<=8'h73; end
                8'h73: if (op_done) begin state<=8'h74; end

                // ======== ANTICOLLISION command: send SEL_CL1 (0x93), NVB=0x20 ========
                // Arduino: buffer[0]=0x93, buffer[1]=0x20, txLastBits=0, bufferUsed=2
                // Transceive: Idle, clear IRQ, flush FIFO, write 2 bytes, BitFraming=0x00, Transceive, StartSend
                8'h74: if (op_step==3'd0) begin start_write(REG_Command_W, PCD_Idle); state<=8'h75; end
                8'h75: if (op_done) begin state<=8'h76; end
                8'h76: if (op_step==3'd0) begin start_write(REG_ComIrq_W, 8'h7F); state<=8'h77; end
                8'h77: if (op_done) begin state<=8'h78; end
                8'h78: if (op_step==3'd0) begin start_write(REG_FIFOLevel_W, 8'h80); state<=8'h79; end
                8'h79: if (op_done) begin state<=8'h7A; end
                // Write SEL_CL1 to FIFO
                8'h7A: if (op_step==3'd0) begin start_write(REG_FIFOData_W, PICC_CMD_SEL_CL1); state<=8'h7B; end
                8'h7B: if (op_done) begin state<=8'h7C; end
                // Write NVB=0x20 to FIFO
                8'h7C: if (op_step==3'd0) begin start_write(REG_FIFOData_W, 8'h20); state<=8'h7D; end
                8'h7D: if (op_done) begin state<=8'h7E; end
                // BitFramingReg = 0x00 (no alignment, full bytes)
                8'h7E: if (op_step==3'd0) begin start_write(REG_BitFraming_W, 8'h00); state<=8'h7F; end
                8'h7F: if (op_done) begin state<=8'h80; end
                // Start Transceive
                8'h80: if (op_step==3'd0) begin start_write(REG_Command_W, PCD_Transceive); state<=8'h81; end
                8'h81: if (op_done) begin state<=8'h82; end
                // StartSend: set bit 7 of BitFramingReg
                8'h82: if (op_step==3'd0) begin start_read(REG_BitFraming_R); state<=8'h83; end
                8'h83: if (op_done) begin state<=8'h84; end
                8'h84: if (op_step==3'd0) begin start_write(REG_BitFraming_W, tmp_reg | 8'h80); state<=8'h85; end
                8'h85: if (op_done) begin poll_ctr<=0; state<=8'h86; end

                // Poll for anticollision response
                8'h86: if (op_step==3'd0) begin start_read(REG_ComIrq_R); state<=8'h87; end
                8'h87: if (op_done) begin comirq<=tmp_reg; state<=8'h88; end
                8'h88: begin
                    if ((comirq & 8'h30) != 8'h00) begin
                        state <= 8'h89; // completed
                    end else if ((comirq & 8'h01) != 8'h00) begin
                        state <= 8'h30; // timeout
                    end else begin
                        poll_ctr <= poll_ctr + 1'b1;
                        if (poll_ctr == 16'hFFFF) begin
                            state <= 8'h30;
                        end else begin
                            state <= 8'h86;
                        end
                    end
                end

                // Read ErrorReg, check for errors
                8'h89: if (op_step==3'd0) begin start_read(REG_Error_R); state<=8'h8A; end
                8'h8A: if (op_done) begin errreg<=tmp_reg; state<=8'h8B; end
                8'h8B: begin
                    // Check for BufferOvfl, ParityErr, ProtocolErr (0x13)
                    if ((errreg & 8'h13) != 8'h00) begin
                        state <= 8'h30; // error, retry
                    end else begin
                        state <= 8'h8C;
                    end
                end

                // Read FIFOLevel (should be 5: UID0-3 + BCC)
                8'h8C: if (op_step==3'd0) begin start_read(REG_FIFOLevel_R); state<=8'h8D; end
                8'h8D: if (op_done) begin fifolvl<=tmp_reg; state<=8'h8E; end
                8'h8E: begin
                    if (fifolvl < 8'd5) begin
                        state <= 8'h30; // not enough data
                    end else begin
                        state <= 8'h8F;
                    end
                end

                // Burst read 5 bytes from FIFO (UID0-3 + BCC)
                // Pattern: send addr, then alternate (dummy, read) pairs
                8'h8F: begin
                    if (!xfer_active) begin
                        spi_cs_0 <= 1'b0;
                        tx_byte <= REG_FIFOData_R;
                        start_xfer <= 1'b1;
                        state <= 8'h90;
                    end
                end
                // SPI is full-duplex: rx_byte from transfer N contains data from transfer N-1
                // Transfer 1 (0x8F): sent addr, rx = garbage
                // Transfer 2 (0x90): send addr, rx = uid0 (from xfer 1 - still garbage!)
                // We need 6 transfers total for 5 data bytes!
                
                // 0x90: xfer 1 done (garbage), start xfer 2
                8'h90: if (xfer_done && !xfer_active) begin
                    // rx_byte is garbage from first transfer, discard it
                    tx_byte <= REG_FIFOData_R;  // send addr to clock out uid0
                    start_xfer <= 1'b1;
                    state <= 8'h91;
                end
                // 0x91: xfer 2 done, rx = uid0
                8'h91: if (xfer_done && !xfer_active) begin
                    uid0 <= rx_byte;
                    tx_byte <= REG_FIFOData_R;
                    start_xfer <= 1'b1;
                    state <= 8'h92;
                end
                // 0x92: xfer 3 done, rx = uid1
                8'h92: if (xfer_done && !xfer_active) begin
                    uid1 <= rx_byte;
                    tx_byte <= REG_FIFOData_R;
                    start_xfer <= 1'b1;
                    state <= 8'h93;
                end
                // 0x93: xfer 4 done, rx = uid2
                8'h93: if (xfer_done && !xfer_active) begin
                    uid2 <= rx_byte;
                    tx_byte <= REG_FIFOData_R;
                    start_xfer <= 1'b1;
                    state <= 8'h94;
                end
                // 0x94: xfer 5 done, rx = uid3
                8'h94: if (xfer_done && !xfer_active) begin
                    uid3 <= rx_byte;
                    tx_byte <= REG_FIFOData_R;  // read BCC from FIFO
                    start_xfer <= 1'b1;
                    state <= 8'h95;
                end
                // 0x95: xfer 6 done, rx = BCC
                8'h95: if (xfer_done && !xfer_active) begin
                    uid_bcc <= rx_byte;
                    dbg_uid_bcc <= rx_byte;  // Save for debug output
                    spi_cs_0 <= 1'b1;
                    state <= 8'h9B;
                end
                
                // One cycle delay for uid_bcc to settle
                8'h9B: begin
                    dbg_calc_bcc <= uid0 ^ uid1 ^ uid2 ^ uid3;  // Save calculated BCC
                    state <= 8'h96;
                end

                // Verify BCC: should be uid0 ^ uid1 ^ uid2 ^ uid3
                8'h96: begin
                    if (uid_bcc == (uid0 ^ uid1 ^ uid2 ^ uid3)) begin
                        state <= 8'hA0; // BCC OK, continue with SELECT
                    end else begin
                        state <= 8'h30; // BCC error, retry
                    end
                end

                // ======== SELECT command: send SEL_CL1, NVB=0x70, UID0-3, BCC, CRC ========
                // First calculate CRC using PCD_CalculateCRC

                // CRC calculation: write 7 bytes to FIFO, run CalcCRC, read result
                // Step 1: Idle
                8'hA0: if (op_step==3'd0) begin start_write(REG_Command_W, PCD_Idle); state<=8'hA1; end
                8'hA1: if (op_done) begin state<=8'hA2; end
                // Step 2: Clear DivIrqReg bit 2 (CRCIRq)
                8'hA2: if (op_step==3'd0) begin start_write(REG_DivIrq_W, 8'h04); state<=8'hA3; end
                8'hA3: if (op_done) begin state<=8'hA4; end
                // Step 3: Flush FIFO
                8'hA4: if (op_step==3'd0) begin start_write(REG_FIFOLevel_W, 8'h80); state<=8'hA5; end
                8'hA5: if (op_done) begin state<=8'hA6; end
                // Step 4: Write 7 bytes to FIFO for CRC: SEL, NVB, UID0-3, BCC
                8'hA6: if (op_step==3'd0) begin start_write(REG_FIFOData_W, PICC_CMD_SEL_CL1); state<=8'hA7; end
                8'hA7: if (op_done) begin state<=8'hA8; end
                8'hA8: if (op_step==3'd0) begin start_write(REG_FIFOData_W, 8'h70); state<=8'hA9; end  // NVB=0x70 (7 bytes)
                8'hA9: if (op_done) begin state<=8'hAA; end
                8'hAA: if (op_step==3'd0) begin start_write(REG_FIFOData_W, uid0); state<=8'hAB; end
                8'hAB: if (op_done) begin state<=8'hAC; end
                8'hAC: if (op_step==3'd0) begin start_write(REG_FIFOData_W, uid1); state<=8'hAD; end
                8'hAD: if (op_done) begin state<=8'hAE; end
                8'hAE: if (op_step==3'd0) begin start_write(REG_FIFOData_W, uid2); state<=8'hAF; end
                8'hAF: if (op_done) begin state<=8'hB0; end
                8'hB0: if (op_step==3'd0) begin start_write(REG_FIFOData_W, uid3); state<=8'hB1; end
                8'hB1: if (op_done) begin state<=8'hB2; end
                8'hB2: if (op_step==3'd0) begin start_write(REG_FIFOData_W, uid_bcc); state<=8'hB3; end
                8'hB3: if (op_done) begin state<=8'hB4; end
                // Step 5: Start CRC calculation
                8'hB4: if (op_step==3'd0) begin start_write(REG_Command_W, PCD_CalcCRC); state<=8'hB5; end
                8'hB5: if (op_done) begin poll_ctr<=0; state<=8'hB6; end
                // Step 6: Poll DivIrqReg for CRCIRq (bit 2)
                8'hB6: if (op_step==3'd0) begin start_read(REG_DivIrq_R); state<=8'hB7; end
                8'hB7: if (op_done) begin
                    if ((tmp_reg & 8'h04) != 8'h00) begin
                        state <= 8'hB8; // CRC done
                    end else begin
                        poll_ctr <= poll_ctr + 1'b1;
                        if (poll_ctr == 16'hFFFF) begin
                            state <= 8'h30; // timeout
                        end else begin
                            state <= 8'hB6;
                        end
                    end
                end
                // Step 7: Stop CRC (Idle)
                8'hB8: if (op_step==3'd0) begin start_write(REG_Command_W, PCD_Idle); state<=8'hB9; end
                8'hB9: if (op_done) begin state<=8'hBA; end
                // Step 8: Read CRC result (low byte first)
                8'hBA: if (op_step==3'd0) begin start_read(REG_CRCResultL_R); state<=8'hBB; end
                8'hBB: if (op_done) begin crc_lo<=tmp_reg; state<=8'hBC; end
                8'hBC: if (op_step==3'd0) begin start_read(REG_CRCResultH_R); state<=8'hBD; end
                8'hBD: if (op_done) begin crc_hi<=tmp_reg; state<=8'hC0; end

                // ======== Now transmit SELECT command with CRC ========
                // Transceive: SEL, NVB, UID0-3, BCC, CRC_L, CRC_H (9 bytes total)
                8'hC0: if (op_step==3'd0) begin start_write(REG_Command_W, PCD_Idle); state<=8'hC1; end
                8'hC1: if (op_done) begin state<=8'hC2; end
                8'hC2: if (op_step==3'd0) begin start_write(REG_ComIrq_W, 8'h7F); state<=8'hC3; end
                8'hC3: if (op_done) begin state<=8'hC4; end
                8'hC4: if (op_step==3'd0) begin start_write(REG_FIFOLevel_W, 8'h80); state<=8'hC5; end
                8'hC5: if (op_done) begin state<=8'hC6; end
                // Write 9 bytes to FIFO
                8'hC6: if (op_step==3'd0) begin start_write(REG_FIFOData_W, PICC_CMD_SEL_CL1); state<=8'hC7; end
                8'hC7: if (op_done) begin state<=8'hC8; end
                8'hC8: if (op_step==3'd0) begin start_write(REG_FIFOData_W, 8'h70); state<=8'hC9; end
                8'hC9: if (op_done) begin state<=8'hCA; end
                8'hCA: if (op_step==3'd0) begin start_write(REG_FIFOData_W, uid0); state<=8'hCB; end
                8'hCB: if (op_done) begin state<=8'hCC; end
                8'hCC: if (op_step==3'd0) begin start_write(REG_FIFOData_W, uid1); state<=8'hCD; end
                8'hCD: if (op_done) begin state<=8'hCE; end
                8'hCE: if (op_step==3'd0) begin start_write(REG_FIFOData_W, uid2); state<=8'hCF; end
                8'hCF: if (op_done) begin state<=8'hD0; end
                8'hD0: if (op_step==3'd0) begin start_write(REG_FIFOData_W, uid3); state<=8'hD1; end
                8'hD1: if (op_done) begin state<=8'hD2; end
                8'hD2: if (op_step==3'd0) begin start_write(REG_FIFOData_W, uid_bcc); state<=8'hD3; end
                8'hD3: if (op_done) begin state<=8'hD4; end
                8'hD4: if (op_step==3'd0) begin start_write(REG_FIFOData_W, crc_lo); state<=8'hD5; end
                8'hD5: if (op_done) begin state<=8'hD6; end
                8'hD6: if (op_step==3'd0) begin start_write(REG_FIFOData_W, crc_hi); state<=8'hD7; end
                8'hD7: if (op_done) begin state<=8'hD8; end
                // BitFramingReg = 0x00
                8'hD8: if (op_step==3'd0) begin start_write(REG_BitFraming_W, 8'h00); state<=8'hD9; end
                8'hD9: if (op_done) begin state<=8'hDA; end
                // Start Transceive
                8'hDA: if (op_step==3'd0) begin start_write(REG_Command_W, PCD_Transceive); state<=8'hDB; end
                8'hDB: if (op_done) begin state<=8'hDC; end
                // StartSend
                8'hDC: if (op_step==3'd0) begin start_read(REG_BitFraming_R); state<=8'hDD; end
                8'hDD: if (op_done) begin state<=8'hDE; end
                8'hDE: if (op_step==3'd0) begin start_write(REG_BitFraming_W, tmp_reg | 8'h80); state<=8'hDF; end
                8'hDF: if (op_done) begin poll_ctr<=0; state<=8'hE0; end

                // Poll for SELECT response
                8'hE0: if (op_step==3'd0) begin start_read(REG_ComIrq_R); state<=8'hE1; end
                8'hE1: if (op_done) begin comirq<=tmp_reg; state<=8'hE2; end
                8'hE2: begin
                    if ((comirq & 8'h30) != 8'h00) begin
                        state <= 8'hE3;
                    end else if ((comirq & 8'h01) != 8'h00) begin
                        state <= 8'h30; // timeout
                    end else begin
                        poll_ctr <= poll_ctr + 1'b1;
                        if (poll_ctr == 16'hFFFF) begin
                            state <= 8'h30;
                        end else begin
                            state <= 8'hE0;
                        end
                    end
                end

                // Check errors
                8'hE3: if (op_step==3'd0) begin start_read(REG_Error_R); state<=8'hE4; end
                8'hE4: if (op_done) begin errreg<=tmp_reg; state<=8'hE5; end
                8'hE5: begin
                    if ((errreg & 8'h13) != 8'h00) begin
                        state <= 8'h30;
                    end else begin
                        state <= 8'hE6;
                    end
                end

                // Read FIFOLevel (should be 3: SAK + 2 CRC bytes)
                8'hE6: if (op_step==3'd0) begin start_read(REG_FIFOLevel_R); state<=8'hE7; end
                8'hE7: if (op_done) begin fifolvl<=tmp_reg; state<=8'hE8; end
                8'hE8: begin
                    if (fifolvl < 8'd3) begin
                        state <= 8'h30;
                    end else begin
                        state <= 8'hE9;
                    end
                end

                // Read ControlReg for validBits
                8'hE9: if (op_step==3'd0) begin start_read(REG_Control_R); state<=8'hEA; end
                8'hEA: if (op_done) begin ctrlreg<=tmp_reg; state<=8'hEB; end
                8'hEB: begin
                    // SAK must be exactly 24 bits (3 bytes, no extra bits)
                    if ((ctrlreg[2:0] != 3'b000) || (fifolvl != 8'd3)) begin
                        state <= 8'h30;
                    end else begin
                        state <= 8'hEC;
                    end
                end

                // Read SAK (1 byte)
                8'hEC: if (op_step==3'd0) begin start_read(REG_FIFOData_R); state<=8'hED; end
                8'hED: if (op_done) begin sak<=tmp_reg; state<=8'hEE; end

                // SUCCESS! Card UID read complete
                8'hEE: begin
                    // Card UID read complete; proceed with LAYR APDU protocol
                    card_uid <= {uid0, uid1, uid2, uid3};
                    unlock <= 1'b0;
                    busy <= 1'b1;
                    hard_fault <= 1'b0;
                    apdu_state <= APDU_CFG_TXMODE;
                    apdu_idx <= 4'd0;
                    resp_idx <= 5'd0;
                    pattern_ok <= 1'b1;
                    auth_pattern <= 128'h0;
                    poll_ctr <= 16'd0;
                    state <= 8'hF1;
                end

                // ======== LAYR APDU protocol (SELECT + AUTH_INIT) ========
                8'hF1: begin
                    busy <= 1'b1;
                    case (apdu_state)
                        APDU_CFG_TXMODE: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_TxMode_W, 8'h80); // enable CRC
                            end
                            if (op_done) apdu_state <= APDU_CFG_RXMODE;
                        end

                        APDU_CFG_RXMODE: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_RxMode_W, 8'h80); // enable CRC
                            end
                            if (op_done) apdu_state <= APDU_SEL_IDLE;
                        end

                        // ---- SELECT APDU ----
                        APDU_SEL_IDLE: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_Command_W, PCD_Idle);
                            end
                            if (op_done) apdu_state <= APDU_SEL_IRQ_CLEAR;
                        end

                        APDU_SEL_IRQ_CLEAR: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_ComIrq_W, 8'h7F);
                            end
                            if (op_done) apdu_state <= APDU_SEL_FIFO_FLUSH;
                        end

                        APDU_SEL_FIFO_FLUSH: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_FIFOLevel_W, 8'h80);
                            end
                            if (op_done) begin
                                apdu_idx <= 4'd0;
                                apdu_state <= APDU_SEL_FIFO_WRITE;
                            end
                        end

                        APDU_SEL_FIFO_WRITE: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_FIFOData_W, select_apdu_byte(apdu_idx));
                            end
                            if (op_done) begin
                                if (apdu_idx == 4'd10) begin
                                    apdu_state <= APDU_SEL_BITFRAMING;
                                end else begin
                                    apdu_idx <= apdu_idx + 1'b1;
                                end
                            end
                        end

                        APDU_SEL_BITFRAMING: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_BitFraming_W, 8'h00);
                            end
                            if (op_done) apdu_state <= APDU_SEL_CMD;
                        end

                        APDU_SEL_CMD: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_Command_W, PCD_Transceive);
                            end
                            if (op_done) apdu_state <= APDU_SEL_STARTSEND;
                        end

                        APDU_SEL_STARTSEND: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_BitFraming_W, 8'h80);
                            end
                            if (op_done) begin
                                poll_ctr <= 16'd0;
                                apdu_state <= APDU_SEL_POLL;
                            end
                        end

                        APDU_SEL_POLL: begin
                            if (op_step == 3'd0) begin
                                start_read(REG_ComIrq_R);
                            end
                            if (op_done) begin
                                comirq <= tmp_reg;
                                if ((tmp_reg & 8'h30) != 8'h00) begin
                                    apdu_state <= APDU_SEL_ERR;
                                end else if ((tmp_reg & 8'h01) != 8'h00) begin
                                    hard_fault <= 1'b1;
                                    unlock_ms <= 12'd3000;
                                    state <= 8'hF0;
                                end else begin
                                    poll_ctr <= poll_ctr + 1'b1;
                                    if (poll_ctr == 16'hFFFF) begin
                                        hard_fault <= 1'b1;
                                        state <= 8'hFF;
                                    end else begin
                                        apdu_state <= APDU_SEL_POLL;
                                    end
                                end
                            end
                        end

                        APDU_SEL_ERR: begin
                            if (op_step == 3'd0) begin
                                start_read(REG_Error_R);
                            end
                            if (op_done) begin
                                errreg <= tmp_reg;
                                if ((tmp_reg & 8'h13) != 8'h00) begin
                                    hard_fault <= 1'b1;
                                    unlock_ms <= 12'd3000;
                                    state <= 8'hF0;
                                end else begin
                                    apdu_state <= APDU_SEL_LEN;
                                end
                            end
                        end

                        APDU_SEL_LEN: begin
                            if (op_step == 3'd0) begin
                                start_read(REG_FIFOLevel_R);
                            end
                            if (op_done) begin
                                fifolvl <= tmp_reg;
                                if (tmp_reg < 8'd2) begin
                                    hard_fault <= 1'b1;
                                    unlock_ms <= 12'd3000;
                                    state <= 8'hF0;
                                end else begin
                                    resp_idx <= 5'd0;
                                    apdu_state <= APDU_SEL_READ;
                                end
                            end
                        end

                        APDU_SEL_READ: begin
                            if (op_step == 3'd0) begin
                                start_read(REG_FIFOData_R);
                            end
                            if (op_done) begin
                                if (resp_idx == 5'd0)
                                    sel_sw1 <= tmp_reg;
                                else
                                    sel_sw2 <= tmp_reg;
                                if (resp_idx == 5'd1)
                                    apdu_state <= APDU_SEL_STATUS;
                                else
                                    resp_idx <= resp_idx + 1'b1;
                            end
                        end

                        APDU_SEL_STATUS: begin
                            if (sel_sw1 == 8'h90 && sel_sw2 == 8'h00) begin
                                apdu_state <= APDU_AUTH_IDLE;
                            end else begin
                                hard_fault <= 1'b1;
                                unlock_ms <= 12'd3000;
                                state <= 8'hF0;
                            end
                        end

                        // ---- AUTH_INIT APDU ----
                        APDU_AUTH_IDLE: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_Command_W, PCD_Idle);
                            end
                            if (op_done) apdu_state <= APDU_AUTH_IRQ_CLEAR;
                        end

                        APDU_AUTH_IRQ_CLEAR: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_ComIrq_W, 8'h7F);
                            end
                            if (op_done) apdu_state <= APDU_AUTH_FIFO_FLUSH;
                        end

                        APDU_AUTH_FIFO_FLUSH: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_FIFOLevel_W, 8'h80);
                            end
                            if (op_done) begin
                                apdu_idx <= 4'd0;
                                apdu_state <= APDU_AUTH_FIFO_WRITE;
                            end
                        end

                        APDU_AUTH_FIFO_WRITE: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_FIFOData_W, auth_init_byte(apdu_idx[2:0]));
                            end
                            if (op_done) begin
                                if (apdu_idx == 4'd3) begin
                                    apdu_state <= APDU_AUTH_BITFRAMING;
                                end else begin
                                    apdu_idx <= apdu_idx + 1'b1;
                                end
                            end
                        end

                        APDU_AUTH_BITFRAMING: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_BitFraming_W, 8'h00);
                            end
                            if (op_done) apdu_state <= APDU_AUTH_CMD;
                        end

                        APDU_AUTH_CMD: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_Command_W, PCD_Transceive);
                            end
                            if (op_done) apdu_state <= APDU_AUTH_STARTSEND;
                        end

                        APDU_AUTH_STARTSEND: begin
                            if (op_step == 3'd0) begin
                                start_write(REG_BitFraming_W, 8'h80);
                            end
                            if (op_done) begin
                                poll_ctr <= 16'd0;
                                apdu_state <= APDU_AUTH_POLL;
                            end
                        end

                        APDU_AUTH_POLL: begin
                            if (op_step == 3'd0) begin
                                start_read(REG_ComIrq_R);
                            end
                            if (op_done) begin
                                comirq <= tmp_reg;
                                if ((tmp_reg & 8'h30) != 8'h00) begin
                                    apdu_state <= APDU_AUTH_ERR;
                                end else if ((tmp_reg & 8'h01) != 8'h00) begin
                                    hard_fault <= 1'b1;
                                    unlock_ms <= 12'd3000;
                                    state <= 8'hF0;
                                end else begin
                                    poll_ctr <= poll_ctr + 1'b1;
                                    if (poll_ctr == 16'hFFFF) begin
                                        hard_fault <= 1'b1;
                                        state <= 8'hFF;
                                    end else begin
                                        apdu_state <= APDU_AUTH_POLL;
                                    end
                                end
                            end
                        end

                        APDU_AUTH_ERR: begin
                            if (op_step == 3'd0) begin
                                start_read(REG_Error_R);
                            end
                            if (op_done) begin
                                errreg <= tmp_reg;
                                if ((tmp_reg & 8'h13) != 8'h00) begin
                                    hard_fault <= 1'b1;
                                    unlock_ms <= 12'd3000;
                                    state <= 8'hF0;
                                end else begin
                                    apdu_state <= APDU_AUTH_LEN;
                                end
                            end
                        end

                        APDU_AUTH_LEN: begin
                            if (op_step == 3'd0) begin
                                start_read(REG_FIFOLevel_R);
                            end
                            if (op_done) begin
                                fifolvl <= tmp_reg;
                                if (tmp_reg < 8'd18) begin
                                    hard_fault <= 1'b1;
                                    unlock_ms <= 12'd3000;
                                    state <= 8'hF0;
                                end else begin
                                    resp_idx <= 5'd0;
                                    auth_pattern <= 128'h0;
                                    pattern_ok <= 1'b1;
                                    apdu_state <= APDU_AUTH_READ;
                                end
                            end
                        end

                        APDU_AUTH_READ: begin
                            if (op_step == 3'd0) begin
                                start_read(REG_FIFOData_R);
                            end
                            if (op_done) begin
                                if (resp_idx < 5'd16) begin
                                    auth_pattern <= {auth_pattern[119:0], tmp_reg};
                                    if (tmp_reg != 8'hAA)
                                        pattern_ok <= 1'b0;
                                end else if (resp_idx == 5'd16) begin
                                    auth_sw1 <= tmp_reg;
                                end else begin
                                    auth_sw2 <= tmp_reg;
                                end

                                if (resp_idx == 5'd17) begin
                                    apdu_state <= APDU_AUTH_STATUS;
                                end else begin
                                    resp_idx <= resp_idx + 1'b1;
                                end
                            end
                        end

                        APDU_AUTH_STATUS: begin
                            if (auth_sw1 == 8'h90 && auth_sw2 == 8'h00 && pattern_ok) begin
                                unlock <= 1'b1;
                                busy <= 1'b0;
                                unlock_ms <= 12'd3000;
                                state <= 8'hEF;
                            end else begin
                                unlock <= 1'b0;
                                busy <= 1'b0;
                                hard_fault <= 1'b1;
                                unlock_ms <= 12'd3000;
                                state <= 8'hF0;
                            end
                        end

                        default: apdu_state <= APDU_CFG_TXMODE;
                    endcase
                end

                // hold unlocked (you can change this to pulse if you want)
                8'hEF: begin
                  busy <= 1'b0;

                  // countdown in ms using the ms_tick we already added
                  if (ms_tick) begin
                    if (unlock_ms != 0)
                      unlock_ms <= unlock_ms - 1'b1;
                    else begin
                      unlock <= 1'b0;
                      busy <= 1'b1;
                      card_seen <= 1'b0;
                      state <= 8'h30;     // zurück in den Arduino-Loop-Pfad
                    end
                  end
                                end

                                // FAULT state: 3s Fault, dann zurück in Loop
                                8'hF0: begin
                                    busy <= 1'b0;
                                    if (ms_tick) begin
                                        if (unlock_ms != 0)
                                            unlock_ms <= unlock_ms - 1'b1;
                                        else begin
                                            hard_fault <= 1'b0;
                                            busy <= 1'b1;
                                            card_seen <= 1'b0;
                                            state <= 8'h30;
                                        end
                                    end
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
