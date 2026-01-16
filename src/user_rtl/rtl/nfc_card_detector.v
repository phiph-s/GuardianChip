// NFC Card Detector Module
// Implements ISO14443A card detection and selection
// Detects card presence and reads UID before triggering authentication

module nfc_card_detector (
  input  logic         clk,
  input  logic         rst_n,
  
  // MFRC522 interrupt input
  input  logic         nfc_irq,           // Interrupt from MFRC522 (card detected)
  
  // Card detection outputs
  output logic         card_detected,     // Card is present
  output logic [31:0]  card_uid,          // 4-byte UID (most common)
  output logic         card_ready,        // Card selected and ready
  
  // To auth_controller
  output logic         start_auth,        // Trigger authentication
  
  // NFC Interface (MFRC522)
  output logic         nfc_cmd_valid,
  input  logic         nfc_cmd_ready,
  output logic         nfc_cmd_write,
  output logic [5:0]   nfc_cmd_addr,
  output logic [7:0]   nfc_cmd_wdata,
  input  logic [7:0]   nfc_cmd_rdata,
  input  logic         nfc_cmd_done,
  
  // Status/Error outputs
  output logic         detection_error,   // Error during detection
  output logic [7:0]   error_code         // Error code for debugging
);

  // MFRC522 Registers
  localparam [5:0] REG_COMMAND    = 6'h01;
  localparam [5:0] REG_COMIRQ     = 6'h04;
  localparam [5:0] REG_FIFODATA   = 6'h09;
  localparam [5:0] REG_FIFOLEVEL  = 6'h0A;
  localparam [5:0] REG_BITFRAMING = 6'h0D;
  localparam [5:0] REG_TXMODE     = 6'h12;
  localparam [5:0] REG_RXMODE     = 6'h13;
  
  // MFRC522 Commands
  localparam [7:0] PCD_IDLE       = 8'h00;
  localparam [7:0] PCD_TRANSCEIVE = 8'h0C;

  // ISO14443A Commands
  localparam [7:0] CMD_REQA     = 8'h26;  // Request Type A
  localparam [7:0] CMD_WUPA     = 8'h52;  // Wake-Up Type A
  localparam [7:0] CMD_ANTICOLL = 8'h93;  // Anti-collision CL1
  localparam [7:0] CMD_SELECT   = 8'h93;  // Select CL1
  
  // Expected responses
  localparam [15:0] ATQA_MIFARE = 16'h0004;  // Typical ATQA response
  
  // State machine
  typedef enum logic [4:0] {
    ST_IDLE,
    ST_WAIT_IRQ,
    
    // Generic Transaction States
    ST_CLEAR_IRQ,       // New: Clear interrupts
    ST_FLUSH_FIFO,      // New: Flush FIFO
    ST_CONFIG_CRC,      // New: Configure Tx CRC
    ST_CONFIG_RX_CRC,   // New: Configure Rx CRC
    ST_TX_FIFO,
    ST_TX_CMD,
    ST_TX_FRAMING,
    ST_POLL_IRQ,
    ST_READ_FIFO_LEVEL,
    ST_READ_FIFO_DATA,
    
    // Protocol Decision States
    ST_CHECK_ATQA,
    ST_CHECK_UID,
    ST_CHECK_SAK,
    
    ST_CARD_READY,
    ST_ERROR
  } state_t;
  
  typedef enum logic [2:0] {
    PROT_IDLE,
    PROT_REQA,
    PROT_ANTICOLL,
    PROT_SELECT
  } protocol_t;
  
  state_t state, next_state;
  protocol_t protocol_state, next_protocol_state;
  
  // Internal registers
  logic [31:0] uid_buffer;
  logic [15:0] atqa_response;
  logic [7:0]  sak_response;
  logic [3:0]  retry_count;
  logic        command_sent;
  logic        irq_detected;
  
  // Transaction buffers
  logic [7:0]  tx_buffer [0:15];
  logic [3:0]  tx_length;
  logic [3:0]  tx_index;
  logic [7:0]  rx_buffer [0:15];
  logic [3:0]  rx_count;
  logic [3:0]  rx_index;
  logic [7:0]  framing_bits;
  
  // State machine - sequential logic
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= ST_IDLE;
      protocol_state <= PROT_IDLE;
    end else begin
      state <= next_state;
      protocol_state <= next_protocol_state;
    end
  end
  
  // State machine - combinational logic
  always_comb begin
    next_state = state;
    next_protocol_state = protocol_state;
    
    case (state)
      ST_IDLE: begin
        if (irq_detected) begin
            next_state = ST_CLEAR_IRQ;
            next_protocol_state = PROT_REQA;
        end
      end
      
      ST_WAIT_IRQ: begin
        if (irq_detected) begin
            next_state = ST_CLEAR_IRQ;
            next_protocol_state = PROT_REQA;
        end
      end
      
      // --- Generic Transaction Sequence ---
      ST_CLEAR_IRQ: begin
        if (nfc_cmd_done) next_state = ST_FLUSH_FIFO;
      end
      
      ST_FLUSH_FIFO: begin
        if (nfc_cmd_done) next_state = ST_CONFIG_CRC;
      end
      
      ST_CONFIG_CRC: begin
        if (nfc_cmd_done) next_state = ST_CONFIG_RX_CRC;
      end

      ST_CONFIG_RX_CRC: begin
        if (nfc_cmd_done) next_state = ST_TX_FIFO;
      end
      
      ST_TX_FIFO: begin
        if (nfc_cmd_ready && !command_sent) begin
            // Wait for write to complete
        end else if (nfc_cmd_done) begin
            if (tx_index < tx_length - 1)
                next_state = ST_TX_FIFO; // Send next byte
            else
                next_state = ST_TX_CMD;
        end
      end
      
      ST_TX_CMD: begin
        if (nfc_cmd_done) next_state = ST_TX_FRAMING;
      end
      
      ST_TX_FRAMING: begin
        if (nfc_cmd_done) next_state = ST_POLL_IRQ;
      end
      
      ST_POLL_IRQ: begin
        if (nfc_cmd_done) begin
            // Check RxIRq bit (0x20)
            if (nfc_cmd_rdata[5]) 
                next_state = ST_READ_FIFO_LEVEL;
            else
                next_state = ST_POLL_IRQ; // Keep polling
        end
      end
      
      ST_READ_FIFO_LEVEL: begin
        if (nfc_cmd_done) next_state = ST_READ_FIFO_DATA;
      end
      
      ST_READ_FIFO_DATA: begin
        if (nfc_cmd_done) begin
            if (rx_index < rx_count - 1)
                next_state = ST_READ_FIFO_DATA;
            else begin
                // Transaction complete, decide next based on protocol
                case (protocol_state)
                    PROT_REQA:     next_state = ST_CHECK_ATQA;
                    PROT_ANTICOLL: next_state = ST_CHECK_UID;
                    PROT_SELECT:   next_state = ST_CHECK_SAK;
                    default:       next_state = ST_IDLE;
                endcase
            end
        end
      end
      
      // --- Protocol Logic ---
      ST_CHECK_ATQA: begin
        if (atqa_response == ATQA_MIFARE) begin
          next_state = ST_TX_FIFO;
          next_protocol_state = PROT_ANTICOLL;
        end else if (retry_count < 3) begin
          next_state = ST_TX_FIFO; // Retry REQA
          next_protocol_state = PROT_REQA;
        end else
          next_state = ST_ERROR;
      end
      
      ST_CHECK_UID: begin
        next_state = ST_TX_FIFO;
        next_protocol_state = PROT_SELECT;
      end
      
      ST_CHECK_SAK: begin
        if (sak_response[2] == 1'b0)
          next_state = ST_CARD_READY;
        else
          next_state = ST_ERROR;
      end
      
      ST_CARD_READY: begin
        next_state = ST_IDLE;
        next_protocol_state = PROT_IDLE;
      end
      
      ST_ERROR: begin
        next_state = ST_IDLE;
        next_protocol_state = PROT_IDLE;
      end
    endcase
  end
  
  // IRQ edge detection
  logic irq_prev;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      irq_prev <= 1'b0;
      irq_detected <= 1'b0;
    end else begin
      irq_prev <= nfc_irq;
      if (nfc_irq && !irq_prev) begin
        irq_detected <= 1'b1;
        `ifdef SIM
        $display("[%0t] [NFC_DETECTOR] Card detected (IRQ triggered)", $time);
        `endif
      end
      if (state == ST_TX_FIFO && irq_detected) begin
        irq_detected <= 1'b0;
      end
    end
  end
  
  // Control logic and datapath
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      card_detected <= 1'b0;
      card_uid <= 32'h0;
      card_ready <= 1'b0;
      start_auth <= 1'b0;
      detection_error <= 1'b0;
      error_code <= 8'h00;
      
      uid_buffer <= 32'h0;
      atqa_response <= 16'h0;
      sak_response <= 8'h0;
      retry_count <= 4'h0;
      command_sent <= 1'b0;
      
      nfc_cmd_valid <= 1'b0;
      nfc_cmd_write <= 1'b0;
      nfc_cmd_addr <= 6'h0;
      nfc_cmd_wdata <= 8'h0;
      
      tx_length <= 0;
      tx_index <= 0;
      rx_count <= 0;
      rx_index <= 0;
      framing_bits <= 0;
      
    end else begin
      // Default: clear single-cycle signals
      nfc_cmd_valid <= 1'b0;
      start_auth <= 1'b0;
      
      if (state != next_state) begin
        command_sent <= 1'b0;
        // Reset indices when entering new state
        if (next_state == ST_TX_FIFO) tx_index <= 0;
        if (next_state == ST_READ_FIFO_DATA) rx_index <= 0;
      end
      
      case (state)
        ST_IDLE: begin
          if (card_ready || card_detected) begin
            `ifdef SIM
            $display("[%0t] [NFC_DETECTOR] Back to IDLE, ready for next card", $time);
            `endif
          end
          card_detected <= 1'b0;
          card_ready <= 1'b0;
          detection_error <= 1'b0;
          retry_count <= 4'h0;
        end
        
        // --- Generic Transaction Execution ---
        ST_CLEAR_IRQ: begin
            if (!command_sent && nfc_cmd_ready) begin
                nfc_cmd_valid <= 1'b1;
                nfc_cmd_write <= 1'b1;
                nfc_cmd_addr <= REG_COMIRQ;
                nfc_cmd_wdata <= 8'h7F; // Clear all interrupts
                command_sent <= 1'b1;
            end else if (nfc_cmd_done) begin
                command_sent <= 1'b0;
            end
        end
        
        ST_FLUSH_FIFO: begin
            if (!command_sent && nfc_cmd_ready) begin
                nfc_cmd_valid <= 1'b1;
                nfc_cmd_write <= 1'b1;
                nfc_cmd_addr <= REG_FIFOLEVEL;
                nfc_cmd_wdata <= 8'h80; // FlushBuffer
                command_sent <= 1'b1;
            end else if (nfc_cmd_done) begin
                command_sent <= 1'b0;
            end
        end
        
        ST_CONFIG_CRC: begin
            if (!command_sent && nfc_cmd_ready) begin
                nfc_cmd_valid <= 1'b1;
                nfc_cmd_write <= 1'b1;
                nfc_cmd_addr <= REG_TXMODE;
                // For SELECT, we need CRC. For others (REQA, ANTICOLL), we don't.
                if (protocol_state == PROT_SELECT)
                    nfc_cmd_wdata <= 8'h80; // TxCRCEn
                else
                    nfc_cmd_wdata <= 8'h00; // Disable CRC
                command_sent <= 1'b1;
            end else if (nfc_cmd_done) begin
                command_sent <= 1'b0;
            end
        end

        ST_CONFIG_RX_CRC: begin
            if (!command_sent && nfc_cmd_ready) begin
                nfc_cmd_valid <= 1'b1;
                nfc_cmd_write <= 1'b1;
                nfc_cmd_addr <= REG_RXMODE;
                // Symmetric to TxMode for our supported protocols
                if (protocol_state == PROT_SELECT)
                    nfc_cmd_wdata <= 8'h80; // RxCRCEn
                else
                    nfc_cmd_wdata <= 8'h00; // Disable CRC
                command_sent <= 1'b1;
            end else if (nfc_cmd_done) begin
                command_sent <= 1'b0;
            end
        end
        
        ST_TX_FIFO: begin
            // Setup Protocol Parameters (One-shot at start of state)
            if (tx_index == 0 && !command_sent) begin
                case (protocol_state)
                    PROT_REQA: begin
                        tx_length <= 1;
                        framing_bits <= 8'h87; // 7 bits
                        `ifdef SIM
                        $display("[%0t] [NFC_DETECTOR] → REQA", $time);
                        `endif
                    end
                    PROT_ANTICOLL: begin
                        tx_length <= 2;
                        framing_bits <= 8'h80; // 8 bits
                        `ifdef SIM
                        $display("[%0t] [NFC_DETECTOR] → ANTICOLL", $time);
                        `endif
                    end
                    PROT_SELECT: begin
                        tx_length <= 7; // 9 bytes total - 2 CRC = 7 bytes data
                        framing_bits <= 8'h80;
                        `ifdef SIM
                        $display("[%0t] [NFC_DETECTOR] → SELECT", $time);
                        `endif
                    end
                endcase
            end

            if (!command_sent && nfc_cmd_ready) begin
                nfc_cmd_valid <= 1'b1;
                nfc_cmd_write <= 1'b1;
                nfc_cmd_addr <= REG_FIFODATA;
                
                // Direct Data Mux (Avoids buffer latency)
                case (protocol_state)
                    PROT_REQA: nfc_cmd_wdata <= CMD_REQA;
                    PROT_ANTICOLL: nfc_cmd_wdata <= (tx_index == 0) ? CMD_ANTICOLL : 8'h20;
                    PROT_SELECT: begin
                        case (tx_index)
                            0: nfc_cmd_wdata <= CMD_SELECT;
                            1: nfc_cmd_wdata <= 8'h70;
                            2: nfc_cmd_wdata <= uid_buffer[31:24];
                            3: nfc_cmd_wdata <= uid_buffer[23:16];
                            4: nfc_cmd_wdata <= uid_buffer[15:8];
                            5: nfc_cmd_wdata <= uid_buffer[7:0];
                            6: nfc_cmd_wdata <= uid_buffer[31:24] ^ uid_buffer[23:16] ^ uid_buffer[15:8] ^ uid_buffer[7:0];
                            default: nfc_cmd_wdata <= 8'h00; // Should not happen with length 7
                        endcase
                    end
                    default: nfc_cmd_wdata <= 8'h00;
                endcase
                
                command_sent <= 1'b1;
            end else if (nfc_cmd_done) begin
                command_sent <= 1'b0;
                if (tx_index < tx_length - 1) tx_index <= tx_index + 1;
            end
        end
        
        ST_TX_CMD: begin
            if (!command_sent && nfc_cmd_ready) begin
                nfc_cmd_valid <= 1'b1;
                nfc_cmd_write <= 1'b1;
                nfc_cmd_addr <= REG_COMMAND;
                nfc_cmd_wdata <= PCD_TRANSCEIVE;
                command_sent <= 1'b1;
            end else if (nfc_cmd_done) begin
                command_sent <= 1'b0;
            end
        end
        
        ST_TX_FRAMING: begin
            if (!command_sent && nfc_cmd_ready) begin
                nfc_cmd_valid <= 1'b1;
                nfc_cmd_write <= 1'b1;
                nfc_cmd_addr <= REG_BITFRAMING;
                nfc_cmd_wdata <= framing_bits;
                command_sent <= 1'b1;
            end else if (nfc_cmd_done) begin
                command_sent <= 1'b0;
            end
        end
        
        ST_POLL_IRQ: begin
            if (!command_sent && nfc_cmd_ready) begin
                nfc_cmd_valid <= 1'b1;
                nfc_cmd_write <= 1'b0; // Read
                nfc_cmd_addr <= REG_COMIRQ;
                command_sent <= 1'b1;
            end else if (nfc_cmd_done) begin
                command_sent <= 1'b0; // Allow re-polling
            end
        end
        
        ST_READ_FIFO_LEVEL: begin
            if (!command_sent && nfc_cmd_ready) begin
                nfc_cmd_valid <= 1'b1;
                nfc_cmd_write <= 1'b0;
                nfc_cmd_addr <= REG_FIFOLEVEL;
                command_sent <= 1'b1;
            end else if (nfc_cmd_done) begin
                rx_count <= nfc_cmd_rdata[3:0]; // Assume max 16 bytes
                command_sent <= 1'b0;
            end
        end
        
        ST_READ_FIFO_DATA: begin
            if (!command_sent && nfc_cmd_ready) begin
                nfc_cmd_valid <= 1'b1;
                nfc_cmd_write <= 1'b0;
                nfc_cmd_addr <= REG_FIFODATA;
                command_sent <= 1'b1;
            end else if (nfc_cmd_done) begin
                command_sent <= 1'b0;
                rx_buffer[rx_index] <= nfc_cmd_rdata;
                if (rx_index < rx_count - 1) rx_index <= rx_index + 1;
                
                // Store data based on protocol
                if (protocol_state == PROT_REQA) begin
                    if (rx_index == 0) atqa_response[7:0] <= nfc_cmd_rdata;
                    if (rx_index == 1) atqa_response[15:8] <= nfc_cmd_rdata;
                end else if (protocol_state == PROT_ANTICOLL) begin
                    if (rx_index < 4) uid_buffer[{rx_index[1:0], 3'b000} +: 8] <= nfc_cmd_rdata;
                end else if (protocol_state == PROT_SELECT) begin
                    if (rx_index == 0) sak_response <= nfc_cmd_rdata;
                end
            end
        end
        
        // --- Protocol Checks ---
        ST_CHECK_ATQA: begin
          if (atqa_response == ATQA_MIFARE) begin
            `ifdef SIM
            $display("[%0t] [NFC_DETECTOR] ← ATQA: %h (valid)", $time, atqa_response);
            `endif
            card_detected <= 1'b1;
            retry_count <= 4'h0;
          end else begin
            `ifdef SIM
            $display("[%0t] [NFC_DETECTOR] ← ATQA: %h (invalid, retry)", $time, atqa_response);
            `endif
            retry_count <= retry_count + 1;
          end
        end
        
        ST_CHECK_UID: begin
          `ifdef SIM
          $display("[%0t] [NFC_DETECTOR] ← UID: %h", $time, uid_buffer);
          `endif
        end
        
        ST_CHECK_SAK: begin
          `ifdef SIM
          $display("[%0t] [NFC_DETECTOR] ← SAK: %h", $time, sak_response);
          `endif
          if (sak_response[2] == 1'b0) begin
            `ifdef SIM
            $display("[%0t] [NFC_DETECTOR] ✓ Card selected successfully", $time);
            `endif
          end else begin
            `ifdef SIM
            $display("[%0t] [NFC_DETECTOR] ✗ Cascade required (not supported)", $time);
            `endif
            detection_error <= 1'b1;
            error_code <= 8'h01;
          end
        end
        
        ST_CARD_READY: begin
          if (!card_ready) begin
            card_ready <= 1'b1;
            card_uid <= uid_buffer;
            start_auth <= 1'b1;
            `ifdef SIM
            $display("[%0t] [NFC_DETECTOR] Card ready → Starting authentication", $time);
            `endif
          end
        end
        
        ST_ERROR: begin
          detection_error <= 1'b1;
          if (error_code == 8'h00) begin
            error_code <= 8'hFF;
            `ifdef SIM
            $display("[%0t] [NFC_DETECTOR] ✗ Detection error", $time);
            `endif
          end
        end
      endcase
    end
  end

endmodule

