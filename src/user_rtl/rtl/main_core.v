// Main Core Module - Guardian Chip Top Level
// Integrates all components for LAYR Authentication System

module main_core #(
  parameter UNLOCK_DURATION_PARAM = 32'd500000000 // 5 seconds at 100MHz (default)
)(
  // System signals
  input  logic         clk,
  input  logic         rst_n,
  
  // MFRC522 IRQ input (card detection)
  input  logic         nfc_irq,
  
  // SPI interface to MFRC522 NFC Reader
  output logic         nfc_spi_cs_n,
  output logic         nfc_spi_sclk,
  output logic         nfc_spi_mosi,
  input  logic         nfc_spi_miso,
  
  // SPI interface to AT25010 EEPROM
  output logic         eeprom_spi_cs_n,
  output logic         eeprom_spi_sclk,
  output logic         eeprom_spi_mosi,
  input  logic         eeprom_spi_miso,
  
  // Door lock control
  output logic         door_unlock,
  
  // Status indicators
  output logic         status_unlock,    // Green LED - door unlocked
  output logic         status_fault,     // Red LED - authentication failed
  output logic         status_busy       // Yellow LED - busy authenticating
);

  // Authentication timeout (in clock cycles)
  localparam TIMEOUT_CYCLES = 32'd100000000; // 1 second at 100MHz
  
  // ============================================
  // Internal signals
  // ============================================
  
  // Card Detector signals
  logic         card_detected;
  logic [31:0]  card_uid;
  logic         card_ready;
  logic         detector_start_auth;
  logic         detection_error;
  logic [7:0]   error_code;
  
  // NFC signals - shared between detector and auth controller
  logic         det_nfc_cmd_valid;
  logic         det_nfc_cmd_write;
  logic [5:0]   det_nfc_cmd_addr;
  logic [7:0]   det_nfc_cmd_wdata;
  logic         auth_nfc_cmd_valid;
  logic         auth_nfc_cmd_write;
  logic [5:0]   auth_nfc_cmd_addr;
  logic [7:0]   auth_nfc_cmd_wdata;
  
  // AuthController signals
  logic         auth_start;
  logic         auth_success;
  logic         auth_failed;
  logic         auth_busy;
  logic [127:0] card_id;
  logic         card_id_valid;
  
  // AES Core signals
  logic         aes_start;
  logic         aes_mode;
  logic [127:0] aes_key;
  logic [127:0] aes_block_in;
  logic [127:0] aes_block_out;
  logic         aes_done;
  
  // Key Storage (EEPROM) signals
  logic         key_load_req;
  logic [6:0]   key_addr;
  logic [7:0]   key_data;
  logic         key_data_valid;
  
  // EEPROM interface signals
  logic         eeprom_cmd_valid;
  logic         eeprom_cmd_ready;
  logic [2:0]   eeprom_cmd_type;
  logic [6:0]   eeprom_cmd_addr;
  logic [7:0]   eeprom_cmd_wdata;
  logic [7:0]   eeprom_cmd_rdata;
  logic         eeprom_cmd_done;
  logic         eeprom_cmd_error;
  
  // Nonce Generator signals
  logic         nonce_req;
  logic [63:0]  nonce;
  logic         nonce_valid;
  
  // NFC Interface signals (shared)
  logic         nfc_cmd_valid;
  logic         nfc_cmd_ready;
  logic         nfc_cmd_write;
  logic [5:0]   nfc_cmd_addr;
  logic [7:0]   nfc_cmd_wdata;
  logic [7:0]   nfc_cmd_rdata;
  logic         nfc_cmd_done;
  
  // NFC arbiter - mux between detector and auth controller
  always_comb begin
    if (!auth_busy && (card_detected || det_nfc_cmd_valid) && !card_ready) begin
      // Detector has priority during detection phase
      nfc_cmd_valid = det_nfc_cmd_valid;
      nfc_cmd_write = det_nfc_cmd_write;
      nfc_cmd_addr  = det_nfc_cmd_addr;
      nfc_cmd_wdata = det_nfc_cmd_wdata;
    end else begin
      // Auth controller has priority during authentication
      nfc_cmd_valid = auth_nfc_cmd_valid;
      nfc_cmd_write = auth_nfc_cmd_write;
      nfc_cmd_addr  = auth_nfc_cmd_addr;
      nfc_cmd_wdata = auth_nfc_cmd_wdata;
    end
  end
  
  // Timer/Watchdog
  logic         timeout_start;
  logic         timeout_occurred;
  logic [31:0]  timeout_counter;
  
  // Door unlock control
  logic         door_unlock_reg;
  logic [31:0]  unlock_timer;
  localparam    UNLOCK_DURATION = UNLOCK_DURATION_PARAM;
  
  // ============================================
  // Component instantiations
  // ============================================
  
  // NFC Card Detector - handles ISO14443A card detection
  nfc_card_detector u_card_detector (
    .clk              (clk),
    .rst_n            (rst_n),
    .nfc_irq          (nfc_irq),
    .card_detected    (card_detected),
    .card_uid         (card_uid),
    .card_ready       (card_ready),
    .start_auth       (detector_start_auth),
    .nfc_cmd_valid    (det_nfc_cmd_valid),
    .nfc_cmd_ready    (nfc_cmd_ready),
    .nfc_cmd_write    (det_nfc_cmd_write),
    .nfc_cmd_addr     (det_nfc_cmd_addr),
    .nfc_cmd_wdata    (det_nfc_cmd_wdata),
    .nfc_cmd_rdata    (nfc_cmd_rdata),
    .nfc_cmd_done     (nfc_cmd_done),
    .detection_error  (detection_error),
    .error_code       (error_code)
  );
  
  // Start authentication when detector signals card is ready
  assign auth_start = detector_start_auth;
  
  // Authentication Controller
  auth_controller u_auth_controller (
    .clk              (clk),
    .rst_n            (rst_n),
    .start_auth       (auth_start),
    .auth_success     (auth_success),
    .auth_failed      (auth_failed),
    .auth_busy        (auth_busy),
    .card_id          (card_id),
    .card_id_valid    (card_id_valid),
    .aes_start        (aes_start),
    .aes_mode         (aes_mode),
    .aes_key          (aes_key),
    .aes_block_in     (aes_block_in),
    .aes_block_out    (aes_block_out),
    .aes_done         (aes_done),
    .key_load_req     (key_load_req),
    .key_addr         (key_addr),
    .key_data         (key_data),
    .key_data_valid   (key_data_valid),
    .nonce_req        (nonce_req),
    .nonce            (nonce),
    .nonce_valid      (nonce_valid),
    .nfc_cmd_valid    (auth_nfc_cmd_valid),
    .nfc_cmd_ready    (nfc_cmd_ready),
    .nfc_cmd_write    (auth_nfc_cmd_write),
    .nfc_cmd_addr     (auth_nfc_cmd_addr),
    .nfc_cmd_wdata    (auth_nfc_cmd_wdata),
    .nfc_cmd_rdata    (nfc_cmd_rdata),
    .nfc_cmd_done     (nfc_cmd_done),
    .timeout_start    (timeout_start),
    .timeout_occurred (timeout_occurred)
  );
  
  // AES Core (with encrypt/decrypt support)
  aes_core u_aes_core (
    .clk              (clk),
    .rst_n            (rst_n),
    .start            (aes_start),
    .mode             (aes_mode),
    .key              (aes_key),
    .block_in         (aes_block_in),
    .block_out        (aes_block_out),
    .done             (aes_done)
  );
  
  // Nonce Generator
  nonce_generator u_nonce_gen (
    .clk              (clk),
    .rst_n            (rst_n),
    .req              (nonce_req),
    .valid            (nonce_valid),
    .nonce            (nonce)
  );
  
  // AT25010 EEPROM Interface
  at25010_interface #(
    .CLKS_PER_HALF_BIT (2),
    .MAX_BYTES_PER_CS  (3),
    .CS_INACTIVE_CLKS  (10)
  ) u_eeprom (
    .clk              (clk),
    .rst_n            (rst_n),
    .cmd_valid        (eeprom_cmd_valid),
    .cmd_ready        (eeprom_cmd_ready),
    .cmd_type         (eeprom_cmd_type),
    .cmd_addr         (eeprom_cmd_addr),
    .cmd_wdata        (eeprom_cmd_wdata),
    .cmd_rdata        (eeprom_cmd_rdata),
    .cmd_done         (eeprom_cmd_done),
    .cmd_error        (eeprom_cmd_error),
    .spi_cs_n         (eeprom_spi_cs_n),
    .spi_sclk         (eeprom_spi_sclk),
    .spi_mosi         (eeprom_spi_mosi),
    .spi_miso         (eeprom_spi_miso)
  );
  
  // MFRC522 NFC Interface
  mfrc522_interface #(
    .CLKS_PER_HALF_BIT (2),
    .MAX_BYTES_PER_CS  (2),
    .CS_INACTIVE_CLKS  (10)
  ) u_nfc (
    .clk              (clk),
    .rst_n            (rst_n),
    .cmd_valid        (nfc_cmd_valid),
    .cmd_ready        (nfc_cmd_ready),
    .cmd_is_write     (nfc_cmd_write),
    .cmd_addr         (nfc_cmd_addr),
    .cmd_wdata        (nfc_cmd_wdata),
    .cmd_rdata        (nfc_cmd_rdata),
    .cmd_done         (nfc_cmd_done),
    .spi_cs_n         (nfc_spi_cs_n),
    .spi_sclk         (nfc_spi_sclk),
    .spi_mosi         (nfc_spi_mosi),
    .spi_miso         (nfc_spi_miso)
  );
  
  // ============================================
  // Key Storage Interface Logic
  // ============================================
  
  typedef enum logic [1:0] {
    KEY_IDLE,
    KEY_READ_START,
    KEY_READ_WAIT,
    KEY_READ_DONE
  } key_state_t;
  
  key_state_t key_state;
  
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      key_state <= KEY_IDLE;
      key_data <= 8'h0;
      key_data_valid <= 1'b0;
      eeprom_cmd_valid <= 1'b0;
      eeprom_cmd_type <= 3'b0;
      eeprom_cmd_addr <= 7'h0;
      eeprom_cmd_wdata <= 8'h0;
    end else begin
      key_data_valid <= 1'b0;
      
      case (key_state)
        KEY_IDLE: begin
          if (key_load_req) begin
            key_state <= KEY_READ_START;
            eeprom_cmd_type <= 3'b100;  // CMD_READ
            eeprom_cmd_addr <= key_addr;
          end
        end
        
        KEY_READ_START: begin
          if (!eeprom_cmd_valid) begin
            eeprom_cmd_valid <= 1'b1;
          end else if (eeprom_cmd_ready) begin
            key_state <= KEY_READ_WAIT;
            eeprom_cmd_valid <= 1'b0;
          end
        end
        
        KEY_READ_WAIT: begin
          if (eeprom_cmd_done) begin
            key_data <= eeprom_cmd_rdata;
            key_data_valid <= 1'b1;
            key_state <= KEY_READ_DONE;
          end
        end
        
        KEY_READ_DONE: begin
          key_state <= KEY_IDLE;
        end
      endcase
    end
  end
  
  // Note: Authentication start is now controlled by nfc_card_detector
  // The detector triggers auth_start after successful card detection
  
  // ============================================
  // Timeout Watchdog
  // ============================================
  
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      timeout_counter <= 32'h0;
      timeout_occurred <= 1'b0;
    end else begin
      if (timeout_start) begin
        timeout_counter <= TIMEOUT_CYCLES;
        timeout_occurred <= 1'b0;
      end else if (timeout_counter > 0) begin
        timeout_counter <= timeout_counter - 1;
        if (timeout_counter == 1) begin
          timeout_occurred <= 1'b1;
        end
      end else begin
        timeout_occurred <= 1'b0;
      end
    end
  end
  
  // ============================================
  // Door Unlock Control
  // ============================================
  
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      door_unlock_reg <= 1'b0;
      unlock_timer <= 32'h0;
    end else begin
      if (auth_success && card_id_valid) begin
        // TODO: Check card_id against authorized list in EEPROM
        // For now, unlock on any successful authentication
        door_unlock_reg <= 1'b1;
        unlock_timer <= UNLOCK_DURATION;
      end else if (unlock_timer > 0) begin
        unlock_timer <= unlock_timer - 1;
        if (unlock_timer == 1) begin
          door_unlock_reg <= 1'b0;
        end
      end
    end
  end
  
  assign door_unlock = door_unlock_reg;
  
  // ============================================
  // Status LED Control
  // ============================================
  
  assign status_unlock = door_unlock_reg;
  assign status_fault  = auth_failed;
  assign status_busy   = auth_busy;

endmodule
