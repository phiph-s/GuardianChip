module aes_core (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,        // Start encryption/decryption
  input  logic         mode,         // 0=encrypt, 1=decrypt
  input  logic [127:0] key,
  input  logic [127:0] block_in,
  output logic [127:0] block_out,
  output logic         done          // Operation complete
);

  logic [127:0] encrypt_out;
  logic [127:0] decrypt_out;
  logic         done_reg;

  // Instantiate AES Encrypt
  AES_Encrypt u_aes_encrypt (
    .in       (block_in),
    .key      (key),
    .out      (encrypt_out)
  );

  // Instantiate AES Decrypt
  AES_Decrypt u_aes_decrypt (
    .in       (block_in),
    .key      (key),
    .out      (decrypt_out)
  );

  // Select output based on mode
  assign block_out = mode ? decrypt_out : encrypt_out;

  // Simple done signal generation (combinational AES needs 1 cycle)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      done_reg <= 1'b0;
    end else begin
      done_reg <= start;
    end
  end

  assign done = done_reg;

endmodule
