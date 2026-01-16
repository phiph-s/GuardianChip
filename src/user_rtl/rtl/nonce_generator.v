module nonce_generator (
  input  logic clk,
  input  logic rst_n,
  input  logic req,
  output logic valid,
  output logic [63:0] nonce
);

  logic [63:0] lfsr;
  logic [31:0] free_counter;

  // Free running counter adds jitter-based entropy  
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      free_counter <= 32'h12345678;
    else
      free_counter <= free_counter + 1;
  end

  // LFSR as above
  function automatic [63:0] lfsr_step(input [63:0] x);
    lfsr_step = { x[62:0], x[0] ^ x[1] ^ x[3] ^ x[4] };
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      lfsr   <= 64'hCAFEBEEF12345678;
      valid  <= 1'b0;
    end else begin
      valid <= 1'b0;

      if (req) begin
        lfsr  <= lfsr_step(lfsr);
        valid <= 1'b1;
      end
    end
  end

  assign nonce = lfsr ^ {32'h0, free_counter};

endmodule
