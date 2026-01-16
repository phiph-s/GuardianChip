final: prev: {
  yosys-eqy = prev.yosys-eqy.overrideAttrs (old: {
    src = prev.fetchFromGitHub {
      owner = "yosyshq";
      repo  = "eqy";
      rev   = "v0.60";
      hash  = "sha256-k/sQSnxiVJVeZBG0bg+X5gtK6M3/p64slRArMH7MIEw=";
    };
  });
}
