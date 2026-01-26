TOP = ulx3s_test_top
OUT = ulx3s_rc522_min

DEVICE  = 85k
PACKAGE = CABGA381
FREQ    = 25
LPF     = ulx3s.lpf

RTL = \
  ulx3s_test_top.v \
  ../src/user_rtl/rtl/nfc_card_detector.v \
  ../src/user_rtl/rtl/spi_master.v

SPIIP = \
  ../src/user_rtl/ip/verilog_spi/spi_module.v \
  ../src/user_rtl/ip/verilog_spi/clock_divider.v \
  ../src/user_rtl/ip/verilog_spi/pos_edge_det.v \
  ../src/user_rtl/ip/verilog_spi/neg_edge_det.v

all: $(OUT).bit

$(OUT).json: $(RTL) $(SPIIP)
	yosys -p "read_verilog -sv $(RTL) $(SPIIP); synth_ecp5 -top $(TOP) -json $@"

$(OUT).config: $(OUT).json $(LPF)
	nextpnr-ecp5 --$(DEVICE) --package $(PACKAGE) \
		--json $(OUT).json --lpf $(LPF) --textcfg $@ --freq $(FREQ)

$(OUT).bit: $(OUT).config
	ecppack --compress $< $@

prog: $(OUT).bit
	sudo fujprog $<

clean:
	rm -f *.json *.config *.bit

.PHONY: all clean prog
