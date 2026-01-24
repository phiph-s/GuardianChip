# ULX3S FPGA Build for MFRC522 Testing
# Toolchain: Yosys, nextpnr-ecp5, ecppack, fujprog

TOP_MODULE = ulx3s_test_top
SOURCE = mfrc_debug.v
LPF = ulx3s.lpf

# ECP5-85K device
DEVICE = 85k
PACKAGE = CABGA381
FREQ = 25

all: mfrc522.bit

mfrc522.bit: mfrc522.config
	ecppack --compress $< $@

mfrc522.config: mfrc522.json $(LPF)
	nextpnr-ecp5 --$(DEVICE) --package $(PACKAGE) --json $< --textcfg $@ --lpf $(LPF) --freq $(FREQ)

mfrc522.json: $(SOURCE)
	yosys -p "read_verilog $<; synth_ecp5 -top $(TOP_MODULE) -json $@"

prog: mfrc522.bit
	sudo fujprog $<

flash: mfrc522.bit
	sudo fujprog -j flash $<

clean:
	rm -f *.json *.config *.bit

info:
	@echo "MFRC522 Hardware Test for ULX3S"
	@echo "================================"
	@echo "Source:  $(SOURCE)"
	@echo "Device:  ECP5-$(DEVICE) $(PACKAGE)"
	@echo "Clock:   $(FREQ) MHz"
	@echo ""
	@echo "Targets:"
	@echo "  make        - Build bitstream"
	@echo "  make prog   - Program SRAM (temporary)"
	@echo "  make flash  - Program Flash (persistent)"
	@echo "  make clean  - Remove build files"

.PHONY: all prog flash clean info

.PHONY: all prog flash clean info
