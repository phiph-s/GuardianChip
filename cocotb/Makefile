# Makefile

# Defaults
SIM ?= icarus
TOPLEVEL_LANG ?= verilog

# Common sources
SPI_MASTER_SRC = $(PWD)/../src/user_rtl/ip/spi-master/SPI_Master_With_Single_CS.v $(PWD)/../src/user_rtl/ip/spi-master/SPI_Master.v

# AT25010 Test
at25010:
	rm -rf sim_build
	$(MAKE) sim MODULE=test_at25010 TOPLEVEL=at25010_interface VERILOG_SOURCES="$(PWD)/../src/user_rtl/rtl/at25010_interface.v $(SPI_MASTER_SRC)"

# MFRC522 Test
mfrc522:
	rm -rf sim_build
	$(MAKE) sim MODULE=test_mfrc522 TOPLEVEL=mfrc522_interface VERILOG_SOURCES="$(PWD)/../src/user_rtl/rtl/mfrc522_interface.v $(SPI_MASTER_SRC)"

# NFC Detector Test
nfc_detector:
	rm -rf sim_build
	$(MAKE) sim MODULE=test_nfc_detector TOPLEVEL=nfc_detector_wrapper VERILOG_SOURCES="$(PWD)/nfc_detector_wrapper.v $(PWD)/../src/user_rtl/rtl/nfc_card_detector.v $(PWD)/../src/user_rtl/rtl/mfrc522_interface.v $(SPI_MASTER_SRC)"

# Main Core Test
main_core:
	rm -rf sim_build
	$(MAKE) sim MODULE=test_main_core TOPLEVEL=main_core VERILOG_SOURCES="$(PWD)/../src/user_rtl/rtl/main_core.v $(PWD)/../src/user_rtl/rtl/nfc_card_detector.v $(PWD)/../src/user_rtl/rtl/auth_controller.v $(PWD)/../src/user_rtl/rtl/aes_core.v $(PWD)/../src/user_rtl/rtl/nonce_generator.v $(PWD)/../src/user_rtl/rtl/at25010_interface.v $(PWD)/../src/user_rtl/rtl/mfrc522_interface.v $(PWD)/../src/user_rtl/ip/aes-verilog/*.v $(SPI_MASTER_SRC)"

include $(shell cocotb-config --makefiles)/Makefile.sim
