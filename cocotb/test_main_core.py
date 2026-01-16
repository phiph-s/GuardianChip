import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer, Event
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend
import os

# --- Constants ---
# MFRC522 Registers
REG_COMMAND     = 0x01
REG_COMIEN      = 0x02
REG_DIVIEN      = 0x03
REG_COMIRQ      = 0x04
REG_DIVIRQ      = 0x05
REG_ERROR       = 0x06
REG_STATUS1     = 0x07
REG_STATUS2     = 0x08
REG_FIFODATA    = 0x09
REG_FIFOLEVEL   = 0x0A
REG_CONTROL     = 0x0C
REG_BITFRAMING  = 0x0D
REG_COLL        = 0x0E
REG_MODE        = 0x11
REG_TXCONTROL   = 0x14
REG_TXAUTO      = 0x15
REG_VERSION     = 0x37

# MFRC522 Commands
PCD_IDLE        = 0x00
PCD_AUTHENT     = 0x0E
PCD_RECEIVE     = 0x08
PCD_TRANSMIT    = 0x04
PCD_TRANSCEIVE  = 0x0C
PCD_RESETPHASE  = 0x0F
PCD_CALCCRC     = 0x03

# ISO14443A Commands
PICC_REQA       = 0x26
PICC_WUPA       = 0x52
PICC_ANTICOLL   = 0x93
PICC_SELECT     = 0x93
PICC_HALT       = 0x50

# LAYR Protocol
CMD_AUTH_INIT   = 0x80
CMD_AUTH        = 0x81 # Assuming
CMD_GET_ID      = 0x82 # Assuming

# --- Helper Functions ---
def aes_encrypt(key, data):
    cipher = Cipher(algorithms.AES(key), modes.ECB(), backend=default_backend())
    encryptor = cipher.encryptor()
    return encryptor.update(data) + encryptor.finalize()

def aes_decrypt(key, data):
    cipher = Cipher(algorithms.AES(key), modes.ECB(), backend=default_backend())
    decryptor = cipher.decryptor()
    return decryptor.update(data) + decryptor.finalize()

# --- Models ---

class AT25010_Model:
    def __init__(self, dut):
        self.dut = dut
        self.memory = [0xFF] * 128
        self.status = 0x00
        self.wel = False
        self.dut.eeprom_spi_miso.value = 1
        cocotb.start_soon(self.run())

    async def run(self):
        while True:
            await FallingEdge(self.dut.eeprom_spi_cs_n)
            try:
                opcode = await self.spi_read_byte()
                if opcode == 0x06: self.wel = True
                elif opcode == 0x04: self.wel = False
                elif opcode == 0x05: await self.spi_write_byte(self.status | (0x02 if self.wel else 0x00))
                elif opcode == 0x01: self.status = await self.spi_read_byte()
                elif opcode == 0x02:
                    addr = await self.spi_read_byte()
                    data = await self.spi_read_byte()
                    if self.wel: self.memory[addr & 0x7F] = data; self.wel = False
                elif opcode == 0x03:
                    addr = await self.spi_read_byte()
                    data = self.memory[addr & 0x7F]
                    await self.spi_write_byte(data)
            except: pass
            if self.dut.eeprom_spi_cs_n.value == 0: await RisingEdge(self.dut.eeprom_spi_cs_n)
            self.dut.eeprom_spi_miso.value = 1

    async def spi_read_byte(self):
        data = 0
        for _ in range(8):
            await RisingEdge(self.dut.eeprom_spi_sclk)
            if self.dut.eeprom_spi_cs_n.value == 1: raise Exception()
            data = (data << 1) | int(self.dut.eeprom_spi_mosi.value)
        return data

    async def spi_write_byte(self, data):
        for i in range(8):
            await FallingEdge(self.dut.eeprom_spi_sclk)
            if self.dut.eeprom_spi_cs_n.value == 1: raise Exception()
            self.dut.eeprom_spi_miso.value = (data >> (7 - i)) & 1

class MFRC522_Model:
    def __init__(self, dut):
        self.dut = dut
        self.registers = {i: 0x00 for i in range(64)}
        self.registers[REG_VERSION] = 0x92
        self.fifo = []
        self.dut.nfc_spi_miso.value = 0
        self.card_present = False
        self.card_uid = [0x01, 0x02, 0x03, 0x04]
        self.psk = bytes([0x00]*16) # Default PSK
        self.card_id = bytes([0xAA]*16)
        cocotb.start_soon(self.run())

    async def run(self):
        while True:
            await FallingEdge(self.dut.nfc_spi_cs_n)
            try:
                byte0 = await self.spi_read_byte()
                # MFRC522: Read=1, Write=0 (MSB)
                is_read = (byte0 & 0x80) == 0x80
                addr = (byte0 >> 1) & 0x3F
                
                if is_read:
                    val = 0x00
                    if addr == REG_FIFODATA:
                        val = self.fifo.pop(0) if self.fifo else 0x00
                        self.registers[REG_FIFOLEVEL] = len(self.fifo)
                    else:
                        val = self.registers.get(addr, 0x00)
                    await self.spi_write_byte(val)
                else:
                    val = await self.spi_read_byte()
                    if addr == REG_FIFODATA:
                        self.fifo.append(val)
                        self.registers[REG_FIFOLEVEL] = len(self.fifo)
                    elif addr == REG_COMMAND:
                        self.registers[addr] = val
                        await self.process_command(val)
                    else:
                        self.registers[addr] = val
            except Exception as e:
                pass
            if self.dut.nfc_spi_cs_n.value == 0: await RisingEdge(self.dut.nfc_spi_cs_n)
            self.dut.nfc_spi_miso.value = 0

    async def spi_read_byte(self):
        data = 0
        for _ in range(8):
            await RisingEdge(self.dut.nfc_spi_sclk)
            if self.dut.nfc_spi_cs_n.value == 1: raise Exception()
            data = (data << 1) | int(self.dut.nfc_spi_mosi.value)
        return data

    async def spi_write_byte(self, data):
        for i in range(8):
            await FallingEdge(self.dut.nfc_spi_sclk)
            if self.dut.nfc_spi_cs_n.value == 1: raise Exception()
            self.dut.nfc_spi_miso.value = (data >> (7 - i)) & 1

    async def process_command(self, cmd):
        # Simulate MFRC522 Command Processing
        if cmd == PCD_TRANSCEIVE:
            # Read data from FIFO (simulating transmission to card)
            tx_data = self.fifo[:]
            self.fifo = [] # Clear FIFO after TX
            self.registers[REG_FIFOLEVEL] = 0
            
            cocotb.log.info(f"[MFRC522] Transmitting: {[hex(x) for x in tx_data]}")
            
            if not self.card_present:
                # No response (Timeout)
                self.registers[REG_COMIRQ] |= 0x01 # TimerIRq (Timeout)
                return

            # Card Logic
            response = []
            
            if len(tx_data) == 1 and tx_data[0] == PICC_REQA:
                cocotb.log.info("[Card] Received REQA -> Sending ATQA")
                response = [0x04, 0x00] # ATQA
                
            elif len(tx_data) == 2 and tx_data[0] == PICC_ANTICOLL and tx_data[1] == 0x20:
                cocotb.log.info("[Card] Received ANTICOLL -> Sending UID")
                # UID + BCC
                bcc = 0
                for b in self.card_uid: bcc ^= b
                response = self.card_uid + [bcc]
                
            elif len(tx_data) >= 2 and tx_data[0] == PICC_SELECT:
                cocotb.log.info("[Card] Received SELECT -> Sending SAK")
                response = [0x08, 0xB6, 0xDD] # SAK + CRC (Dummy CRC)
                
            elif len(tx_data) == 2 and tx_data[0] == 0x80 and tx_data[1] == 0x10:
                cocotb.log.info("[Card] Received AUTH_INIT -> Sending Encrypted Challenge")
                # Generate Challenge (RC)
                rc = bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88])
                padding = bytes([0x00]*8)
                plaintext = rc + padding
                # Encrypt with PSK
                ciphertext = aes_encrypt(self.psk, plaintext)
                response = list(ciphertext)
                self.last_rc = rc
                
            elif len(tx_data) == 18 and tx_data[0] == 0x80 and tx_data[1] == 0x11:
                # AUTH command: 0x80 0x11 + 16 bytes encrypted data
                cocotb.log.info("[Card] Received AUTH -> Verifying Challenge")
                encrypted_data = bytes(tx_data[2:])
                decrypted = aes_decrypt(self.psk, encrypted_data)
                
                # Decrypted should be rt || rc
                rt = decrypted[:8]
                rc_received = decrypted[8:]
                
                if rc_received == self.last_rc:
                    cocotb.log.info("[Card] Authentication Successful")
                    response = [0x00] # Success
                    
                    # Derive Session Key: E_psk(rc || rt)
                    plaintext = rc_received + rt
                    self.session_key = aes_encrypt(self.psk, plaintext)
                else:
                    cocotb.log.info(f"[Card] Auth Failed. Expected RC: {self.last_rc.hex()}, Got: {rc_received.hex()}")
                    response = [0xFF] # Fail

            elif len(tx_data) == 2 and tx_data[0] == 0x80 and tx_data[1] == 0x12:
                # GET_ID command
                cocotb.log.info("[Card] Received GET_ID -> Sending Encrypted ID")
                if hasattr(self, 'session_key'):
                    # Encrypt Card ID with Session Key
                    encrypted_id = aes_encrypt(self.session_key, self.card_id)
                    response = list(encrypted_id)
                else:
                    response = [0xFF] # Not authenticated

            
            # Put response in FIFO
            if response:
                self.fifo = response
                self.registers[REG_FIFOLEVEL] = len(response)
                self.registers[REG_COMIRQ] |= 0x20 # RxIRq (Receive Complete)
                cocotb.log.info(f"[MFRC522] Received Response: {[hex(x) for x in response]}")
            else:
                # Timeout
                self.registers[REG_COMIRQ] |= 0x01

        elif cmd == PCD_IDLE:
            pass # Stop current command

@cocotb.test()
async def test_main_core_full_flow(dut):
    """Test Main Core: Full Authentication Flow"""
    
    clock = Clock(dut.clk, 10, unit="ns") # 100 MHz
    cocotb.start_soon(clock.start())
    
    # Initialize Models
    eeprom = AT25010_Model(dut)
    nfc = MFRC522_Model(dut)
    
    # Setup PSK in EEPROM
    psk = bytes([0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 
                 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F])
    nfc.psk = psk # Give card the same key
    for i, b in enumerate(psk):
        eeprom.memory[i] = b
        
    # Reset
    dut.rst_n.value = 0
    dut.nfc_irq.value = 0
    await Timer(100, unit="ns")
    dut.rst_n.value = 1
    await Timer(100, unit="ns")
    
    # 1. Trigger Card Detection
    cocotb.log.info("--- Step 1: Trigger Card Detection ---")
    nfc.card_present = True
    dut.nfc_irq.value = 1
    await Timer(100, unit="ns")
    dut.nfc_irq.value = 0
    
    # Wait for detection to complete (Monitor internal state or outputs)
    # Since we expect it to fail due to RTL bug, we set a timeout
    
    try:
        await Timer(500000, unit="ns") # Wait 500us
        
        # Check if REQA was sent
        # In the broken RTL, it writes to FIFO but doesn't Transceive.
        # So MFRC522 Model will see writes to FIFO but no Transceive command.
        
        # Check if door unlocked
        if dut.door_unlock.value == 1:
             cocotb.log.info("✓ Door Unlocked Successfully!")
        else:
             assert False, "Door did not unlock. Authentication failed or stuck."
        
    except Exception as e:
        cocotb.log.error(f"Test failed with exception: {e}")
        raise

    # If the RTL was working, we would see:
    # 1. REQA sent -> ATQA received
    # 2. ANTICOLL sent -> UID received
    # 3. SELECT sent -> SAK received
    # 4. AUTH_INIT sent -> Challenge received
    # ...
    
    # Since we know RTL is likely broken regarding MFRC522 command execution:
    # We expect the test to time out or fail to progress.
    
    # For now, let's just assert that we at least tried to communicate.
    # We can check if CS was asserted.
    
    cocotb.log.info("Verifying if any SPI communication happened...")
    # This is hard to check post-facto without monitors.
    
    # Let's fail if door didn't unlock (which is the ultimate goal)
    # But for this specific run, we expect failure.
    
    if dut.door_unlock.value == 1:
        cocotb.log.info("SUCCESS: Door Unlocked!")
    else:
        cocotb.log.warning("FAILURE: Door did not unlock (Expected due to known RTL issues)")
        # We don't raise assertion error to let the user see the log, 
        # or we can raise it to be strict.
        # The user said "Testbench should behave like real hardware".
        # So if it fails, it fails.
        assert False, "Door did not unlock. Authentication failed or stuck."

