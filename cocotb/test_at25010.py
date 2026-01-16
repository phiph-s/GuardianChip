import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer, Edge

# Command constants
CMD_WREN  = 0
CMD_WRDI  = 1
CMD_RDSR  = 2
CMD_WRSR  = 3
CMD_READ  = 4
CMD_WRITE = 5

class AT25010_Model:
    def __init__(self, dut):
        self.dut = dut
        self.memory = [0xFF] * 128
        self.status = 0x00
        self.wel = False  # Write Enable Latch
        self.dut.spi_miso.value = 1 # High-Z / Idle high
        cocotb.start_soon(self.run())

    async def run(self):
        while True:
            # Wait for CS falling edge (Chip Select)
            await FallingEdge(self.dut.spi_cs_n)
            
            try:
                # Read Opcode
                opcode = await self.spi_read_byte()
                
                if opcode == 0x06: # WREN
                    self.wel = True
                    # self.log("WREN executed")
                elif opcode == 0x04: # WRDI
                    self.wel = False
                    # self.log("WRDI executed")
                elif opcode == 0x05: # RDSR
                    # Read Status Register
                    status_byte = self.status | (0x02 if self.wel else 0x00)
                    await self.spi_write_byte(status_byte)
                elif opcode == 0x01: # WRSR
                    status = await self.spi_read_byte()
                    # self.log(f"WRSR: {status:02x}")
                    # Only WPEN, BP1, BP0 are writable usually, but let's just store it
                    self.status = status
                elif opcode == 0x02: # WRITE
                    addr = await self.spi_read_byte()
                    data = await self.spi_read_byte()
                    if self.wel:
                        self.memory[addr & 0x7F] = data
                        self.wel = False # Reset WEL after write
                        # self.log(f"WRITE: addr={addr:02x}, data={data:02x}")
                    else:
                        pass # self.log("WRITE ignored (WEL=0)")
                elif opcode == 0x03: # READ
                    addr = await self.spi_read_byte()
                    data = self.memory[addr & 0x7F]
                    await self.spi_write_byte(data)
                    # self.log(f"READ: addr={addr:02x}, data={data:02x}")
                else:
                    pass # self.log(f"Unknown opcode: {opcode:02x}")

            except Exception as e:
                # CS might go high in the middle, resetting the state
                pass
            
            # Wait for CS to go high if it hasn't already
            if self.dut.spi_cs_n.value == 0:
                await RisingEdge(self.dut.spi_cs_n)
            
            self.dut.spi_miso.value = 1 # Tri-state/Idle

    async def spi_read_byte(self):
        data = 0
        for _ in range(8):
            # Sample on Rising Edge
            await RisingEdge(self.dut.spi_sclk)
            if self.dut.spi_cs_n.value == 1: raise Exception("CS Deasserted")
            bit = int(self.dut.spi_mosi.value)
            data = (data << 1) | bit
        return data

    async def spi_write_byte(self, data):
        for i in range(8):
            # Drive on Falling Edge
            await FallingEdge(self.dut.spi_sclk)
            if self.dut.spi_cs_n.value == 1: raise Exception("CS Deasserted")
            bit = (data >> (7 - i)) & 1
            self.dut.spi_miso.value = bit
            # Master samples on next Rising Edge

    def log(self, msg):
        cocotb.log.info(f"[AT25010 Model] {msg}")

@cocotb.test()
async def test_at25010_basic(dut):
    """Test AT25010 Interface with a behavioral model"""
    
    # Initialize Clock
    clock = Clock(dut.clk, 20, unit="ns") # 50 MHz
    cocotb.start_soon(clock.start())
    
    # Initialize Model
    eeprom = AT25010_Model(dut)
    
    # Reset
    dut.rst_n.value = 0
    dut.cmd_valid.value = 0
    dut.cmd_type.value = 0
    dut.cmd_addr.value = 0
    dut.cmd_wdata.value = 0
    
    await Timer(100, unit="ns")
    dut.rst_n.value = 1
    await Timer(100, unit="ns")
    
    # Helper to send command
    async def send_cmd(cmd_type, addr=0, wdata=0):
        await RisingEdge(dut.clk)
        while dut.cmd_ready.value == 0:
            await RisingEdge(dut.clk)
        
        dut.cmd_type.value = cmd_type
        dut.cmd_addr.value = addr
        dut.cmd_wdata.value = wdata
        dut.cmd_valid.value = 1
        await RisingEdge(dut.clk)
        dut.cmd_valid.value = 0
        
        # Wait for done
        while dut.cmd_done.value == 0 and dut.cmd_error.value == 0:
            await RisingEdge(dut.clk)
            
        return dut.cmd_rdata.value

    # Test 1: Read Status (Should be 0)
    cocotb.log.info("Test 1: Read Status")
    rdata = await send_cmd(CMD_RDSR)
    assert rdata == 0x00, f"Expected status 0x00, got {rdata}"
    
    # Test 2: Write Enable
    cocotb.log.info("Test 2: Write Enable")
    await send_cmd(CMD_WREN)
    
    # Verify WEL bit set
    rdata = await send_cmd(CMD_RDSR)
    assert rdata == 0x02, f"Expected status 0x02 (WEL), got {rdata}"
    
    # Test 3: Write Data
    cocotb.log.info("Test 3: Write Data")
    # Need to WREN again? The model resets WEL after write? 
    # No, we just did WREN, haven't written yet.
    # Wait, the previous RDSR shouldn't clear WEL.
    
    await send_cmd(CMD_WRITE, addr=0x10, wdata=0xAB)
    
    # Wait a bit for internal write cycle (simulated by model being instant, but interface has delay)
    await Timer(1000, unit="ns")
    
    # Test 4: Read Data
    cocotb.log.info("Test 4: Read Data")
    rdata = await send_cmd(CMD_READ, addr=0x10)
    assert rdata == 0xAB, f"Expected data 0xAB, got {rdata}"
    
    # Test 5: Verify WEL cleared after write
    rdata = await send_cmd(CMD_RDSR)
    assert rdata == 0x00, f"Expected status 0x00 (WEL cleared), got {rdata}"

    cocotb.log.info("All tests passed!")
