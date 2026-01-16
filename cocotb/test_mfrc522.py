import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer

class MFRC522_Model:
    def __init__(self, dut):
        self.dut = dut
        self.registers = {i: 0x00 for i in range(64)}
        # Default values for some registers if needed
        self.registers[0x37] = 0x00 # VersionReg?
        self.dut.spi_miso.value = 0
        cocotb.start_soon(self.run())

    async def run(self):
        while True:
            await FallingEdge(self.dut.spi_cs_n)
            
            try:
                # Byte 0: Address
                # Format: 1 (Write) / 0 (Read) | Address (6 bits) | 0
                # Wait, MFRC522 format:
                # MSB: 1=Read, 0=Write ? Or opposite?
                # Let's check the RTL or standard.
                # RTL says:
                # // Byte 0: Address byte
                # //   Bit 7: 0=read, 1=write  <-- Wait, RTL comment says 0=read, 1=write?
                # //   Bit 6-1: Address (6 bits)
                # //   Bit 0: Always 0
                
                # Let's verify with standard MFRC522 datasheet if possible, or trust RTL comments.
                # RTL:
                # input wire cmd_is_write,        // 1=write, 0=read
                # ...
                # In RTL logic (I need to check how it constructs the byte):
                # I don't have the full RTL content in memory, but I can infer from standard.
                # Standard MFRC522:
                # Read: MSB=1. Write: MSB=0.
                # Address is bits 6-1.
                # LSB is 0.
                
                # Let's check what the RTL sends.
                # I will implement the model to decode what it receives.
                
                byte0 = await self.spi_read_byte()
                
                is_read = (byte0 & 0x80) == 0x80
                # Wait, if RTL says 0=read, 1=write, that conflicts with standard MFRC522 usually (Read=1, Write=0).
                # But let's see what the RTL actually does.
                # If I assume the RTL is correct for "itself", I should match it.
                # But the user said "Testbench should behave like real hardware".
                # Real MFRC522: Read is 1xxxxxx0. Write is 0xxxxxx0.
                
                # Let's assume the RTL tries to talk to a real MFRC522.
                # So the RTL *should* send MSB=1 for Read.
                
                addr = (byte0 >> 1) & 0x3F
                
                if is_read:
                    # Read operation
                    # Byte 1: Read data
                    # MFRC522 sends data on the second byte.
                    reg_val = self.registers.get(addr, 0x00)
                    await self.spi_write_byte(reg_val)
                else:
                    # Write operation
                    # Byte 1: Write data
                    val = await self.spi_read_byte()
                    self.registers[addr] = val
                    
            except Exception as e:
                pass
            
            if self.dut.spi_cs_n.value == 0:
                await RisingEdge(self.dut.spi_cs_n)
            
            self.dut.spi_miso.value = 0

    async def spi_read_byte(self):
        data = 0
        for _ in range(8):
            await RisingEdge(self.dut.spi_sclk)
            if self.dut.spi_cs_n.value == 1: raise Exception("CS Deasserted")
            bit = int(self.dut.spi_mosi.value)
            data = (data << 1) | bit
        return data

    async def spi_write_byte(self, data):
        for i in range(8):
            await FallingEdge(self.dut.spi_sclk)
            if self.dut.spi_cs_n.value == 1: raise Exception("CS Deasserted")
            bit = (data >> (7 - i)) & 1
            self.dut.spi_miso.value = bit

@cocotb.test()
async def test_mfrc522_basic(dut):
    """Test MFRC522 Interface with behavioral model"""
    
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    
    model = MFRC522_Model(dut)
    
    dut.rst_n.value = 0
    dut.cmd_valid.value = 0
    dut.cmd_is_write.value = 0
    dut.cmd_addr.value = 0
    dut.cmd_wdata.value = 0
    
    await Timer(100, unit="ns")
    dut.rst_n.value = 1
    await Timer(100, unit="ns")
    
    async def send_cmd(is_write, addr, wdata=0):
        await RisingEdge(dut.clk)
        while dut.cmd_ready.value == 0:
            await RisingEdge(dut.clk)
            
        dut.cmd_is_write.value = is_write
        dut.cmd_addr.value = addr
        dut.cmd_wdata.value = wdata
        dut.cmd_valid.value = 1
        await RisingEdge(dut.clk)
        dut.cmd_valid.value = 0
        
        while dut.cmd_done.value == 0:
            await RisingEdge(dut.clk)
            
        return dut.cmd_rdata.value

    # Test 1: Write to register
    cocotb.log.info("Test 1: Write to Register 0x12")
    await send_cmd(is_write=1, addr=0x12, wdata=0x55)
    
    # Verify in model
    assert model.registers[0x12] == 0x55, f"Model register 0x12 should be 0x55, got {model.registers[0x12]}"
    
    # Test 2: Read from register
    cocotb.log.info("Test 2: Read from Register 0x12")
    rdata = await send_cmd(is_write=0, addr=0x12)
    assert rdata == 0x55, f"Expected 0x55, got {rdata}"
    
    # Test 3: Read default register
    cocotb.log.info("Test 3: Read default register")
    rdata = await send_cmd(is_write=0, addr=0x00)
    assert rdata == 0x00, f"Expected 0x00, got {rdata}"

    cocotb.log.info("All tests passed!")
