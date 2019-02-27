import pychronos

class vram(pychronos.fpgamap):
    """Return a new map of the FPGA video RAM readout register space.

    Return a new map of the FPGA video RAM readout register space. This map
    provides structured read and write access to the registers and cache used
    for the readout of data from video RAM.

    offset : `int`, optional
        Starting offset of the register map (FPGA_VRAM_BASE, by default)
    size : `int`, optional
        Length of the register map (FPGA_MAP_SIZE, by default)
    """
    def __init__(self, offset=pychronos.FPGA_VRAM_BASE, size=0x100):
        self.offset = offset
        super().__init__(offset, size)
    
    def __regprop(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        fset=lambda self, value: self.regWrite(offset, size, value),
                        doc = docstring)

    # Video RAM Readout Registers
    idReg       = __regprop(0x00, 2, "Identifier")
    version     = __regprop(0x04, 2, "Version")
    subver      = __regprop(0x08, 2, "Sub Version")
    control     = __regprop(0x0C, 2, "Control Register")
    status      = __regprop(0x10, 2, "Status Register")
    address     = __regprop(0x20, 4, "Video RAM Address")
    burst       = __regprop(0x24, 2, "Transaction Burst Length")
    
    # Video RAM Readout Constants
    IDENTIFIER  = 0x10
    TRIG_READ   = (1 << 0)
    TRIG_WRITE  = (1 << 1)

    @property
    def buffer(self):
        """Video RAM Buffer"""
        return pychronos.fpgamap(self.offset + 0x200, 2048)
    
    # TODO: Other helpful features might include:
    #   - Yielding generator to perform the readout sequence.
    #   - Test if the block exist.
