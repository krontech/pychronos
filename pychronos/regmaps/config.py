import pychronos

class config(pychronos.fpgamap):
    """Return a new map of the FPGA configuration register space.

    Return a new map of the FPGA display register space. This map provides
    structured read and write access to the registers which control the
    playback and display of frames out of video RAM to the primary video
    port.

    offset : `int`, optional
        Starting offset of the register map (FPGA_DISPLAY_BASE, by default)
    size : `int`, optional
        Length of the register map (FPGA_MAP_SIZE, by default)
    """
    def __init__(self, offset=pychronos.FPGA_CONFIG_BASE, size=0x200):
        super().__init__(offset, size)

    def __regprop(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        fset=lambda self, value: self.regWrite(offset, size, value),
                        doc = docstring)
    
    mmuConfig =     __regprop(0x020, 4, "MMU Configuration")
    sysReset =      __regprop(0x100, 2, "FPGA Soft Reset Control")
    version =       __regprop(0x104, 2, "FPGA Version")
    subver =        __regprop(0x104, 2, "FPGA Sub Version")

    MMU_INVERT_CS       = (1 << 0)  # Invert SODIMM chip select
    MMU_SWITCH_STUFFED  = (1 << 1)  # Stuff 8GB SODIMMs into 16GB space

    @property
    def wlDelay(self):
        """Write Leveling Delay"""
        return pychronos.arrayview(self, offset=0x00, size=4, count=4)
