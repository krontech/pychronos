import pychronos

class overlay(pychronos.fpgamap):
    """Return a new map of the FPGA overlay register space.

    Return a new map of the FPGA video overlay register space. This map provides
    structured read and write access to the registers used to configure text and
    and watermarking overlays onto the video stream.

    offset : `int`, optional
        Starting offset of the register map (FPGA_OVERLAY_BASE, by default)
    size : `int`, optional
        Length of the register map (FPGA_MAP_SIZE, by default)
    """
    def __init__(self, offset=pychronos.FPGA_OVERLAY_BASE, size=0x1000):
        super().__init__(offset, size)

    def __regprop(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        fset=lambda self, value: self.regWrite(offset, size, value),
                        doc = docstring)
    
    def __regbytes(offset, size, docstring):
        return property(fget=lambda self: pychronos.arrayview(self, offset=offset, size=1, count=size), doc=docstring)

    OVERLAY_TEXT_LENGTH = 128
    OVERLAY_LUT_SIZE = 256

    # Overlay Registers
    id =            __regprop(0x00, 2, "Identifier")
    version =       __regprop(0x02, 2, "Version")
    subver =        __regprop(0x04, 2, "Sub Version")
    control =       __regprop(0x06, 2, "Control Register")
    status =        __regprop(0x08, 2, "Status Register")
    text0hpos =     __regprop(0x0A, 2, "Textbox 0 Horizontal Position")
    text0vpos =     __regprop(0x0C, 2, "Textbox 0 Vertical Position")
    text0width =    __regprop(0x0E, 2, "Textbox 0 Width")
    text0height =   __regprop(0x10, 2, "Textbox 0 Hight")
    text1hpos =     __regprop(0x12, 2, "Textbox 1 Horizontal Position")
    text1vpos =     __regprop(0x14, 2, "Textbox 1 Vertical Position")
    text1width =    __regprop(0x16, 2, "Textbox 1 Width")
    text1height =   __regprop(0x18, 2, "Textbox 1 Hight")
    wmarkHpos =     __regprop(0x1A, 2, "Watermark Horizontal Position")
    wmarkVpos =     __regprop(0x1C, 2, "Watermark Vertical Position")
    text0hoff =     __regprop(0x1E, 1, "Textbox 0 Horizontal Offset")
    text1hoff =     __regprop(0x1F, 1, "Textbox 1 Horizontal Offset")
    text0voff =     __regprop(0x20, 1, "Textbox 0 Vertical Offset")
    text1voff =     __regprop(0x21, 1, "Textbox 1 Vertical Offset")
    logoHpos =      __regprop(0x22, 2, "Logo Horizontal Position")
    logoVpos =      __regprop(0x24, 2, "Logo Vertical Position")
    logoWidth =     __regprop(0x26, 2, "Logo Width")
    logoHeight =    __regprop(0x28, 2, "Logo Hight")

    text0color =    __regbytes(0x2A, 4, "Textbox 0 Color")
    text1color =    __regbytes(0x2E, 4, "Textbox 1 Color")
    wmarkColor =    __regbytes(0x32, 4, "Watermark Color")
    text0buf =      __regbytes(0x100, OVERLAY_TEXT_LENGTH, "Textbox 0 Buffer")
    text1buf =      __regbytes(0x200, OVERLAY_TEXT_LENGTH, "Textbox 1 Buffer")
    logoRedLut =    __regbytes(0x300, OVERLAY_LUT_SIZE, "Logo Red Lookup Table")
    logoGreenLut =  __regbytes(0x400, OVERLAY_LUT_SIZE, "Logo Green Lookup Table")
    logoBlueLut =   __regbytes(0x500, OVERLAY_LUT_SIZE, "Logo Blue Lookup Table")

    @property
    def text0fonts(self):
        """Textbox 0 Font Bitmaps"""
        return pychronos.arrayview(self, offset=0x1000, size=2, count=4096)
    
    @property
    def text1fonts(self):
        """Textbox 1 Font Bitmaps"""
        return pychronos.arrayview(self, offset=0x3000, size=2, count=4096)
    
    @property
    def logo(self):
        """Logo Image Buffer"""
        ## TODO: I think the FPGA register addressing is wrong for the logo buffer.
        return pychronos.arrayview(self, offset=0x5000, size=1, count=16384)
