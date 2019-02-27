import pychronos

# TODO: This breaks the numpy array access since we haven't implemented
# the buffer protocol. It may be worth moving this back into C as a
# special class of the arrayview so that we can fix it.
class ccmArrayView:
    """Private array view class to fix the CCM address gap"""
    def __init__(self, parent, offset):
        self.offset = offset
        self.parent = parent
    
    # Iterator class
    class __ccmIterator:
        def __init__(self, parent):
            self.parent = parent
            self.key = 0
        
        def __iter__(self):
            return self

        def __next__(self):
            if (self.key >= 9):
                raise StopIteration
            value = self.parent[self.key]
            self.key += 1
            return value

    # Fixes the CCM address gap
    def __key2addr(self, key):
        if (key < 3):
            return (self.offset + key * 4)
        else:
            return (self.offset + (key+1) * 4)

    def __len__(self):
        return 9

    def __getitem__(self, key):
        u16val = self.parent.mem16[self.__key2addr(key) // 2]
        return (u16val ^ 0x8000) - 0x8000   # Convert to signed
    
    def __setitem__(self, key, value):
        self.parent.mem16[self.__key2addr(key) // 2] = int(value) & 0xffff
    
    def __iter__(self):
        return self.__ccmIterator(self)

class display(pychronos.fpgamap):
    """Return a new map of the FPGA display register space.

    Return a new map of the FPGA display register space. This map provides
    structured read and write access to the registers which control the
    playback and display of frames out of video RAM to the primary video
    port.

    offset : `int`, optional
        Starting offset of the register map (FPGA_DISPLAY_BASE, by default)
    size : `int`, optional
        Length of the register map (FPGA_MAP_SIZE, by default)
    """
    def __init__(self, offset=pychronos.FPGA_DISPLAY_BASE, size=0x100):
        super().__init__(offset, size)

    def __regprop(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        fset=lambda self, value: self.regWrite(offset, size, value),
                        doc = docstring)
    
    # Display Registers
    control =       __regprop(0x00, 2, "Control Register")
    frameAddr =     __regprop(0x04, 4, "Frame Address Register")
    fpnAddr =       __regprop(0x08, 4, "FPN Address Register")
    gain =          __regprop(0x0C, 2, "Gain Control Register")
    hPeriod =       __regprop(0x10, 2, "Horizontal Period Register")
    vPeriod =       __regprop(0x14, 2, "Vertical Period Register")
    hSyncLen =      __regprop(0x18, 2, "Horizontal Sync Length Register")
    vSyncLen =      __regprop(0x1C, 2, "Vertical Sync Length Register")
    hBackPorch =    __regprop(0x20, 2, "Horizontal Back Porch Register")
    vBackPorch =    __regprop(0x24, 2, "Vertical Back Porch Register")
    hRes =          __regprop(0x28, 2, "Horizontal Resolution Register")
    vRes =          __regprop(0x2C, 2, "Vertical Resolution Register")
    hOutRes =       __regprop(0x30, 2, "Horizontal Output Register")
    vOutRes =       __regprop(0x40, 2, "Vertical Output Register")
    peakThresh =    __regprop(0x44, 2, "Focus Peaking Threshold Register")
    pipeline =      __regprop(0x48, 2, "Display Pipeline Register")
    manualSync =    __regprop(0x50, 2, "Manual Frame Sync Register")
    gainControl =   __regprop(0x54, 2, "Gain Calibration Control Register")
    
    # Display Constrol Constants
    ADDR_SELECT     = (1 << 0)
    SCALER_NN       = (1 << 1)
    SYNC_INHIBIT    = (1 << 2)
    READOUT_INHIBIT = (1 << 3)
    COLOR_MODE      = (1 << 4)
    FOCUS_PEAK      = (1 << 5)
    FOCUS_COLOR     = (0x7 << 6) # TODO: Maybe this needs to be a bitmask.
    ZEBRA_ENABLE    = (1 << 9)
    BLACK_CAL_MODE  = (1 << 10)

    # Display Pipeline Constants
    BYPASS_FPN      = (1 << 0)
    BYPASS_GAIN     = (1 << 1)
    BYPASS_DEMOSAIC = (1 << 2)
    BYPASS_COLOR_MATRIX = (1 << 3)
    BYPASS_GAMMA_TABLE  = (1 << 4)
    RAW_12BPP       = (1 << 5)
    RAW_16BPP       = (1 << 6)
    RAW_16PAD       = (1 << 7)
    TEST_PATTERN    = (1 << 15)

    # Display Gain Control Constants
    GAINCTL_3POINT  = (1 << 0)

    # Scale for fixed-point integers
    WHITE_BALANCE_BITS  = 12
    WHITE_BALANCE_DIV   = (1 << WHITE_BALANCE_BITS)
    COLOR_MATRIX_BITS   = 12
    COLOR_MATRIX_DIV    = (1 << COLOR_MATRIX_BITS)

    @property
    def whiteBalance(self):
        """White Balance Matrix"""
        return pychronos.arrayview(self, offset=0xF0, size=4, count=3)

    @property
    def colorMatrix(self):
        """Color Correction Matrix"""
        return ccmArrayView(self, offset=0xC0)
