# IO system for Chronos high speed cameras
import pychronos
import logging

class trigger(pychronos.fpgamap):
    """Return a new map of the FPGA trigger register space.
    
    Return a new map of the FPGA trigger register space. This map provides
    structured read and write access to the legacy registers which external
    trigger and IO control.

    Parameters
    ----------
    offset : `int`, optional
        Starting offset of the register map (FPGA_TRIGGER_BASE, by default)
    size : `int`, optional
        Length of the register map (FPGA_MAP_SIZE, by default)
    """
    def __init__(self, offset=pychronos.FPGA_TRIGGER_BASE, size=0x20):
        super().__init__(offset, size)
    
    def __regprop(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        fset=lambda self, value: self.regWrite(offset, size, value),
                        doc = docstring)
    
    enable =        __regprop(0x00, 2, "Trigger Enable Register")
    invert =        __regprop(0x04, 2, "Trigger Invert Register")
    debounce =      __regprop(0x08, 2, "Trigger Debounce Register")
    ioOutLevel =    __regprop(0x0C, 2, "IO Output Level Register")
    ioOutSource =   __regprop(0x10, 2, "IO Output Source Register")
    ioOutInvert =   __regprop(0x14, 2, "IO Output Invert Register")
    ioInput =       __regprop(0x18, 2, "IO Input Status Register")
    extShutter =    __regprop(0x1C, 2, "External Shutter Control Register")
