import pychronos

class sensor(pychronos.fpgamap):
    """Return a new map of the FPGA sensor register space.
    
    Return a new map of the FPGA sensor register space. This map provides
    structured read and write access to the registers which control SCI
    communication to the image sensor, and timing parameters for the
    frame control signals.

    Parameters
    ----------
    offset : `int`, optional
        Starting offset of the register map (FPGA_SENSOR_BASE, by default)
    size : `int`, optional
        Length of the register map (FPGA_MAP_SIZE, by default)
    """
    def __init__(self, offset=pychronos.FPGA_SENSOR_BASE, size=0x100):
        super().__init__(offset, size)

    def __regprop(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        fset=lambda self, value: self.regWrite(offset, size, value),
                        doc = docstring)
    
    # Sensor control registers.
    control =       __regprop(0x00, 2, "Control Register")
    clkPhase =      __regprop(0x04, 2, "Clock Phase Register")
    syncToken =     __regprop(0x08, 2, "Sync Token Register")
    dataCorrect =   __regprop(0x0C, 4, "Data Correct Status Register")
    fifoStart =     __regprop(0x10, 2, "FIFO Starting Threshold Register")
    fifoStop =      __regprop(0x14, 2, "FIFO Stop Threshold Register")
    framePeriod =   __regprop(0x1C, 4, "Frame Period Register")
    intTime =       __regprop(0x20, 4, "Integration Time Register")
    # Serial Communication Interface registers.
    sciControl =    __regprop(0x24, 2, "SCI Control Register")
    sciAddress =    __regprop(0x28, 2, "SCI Address Register")
    sciDataLen =    __regprop(0x2C, 2, "SCI Data Length Register")
    sciFifoWrite =  __regprop(0x30, 2, "SCI Write FIFO Register")
    sciFifoRead =   __regprop(0x34, 2, "SCI Read FIFO Register")
    startDelay =    __regprop(0x68, 2, "ABN/exposure start delay")
    linePeriod =    __regprop(0x6C, 2, "Horizontal line period")
    # Sensor control bits
    RESET       =  (1 << 0)
    EVEN_TIMESLOT = (1 << 1)
    # SCI control bits
    SCI_RUN     = (1 << 0)
    SCI_RW      = (1 << 1)
    SCI_FULL    = (1 << 2)
    SCI_RESET   = (1 << 15)

    def sciRead(self, address, mask=0):
        """Helper function to read an integer value of the given size from the
        image sensor registers using the Serial Communication Interface
        protocol.

        Parameters
        ----------
        address : `int`
            Address of the register to read via SCI.
        mask : `int`, optional
            Mask of bits to extract from the register value.
        
        Returns
        -------
        int
        """
        # Set RW, address and length
        self.sciControl = self.SCI_RW
        self.sciAddress = address
        self.sciDataLen = 2

        # Start the transfer and wait for completion.
        self.sciControl = (self.SCI_RW | self.SCI_RUN)
        for x in range(0, 1000):
            if (self.sciControl & self.SCI_RUN) == 0:
                break

        # Return the result, optionally with a mask.
        if mask:
            lsb = (~mask + 1) & mask
            return (self.sciFifoRead & mask) // lsb
        else:
            return self.sciFifoRead
    
    def sciWrite(self, address, value, mask=0):
        """Helper function to write an integer value of the given size into the
        image sensor registers using the Serial Communication Interface
        protocol.

        Parameters
        ----------
        address : `int`
            Address of the register to read via SCI.
        value : `int`
            Value to be written into the register.
        mask : `int`, optional
            Mask of bits to write into the register value.
        """
        # Do a read/modify/write for masked registers.
        if mask:
            lsb = (~mask + 1) & mask
            value = (value * lsb) | (self.sciRead(address) & ~mask)
        
        # Clear RW and reset the FIFO.
        self.sciControl = self.SCI_RESET
        self.sciAddress = address
        self.sciDataLen = 2
        self.sciFifoWrite = (value >> 8) & 0xff
        self.sciFifoWrite = (value >> 0) & 0xff

        # Start the write and wait for completion.
        self.sciControl = self.SCI_RUN
        for x in range(0, 1000):
            if not (self.sciControl & self.SCI_RUN):
                break
    
    class sciArrayView:
        """Helper class to create an array view of the SCI register
        space that can be indexed or iterated upon to read and write
        the image sensor registers.

        Parameters
        ----------
        parent : `sensor`
            Parent sensor object for the SCI register mapping.
        offset : `int`
            Starting address of the first register in the array.
        length : `int`
            Number of registers in the array.
        """
        def __init__(self, parent, offset, length=0x7F):
            self.offset = offset
            self.parent = parent
            self.length = length

        # Array view protocol.
        def __len__(self):
            return self.length

        def __getitem__(self, key):
            return self.parent.sciRead(self.offset + key)
        
        def __setitem__(self, key, value):
            return self.parent.sciWrite(self.offset + key, value)

        # Iterator protocol.
        class __sciIterator:
            """SCI array iteratator helper class"""
            def __init__(self, parent):
                self.parent = parent
                self.key = 0
            
            def __iter__(self):
                return self

            def __next__(self):
                if (self.key >= self.parent.__len__()):
                    raise StopIteration
                value = self.parent[self.key]
                self.key += 1
                return value

        def __iter__(self):
            return self.__sciIterator(self)