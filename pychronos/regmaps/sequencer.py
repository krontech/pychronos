import pychronos

class seqcommand:
    # Block termination flags
    BLK_TERM_RISING = (1 << 7)
    BLK_TERM_FALLING = (1 << 6)
    BLK_TERM_HIGH = (1 << 5)
    BLK_TERM_LOW = (1 << 4)
    BLK_TERM_FULL = (1 << 3)

    # Recording Termination flags
    REC_TERM_BLOCK = (1 << 2)
    REC_TERM_MEMORY = (1 << 1)
    REC_TERM_TRIGGER = (1 << 0)

    def __init__(self, command=0, **kwargs):
        """Create a program sequencer command for controlling the acquisition of
        frames from the image sensor.
        
        Parameters
        ----------
        command : `int`, optional
            Raw integer command value (default zero)
        
        Keyword Arguments
        -----------------
        blockSize : `int`, optional
            Number of frames in a block
        nextState : `int`, optional
            Jump to this address on completion of this block
        blkTermRising : `bool`, optional
            Enable block terimation on rising edge
        blkTermFalling : `bool`, optional
            Enable block termination on falling edge
        blkTermHigh : `bool`, optional
            Enable block termination on high trigger
        blkTermLow : `bool`, optional
            Enable block termination on low trigger
        blkTermFull : `bool`, optional
            Enable block termination when the block becomes full
        recTermBlockEnd : `bool`, optional
            Enable recording termination when the block terminates
        recTermMemory : `bool`, optional
            Enable recording termination when recording memory becomes full
        recTermTrigger : `bool`, optional
            Enable record termination on block termination trigger
        """
        self.value = int(command)
        if ("blockSize" in kwargs):
            self.blockSize = kwargs["blockSize"]
        if ("nextState" is kwargs):
            self.nextState = kwargs["nextState"]
        if ("flags" in kwargs):
            self.flags = kwargs["flags"]
        if ("blkTermRising" in kwargs):
            self.blkTermRising = kwargs["blkTermRising"]
        if ("blkTermFalling" in kwargs):
            self.blkTermFalling = kwargs["blkTermFalling"]
        if ("blkTermHigh" in kwargs):
            self.blkTermHigh = kwargs["blkTermHigh"]
        if ("blkTermLow" in kwargs):
            self.blkTermLow = kwargs["blkTermLow"]
        if ("blkTermFull" in kwargs):
            self.blkTermFull = kwargs["blkTermFull"]
        if ("recTermBlockEnd" in kwargs):
            self.recTermBlockEnd = kwargs["recTermBlockEnd"]
        if ("recTermMemory" in kwargs):
            self.recTermMemory = kwargs["recTermMemory"]
        if ("recTermTrigger" in kwargs):
            self.recTermTrigger = kwargs["recTermTrigger"]
    
    def __repr__(self):
        result = "seqcommand(blockSize=%s, nextState=%s,\n" % (self.blockSize, self.nextState)
        result += "\tblkTermRising=%s, blkTermFalling=%s,\n" % (self.blkTermRising, self.blkTermFalling)
        result += "\tblkTermHigh=%s, blkTermLow=%s, blkTermFull=%s,\n" % (self.blkTermHigh, self.blkTermLow, self.blkTermFull)
        result += "\trecTermBlockEnd=%s, recTermMemory=%s, recTermTrigger=%s)" % (self.recTermBlockEnd, self.recTermMemory, self.recTermTrigger)
        return result

    @property
    def blockSize(self):
        return ((self.value & 0xffffffff000) >> 12) + 1
    @blockSize.setter
    def blockSize(self, value):
        if ((value == 0) or (value > (1 << 32))):
            raise ValueError
        self.value &= ~0xffffffff000
        self.value |= (value - 1) << 12

    #--------------------------------------------
    # Getters and Setters for Command Components
    #--------------------------------------------
    @property
    def flags(self):
        return (self.value & 0xff)
    @flags.setter
    def flags(self, value):
        self.value &= ~0xff
        self.value |= (value & 0xff)
    
    @property
    def nextState(self):
        return (self.value & 0xf00) >> 8
    @nextState.setter
    def nextState(self, value):
        self.value &= ~0xf00
        self.value |= (value << 8) & 0xf00
    
    def __setflags(self, enable, flag):
        if (enable):
            self.value |= flag
        else:
            self.value &= ~flag

    @property
    def blkTermRising(self):
        """Enable block terimation on rising edge"""
        return (self.value & self.BLK_TERM_RISING) != 0
    @blkTermRising.setter
    def blkTermRising(self, value):
        self.__setflags(value, self.BLK_TERM_RISING)
    
    @property
    def blkTermFalling(self):
        """Enable block terimation on falling edge"""
        return (self.value & self.BLK_TERM_FALLING) != 0
    @blkTermFalling.setter
    def blkTermFalling(self, value):
        self.__setflags(value, self.BLK_TERM_FALLING)
    
    @property
    def blkTermHigh(self):
        """Enable block terimation on high trigger level"""
        return (self.value & self.BLK_TERM_HIGH) != 0
    @blkTermHigh.setter
    def blkTermHigh(self, value):
        self.__setflags(value, self.BLK_TERM_HIGH)
    
    @property
    def blkTermLow(self):
        """Enable block terimation on low trigger level"""
        return (self.value & self.BLK_TERM_LOW) != 0
    @blkTermLow.setter
    def blkTermLow(self, value):
        self.__setflags(value, self.BLK_TERM_LOW)

    @property
    def blkTermFull(self):
        """Enable block terminatio when the block becomes full"""
        return (self.value & self.BLK_TERM_FULL) != 0
    @blkTermFull.setter
    def blkTermFull(self, value):
        self.__setflags(value, self.BLK_TERM_FULL)
    
    @property
    def recTermBlockEnd(self):
        """Enable recording termination when the block terminates"""
        return (self.value & self.REC_TERM_BLOCK) != 0
    @recTermBlockEnd.setter
    def recTermBlockEnd(self, value):
        self.__setflags(value, self.REC_TERM_BLOCK)

    @property
    def recTermMemory(self):
        """Enable recording termination when memory becomes full"""
        return (self.value & self.REC_TERM_MEMORY) != 0
    @recTermMemory.setter
    def recTermMemory(self, value):
        self.__setflags(value, self.REC_TERM_MEMORY)
    
    @property
    def recTermTrigger(self):
        # TODO: This docstring doesn't make any sense???
        """Enable recording termination on block termination"""
        return (self.value & self.REC_TERM_TRIGGER) != 0
    @recTermTrigger.setter
    def recTermTrigger(self, value):
        self.__setflags(value, self.REC_TERM_TRIGGER)
    
    #--------------------------------------------
    # Python Number Protocol
    #--------------------------------------------
    def __int__(self):
        return self.value
    
    def __lshift__(self, other):
        return seqcommand(self.value, flags=(self.flags << int(other)))
    
    def __rshift__(self, other):
        return seqcommand(self.value, flags=(self.flags >> int(other)))
    
    def __and__(self, other):
        return seqcommand(self.value, flags=(self.flags & int(other)))
    
    def __xor__(self, other):
        return seqcommand(self.value, flags=(self.flags ^ int(other)))
    
    def __or__(self, other):
        return seqcommand(self.value, flags=(self.flags | int(other)))
    
    def __invert__(self):
        return seqcommand(self.value, flags=~self.flags)
    
    def __abs__(self):
        return seqcommand(self.value)

class sequencer(pychronos.fpgamap):
    """Return a new map of the FPGA sequencer register space.
        
    This map provides structured read and write access to the registers
    which control the acquisition and storage of frames from the image
    sensor.

    Parameters
    ----------
    offset : `int`, optional
        Starting offset of the register map (FPGA_SEQUENCER_BASE, by default)
    size : `int`, optional
        Length of the register map (FPGA_MAP_SIZE, by default)"
    """
    def __init__(self, offset=pychronos.FPGA_SEQUENCER_BASE, size=(0x200 - pychronos.FPGA_SEQUENCER_BASE)):
        super().__init__(offset, size)

        # Live readout frame results.
        self.__result = None

    def __regprop(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        fset=lambda self, value: self.regWrite(offset, size, value),
                        doc = docstring)

    # Define simple registers
    control =       __regprop(0x00, 2, "Control Register")
    status =        __regprop(0x04, 2, "Status Register")
    frameSize =     __regprop(0x08, 4, "Frame Size Register")
    regionStart =   __regprop(0x0C, 4, "Recording Region Start Address")
    regionStop =    __regprop(0x10, 4, "Recording Region End Address")
    trigDelay =     __regprop(0x20, 4, "Trigger Delay Register")
    mdFifo =        __regprop(0x24, 4, "Recording Sequencer FIFO Readout Register")
    writeAddr =     __regprop(0x34, 4, "Current Frame Write Address")
    lastAddr =      __regprop(0x38, 4, "Last Frame Write Address")
    # Control Register Constants
    SW_TRIG =       (1 << 0)
    START_REC =     (1 << 1)
    STOP_REC =      (1 << 2)
    TRIG_DELAY =    (1 << 3)
    # Status Register Constants
    ACTIVE_REC =    (1 << 0)
    FIFO_EMPTY =    (1 << 1)
    
    @property
    def liveAddr(self):
        """Live Display Addresses"""
        return self.regArray(offset=0x14, size=4, count=3)
    
    @property
    def program(self):
        """Sequencer Program Registers"""
        return self.regArray(offset=pychronos.FPGA_SEQPROGRAM_BASE - pychronos.FPGA_SEQUENCER_BASE, size=8, count=16)

    @property
    def liveResult(self):
        """Result of the frame readout operation, or None if in progress"""
        return self.__result

    ## FIXME: Does this require locking or other mutual exclusion mechanisms to
    ## ensure multiple readout functions don't collide and do bad things?
    def startLiveReadout(self, hRes, vRes):
        """Begin readout of a frame from the live display buffer.

        This helper function is typically used during calibration routines to
        extract frames without interrupting the live display or recording
        sequencer. The frame is returned to the caller via a callback function,
        with the frame as its argument.

        Parameters
        ----------
        hRes : int
            The horizontal resolution of image data to read out.
        vRes : int
            The vertical resolution of image data to read out.

        Returns
        -------
        generator (float)
            Sleep time, in seconds, between steps of the readout proceedure.
        """
        backup = [self.liveAddr[0], self.liveAddr[1], self.liveAddr[2]]
        address = self.writeAddr
        self.__result = None

        # Remove the currently-writing frame from the live buffer, or do
        # nothing if not writing to the live display region (eg: recording)
        if address == backup[0]:
            self.liveAddr[0] = backup[1]
        elif address == backup[1]:
            self.liveAddr[1] = backup[2]
        elif address == backup[2]:
            self.liveAddr[2] = backup[0]
        
        # Wait for the sequencer to begin writing to somewhere else.
        while (self.writeAddr == address):
            yield 0.001 # 1ms
        
        # Readout the frame and restore the live display configuration.
        self.__result = pychronos.readframe(address, hRes, vRes)
        self.liveAddr[0] = backup[0]
        self.liveAddr[1] = backup[1]
        self.liveAddr[2] = backup[2]
