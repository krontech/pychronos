# IO system for Chronos high speed cameras
import pychronos
import logging

class ioInterface(pychronos.fpgamap):
    """Return a new map of the FPGA IO register space.
    
    Return a new map of the FPGA IO register space. This map provides
    structured read and write access to the registers which control the
    internal and external IO signal routing commonly used for trigger
    and shutter control.

    Parameters
    ----------
    offset : `int`, optional
        Starting offset of the register map (FPGA_IO_BASE, by default)
    size : `int`, optional
        Length of the register map (FPGA_MAP_SIZE, by default)
    """
    def __init__(self, offset=pychronos.FPGA_IO_BASE, size=0x100):
        super().__init__(offset, size)
    
    # Mapping of source numbers to their signal names.
    SOURCES = [
        'none',
        'io1',
        'io2',
        'io3',
        'comb',
        'software',
        'delay',
        'toggle',
        'shutter',
        'recording',
        'dispFrame',
        'startRec',
        'endRec',
        'nextSeg',
        'timingIo',
        'alwaysHigh'
    ]
    
    # this is to use the same name as in the verilog
    SOURCE_NONE       =  0
    SOURCE_IO1        =  1
    SOURCE_IO2        =  2
    SOURCE_IO3        =  3
    SOURCE_COMB       =  4
    SOURCE_SOFTWARE   =  5
    SOURCE_DELAY      =  6
    SOURCE_TOGGLE     =  7
    SOURCE_SHUTTER    =  8
    SOURCE_RECORDING  =  9
    SOURCE_DISPFRAME  = 10
    SOURCE_STARTREC   = 11
    SOURCE_ENDREC     = 12
    SOURCE_NEXTSEG    = 13
    SOURCE_TIMING_IO  = 14
    SOURCE_ALWAYSHIGH = 15

    IO_BLOCK_FREQUENCY = 100000000
    
    def __regprop(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        fset=lambda self, value: self.regWrite(offset, size, value),
                        doc = docstring)
    def __regprop_ro(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        doc = docstring)
    def __bitprop(offset, size, bitOffset, nBits, docstring):
        return property(fget=lambda self: self.regRead(offset, size, (2**nBits-1) << bitOffset),
                        fset=lambda self, value: self.regWrite(offset, size, value, (2**nBits-1) << bitOffset),
                        doc = docstring)
    def __bitprop_ro(offset, size, bitOffset, nBits, docstring):
        return property(fget=lambda self: self.regRead(offset, size, (2**nBits-1) << bitOffset),
                        doc = docstring)

    id =            __regprop(0x00, 2, "IO Module Identifier")
    version =       __regprop(0x02, 2, "IO Module Version")
    subver =        __regprop(0x04, 2, "IO Module Sub Version")

    # Control Register
    control =       __regprop(0x0C, 4, "IO Module Control Register")
    enable         = __bitprop(0x0C, 4, 0, 1, 'global enable of IO block')
    ioDeviceHold   = __bitprop(0x0C, 4, 3, 1, '... not implemented in FPGA but present')
    
    # Status
    status =        __regprop(0x10, 4, "IO Module Status Register")
    io1_SignalOut    = __bitprop_ro(0x10, 4, 0, 1, 'io1 output level - may not be driving if no drive-strength is enabled')
    io2_SignalOut    = __bitprop_ro(0x10, 4, 1, 1, 'io2 output level - may not be driving if no drive-strength is enabled')
    combSignalOut    = __bitprop_ro(0x10, 4, 2, 1, 'combinatorial block output state')
    delaySignalOut   = __bitprop_ro(0x10, 4, 3, 1, 'delay block output state')
    combOr1SignalOut = __bitprop_ro(0x10, 4, 4, 1, 'state as seen by the combinatorial Or1 input')
    combOr2SignalOut = __bitprop_ro(0x10, 4, 5, 1, 'state as seen by the combinatorial Or2 input')
    combOr3SignalOut = __bitprop_ro(0x10, 4, 6, 1, 'state as seen by the combinatorial Or3 input')
    combAndSignalOut = __bitprop_ro(0x10, 4, 8, 1, 'state as seen by the combinatorial And input')
    combXOrSignalOut = __bitprop_ro(0x10, 4, 9, 1, 'state as seen by the combinatorial XOr input')
    startSignalOut   = __bitprop_ro(0x10, 4, 12, 1, 'start signal output state')
    stopSignalOut    = __bitprop_ro(0x10, 4, 13, 1, 'stop signal output state')
    gateSignalOut    = __bitprop_ro(0x10, 4, 14, 1, 'gate signal output state')
    toggleSignalOut  = __bitprop_ro(0x10, 4, 15, 1, 'toggle block signal output state')
    shutterSignalOut = __bitprop_ro(0x10, 4, 16, 1, 'shutter signal output state')
    triggerSignalOut = __bitprop_ro(0x10, 4, 17, 1, 'trigger signal output state')

    # raw channels
    channel =       __regprop(0x14, 4, "IO Module Channel Status Register")
    sourceNone       = __bitprop_ro(0x14, 4,  0, 1, 'always off channel (always 0)')
    sourceIo1        = __bitprop_ro(0x14, 4,  1, 1, 'input 1')
    sourceIo2        = __bitprop_ro(0x14, 4,  2, 1, 'input 2')
    sourceIo3        = __bitprop_ro(0x14, 4,  3, 1, 'input 3')
    sourceComb       = __bitprop_ro(0x14, 4,  4, 1, 'combinatorial output channel')
    sourceSoftware   = __bitprop_ro(0x14, 4,  5, 1, 'software controlled channel - currently does nothing')
    sourceDelay      = __bitprop_ro(0x14, 4,  6, 1, 'delay block output channel')
    sourceToggle     = __bitprop_ro(0x14, 4,  7, 1, 'toggle block output channel')
    sourceShutter    = __bitprop_ro(0x14, 4,  8, 1, 'shutter signal from old timing block')
    sourceRecording  = __bitprop_ro(0x14, 4,  9, 1, 'sequencer recording state')
    sourceDispFrame  = __bitprop_ro(0x14, 4, 10, 1, 'has just started drawing frame to CPU')
    sourceStartRec   = __bitprop_ro(0x14, 4, 11, 1, 'Just started a recording')
    sourceEndRec     = __bitprop_ro(0x14, 4, 12, 1, 'Recording just stopped')
    sourceNextSeg    = __bitprop_ro(0x14, 4, 13, 1, 'sequencer just moved to a new segment')
    sourceTimingIo   = __bitprop_ro(0x14, 4, 14, 1, 'ioDriver output from sensor timing block (the new one)')
    sourceAlwaysHigh = __bitprop_ro(0x14, 4, 15, 1, 'always on channel (always 1)')

    # IO Source class
    class ioSource:
        def __init__(self, parent, offset):
            self.offset = offset
            self.parent = parent
        
        @property
        def source(self):
            """IO source mapping register"""
            return self.parent.SOURCES[self.parent.regRead(self.offset, 2, mask=0xF)]
        @source.setter
        def source(self, value):
            if (isinstance(value, str)):
                value = self.parent.SOURCES.index(value.lower())
            self.parent.regWrite(self.offset, 2, value, mask=0xF)
        
        @property
        def invert(self):
            """IO invert enable"""
            return self.parent.regRead(self.offset, 2, mask=0x0100)
        @invert.setter
        def invert(self, value):
            self.parent.regWrite(self.offset, 2, value, mask=0x0100)
        
        @property
        def debounce(self):
            """IO debounce enable"""
            return self.parent.regRead(self.offset, 2, mask=0x0200)
        @debounce.setter
        def debounce(self, value):
            self.parent.regWrite(self.offset, 2, value, mask=0x0200)
        
    # IO Source class for output pins
    class pinSource(ioSource):
        def __init__(self, parent, offset):
            super().__init__(parent, offset)
        
        @property
        def drive(self):
            """IO drive strength"""
            return self.parent.regRead(self.offset, 2, mask=0x3000)
        @drive.setter
        def drive(self, value):
            self.parent.regWrite(self.offset, 2, value, mask=0x3000)

    @property
    def io1(self):
        """IO1 output confiruation registers"""
        return self.pinSource(self, 0x20)
    
    @property
    def io2(self):
        """IO2 output confiruation registers"""
        return self.pinSource(self, 0x22)
    
    @property
    def combOr1(self):
        """Combinatorial block OR input 1 configuration"""
        return self.ioSource(self, 0x24)
    
    @property
    def combOr2(self):
        """Combinatorial block OR input 2 configuration"""
        return self.ioSource(self, 0x26)

    @property
    def combOr3(self):
        """Combinatorial block OR input 3 configuration"""
        return self.ioSource(self, 0x28)

    @property
    def combAnd(self):
        """Combinatorial block AND input configuration"""
        return self.ioSource(self, 0x2A)
    
    @property
    def combXor(self):
        """Combinatorial block XOR input configuration"""
        return self.ioSource(self, 0x2C)

    @property
    def delay(self):
        """Programmable delay block input configuration"""
        return self.ioSource(self, 0x2E)
    
    @property
    def toggleSet(self):
        """Toggle block SET input configuration"""
        return self.ioSource(self, 0x30)
    
    @property
    def toggleClear(self):
        """Toggle block CLEAR input configuration"""
        return self.ioSource(self, 0x32)
    
    @property
    def toggleFlip(self):
        """Toggle block FLIP input configuration"""
        return self.ioSource(self, 0x34)
    
    @property
    def gate(self):
        """Gate input signal configuration"""
        return self.ioSource(self, 0x36)
    
    @property
    def start(self):
        """Start Recording signal configuration"""
        return self.ioSource(self, 0x38)
    
    @property
    def stop(self):
        """Stop Recording signal configuration"""
        return self.ioSource(self, 0x3A)
    
    @property
    def shutter(self):
        """Shutter control signal configuration"""
        return self.ioSource(self, 0x3C)
    
    @property
    def trigger(self):
        """Recording Trigger signal configuration"""
        return self.ioSource(self, 0x3E)

    @property
    def shutterTriggersFrame(self):
        """Enable shutter gating signal to the timing engine"""
        return self.regRead(0x3c, 2, mask=0x1000)
    @shutterTriggersFrame.setter
    def shutterTriggersFrame(self, value):
        oldtrig = pychronos.regmaps.trigger()
        self.regWrite(0x3c, 2, bool(value), mask=0x1000)
        oldtrig.extShutter = 2 if value else 0

    #------------------------------------------------------------------------------------------------------
    # delay block controls
    delayControl      = __regprop(0x80, 2,       'delay block control bits')
    delayClockEnable  = __bitprop(0x80, 2, 0, 1, 'clock enable')
    delayOutputEnable = __bitprop(0x80, 2, 1, 1, 'output enable')
    delayFlush        = __bitprop(0x80, 2, 2, 1, 'flush the delay block - may be needed if long time bases are configured')
    delayDivider      = __regprop(0x84, 4,       'divider portion of clock prescaler')
    delayCount        = __regprop(0x88, 2,       'counter or upper part of fraction in clock prescaler')

    @property
    def delayTime(self):
        return (self.delayDivider * self.delayCount) / self.IO_BLOCK_FREQUENCY
    @delayTime.setter
    def delayTime(self, value):
        divider = 1
        delay = self.IO_BLOCK_FREQUENCY * value
        while delay > 2**14:
            delay //= 2
            divider *= 2

        self.delayClockEnable  = False
        self.delayOutputEnable = True
        #self.delayFlush        = True
        self.delayDivider      = int(divider)
        self.delayCount        = int(delay)
        #self.delayFlush        = False
        self.delayClockEnable  = True
                       
    # some counters to help figure out what's going on
    io1TimeSinceRising        = __regprop_ro(0xC0, 4, 'time since rising edge')
    io1TimeSinceFalling       = __regprop_ro(0xC4, 4, 'time since falling edge')
    io2TimeSinceRising        = __regprop_ro(0xC8, 4, 'time since rising edge')
    io2TimeSinceFalling       = __regprop_ro(0xCc, 4, 'time since falling edge')
    startTimeSinceRising      = __regprop_ro(0xD0, 4, 'time since rising edge')
    startTimeSinceFalling     = __regprop_ro(0xD4, 4, 'time since falling edge')
    stopTimeSinceRising       = __regprop_ro(0xD8, 4, 'time since rising edge')
    stopTimeSinceFalling      = __regprop_ro(0xDC, 4, 'time since falling edge')
    shutterTimeSinceRising    = __regprop_ro(0xE0, 4, 'time since rising edge')
    shutterTimeSinceFalling   = __regprop_ro(0xE4, 4, 'time since falling edge')
    toggleTimeSinceRising     = __regprop_ro(0xE8, 4, 'time since rising edge')
    toggleTimeSinceFalling    = __regprop_ro(0xEC, 4, 'time since falling edge')
    interruptTimeSinceRising  = __regprop_ro(0xF0, 4, 'time since rising edge')
    interruptTimeSinceFalling = __regprop_ro(0xF4, 4, 'time since falling edge')
    