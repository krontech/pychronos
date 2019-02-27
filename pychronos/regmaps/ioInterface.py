# IO system for Chronos high speed cameras
import pychronos

class ioInterface(pychronos.fpgamap):
    SOURCES = ['none', 'io1', 'io2', 'io3', 'comb',
               'software', 'delay', 'toggle', 'shutter',
               'recording', 'dispFrame',
               'startRec', 'endRec', 'nextSeg', 'timingIo',
               'alwaysHigh']
    
    SOURCENUMBERS = {'none':0, 'io1':1, 'io2':2, 'io3':3, 'comb':4,
                     'software':5, 'delay':6, 'toggle':7, 'shutter':8,
                     'recording':9, 'dispFrame':10,
                     'startRec':11, 'endRec':12, 'nextSeg':13, 'timingIo':14,
                     'alwaysHigh':15}
    SOURCENUMBERS = {'none':0, 'io1':1, 'io2':2, 'io3':3, 'comb':4,
                     'software':5, 'delay':6, 'toggle':7, 'shutter':8,
                     'recording':9, 'dispFrame':10,
                     'startRec':11, 'endRec':12, 'nextSeg':13, 'timingIo':14,
                     'alwaysHigh':15,
                     'NONE':0, 'IO1':1, 'IO2':2, 'IO3':3, 'COMB':4,
                     'SOFTWARE':5, 'DELAY':6, 'TOGGLE':7, 'SHUTTER':8,
                     'RECORDING':9, 'DISPFRAME':10,
                     'STARTREC':11, 'ENDREC':12, 'NEXTSEG':13, 'TIMING_IO':14,
                     'ALWAYSHIGH':15}

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
    
    def setPropertyBits(self, offset, size, bitOffset, nBits, value):
        self.regWrite(offset, size, (self.regRead(offset, size) & ~((2**nBits-1)<<bitOffset) |
                                     ((value & (2**nBits-1)) << bitOffset)))
    
    def __regprop(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        fset=lambda self, value: self.regWrite(offset, size, value),
                        doc = docstring)
    def __regprop_ro(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        doc = docstring)
    def __bitprop(offset, size, bitOffset, nBits, docstring):
        return property(fget=lambda self: ((self.regRead(offset, size) >> bitOffset) & (2**nBits-1)),
                        fset=lambda self, value: self.setPropertyBits(offset, size, bitOffset, nBits, value),
                        doc = docstring)
    def __bitprop_ro(offset, size, bitOffset, nBits, docstring):
        return property(fget=lambda self: ((self.regRead(offset, size) >> bitOffset) & (2**nBits-1)),
                        doc = docstring)

    def __srcprop(offset, docstring):
        return property(fget=lambda self: self.SOURCES[self.regRead(offset, 2) & 0xF],
                        fset=lambda self, value: self.regWrite(offset, 2, (self.regRead(offset, 2) & ~0xF) | value if (type(value) == int) else self.SOURCENUMBERS.get(value, 0)),
                        doc = docstring)

    identifier     = __regprop_ro(0x00, 4, 'ID to make sure the module is there')
    version_reg    = __regprop_ro(0x04, 4, 'whole number portion of version')
    subversion_reg = __regprop_ro(0x08, 4, 'decimal portion of version')
    # Control
    control        = __regprop(0x0C, 4,       'global control register')
    enable         = __bitprop(0x0C, 4, 0, 1, 'global enable of IO block')
    ioDeviceHold   = __bitprop(0x0C, 4, 3, 1, '... not implemented in FPGA but present')
    
    # Status
    status           = __regprop(0x10, 4,          'status')
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
    cpuIntSignalOut  = __bitprop_ro(0x10, 4, 17, 1, 'cpu interrupt signal output state')

    # raw channels
    channelStatus    = __regprop(0x14, 4,           'raw channel status')
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

    # Now for the input mapping
    io1SourceReg           = __regprop(0x20, 2,        'io1 driver source and flags')
    io1Source              = __srcprop(0x20,           'io1 source')
    io1InvertInput         = __bitprop(0x20, 2,  8, 1, 'invert flag')
    io1Debounce            = __bitprop(0x20, 2,  9, 1, 'debounce flag')
    io1DriveStrength       = __bitprop(0x20, 2, 12, 2, 'drive strength')
        
    io2SourceReg           = __regprop(0x22, 2,        'io2 driver source and flags')
    io2Source              = __srcprop(0x22,           'io2 source')
    io2InvertInput         = __bitprop(0x22, 2,  8, 1, 'invert flag')
    io2Debounce            = __bitprop(0x22, 2,  9, 1, 'debounce flag')
    io2DriveStrength       = __bitprop(0x22, 2, 12, 2, 'drive strength')
    
    combOr1SourceReg       = __regprop(0x24, 2,       'combOr1 driver source and flags')
    combOr1Source          = __srcprop(0x24,          'combOr1 source')
    combOr1InvertInput     = __bitprop(0x24, 2, 8, 1, 'invert flag')
    combOr1Debounce        = __bitprop(0x24, 2, 9, 1, 'debounce flag')
    
    combOr2SourceReg       = __regprop(0x26, 2,       'combOr2 driver source and flags')
    combOr2Source          = __srcprop(0x26,          'combOr2 source')
    combOr2InvertInput     = __bitprop(0x26, 2, 8, 1, 'invert flag')
    combOr2Debounce        = __bitprop(0x26, 2, 9, 1, 'debounce flag')
    
    combOr3SourceReg       = __regprop(0x28, 2,       'combOr3 driver source and flags')
    combOr3Source          = __srcprop(0x28,          'combOr3 source')
    combOr3InvertInput     = __bitprop(0x28, 2, 8, 1, 'invert flag')
    combOr3Debounce        = __bitprop(0x28, 2, 9, 1, 'debounce flag')
    
    combAndSourceReg       = __regprop(0x2A, 2,       'combAnd driver source and flags')
    combAndSource          = __srcprop(0x2A,          'combAnd source')
    combAndInvertInput     = __bitprop(0x2A, 2, 8, 1, 'invert flag')
    combAndDebounce        = __bitprop(0x2A, 2, 9, 1, 'debounce flag')
    
    combXOrSourceReg       = __regprop(0x2C, 2,       'combXOr driver source and flags')
    combXOrSource          = __srcprop(0x2C,          'combXOr source')
    combXOrInvertInput     = __bitprop(0x2C, 2, 8, 1, 'invert flag')
    combXOrDebounce        = __bitprop(0x2C, 2, 9, 1, 'debounce flag')
    
    delaySourceReg         = __regprop(0x2E, 2,       'delay driver source and flags')
    delaySource            = __srcprop(0x2E,          'delay source')
    delayInvertInput       = __bitprop(0x2E, 2, 8, 1, 'invert flag')
    delayDebounce          = __bitprop(0x2E, 2, 9, 1, 'debounce flag')
    
    toggleSetSourceReg     = __regprop(0x30, 2,       'toggleSet driver source and flags')
    toggleSetSource        = __srcprop(0x30,          'toggleSet source')
    toggleSetInvertInput   = __bitprop(0x30, 2, 8, 1, 'invert flag')
    toggleSetDebounce      = __bitprop(0x30, 2, 9, 1, 'debounce flag')
    
    toggleClearSourceReg   = __regprop(0x32, 2,       'toggleClear driver source and flags')
    toggleClearSource      = __srcprop(0x32,          'toggleClear source')
    toggleClearInvertInput = __bitprop(0x32, 2, 8, 1, 'invert flag')
    toggleClearDebounce    = __bitprop(0x32, 2, 9, 1, 'debounce flag')
    
    toggleFlipSourceReg    = __regprop(0x34, 2,       'toggleFlip driver source and flags')
    toggleFlipSource       = __srcprop(0x34,          'toggleFlip source')
    toggleFlipInvertInput  = __bitprop(0x34, 2, 8, 1, 'invert flag')
    toggleFlipDebounce     = __bitprop(0x34, 2, 9, 1, 'debounce flag')
    
    gateSourceReg          = __regprop(0x36, 2,       'gate driver source and flags')
    gateSource             = __srcprop(0x36,          'gate source')
    gateInvertInput        = __bitprop(0x36, 2, 8, 1, 'invert flag')
    gateDebounce           = __bitprop(0x36, 2, 9, 1, 'debounce flag')
    
    startSourceReg         = __regprop(0x38, 2,       'start driver source and flags')
    startSource            = __srcprop(0x38,          'start source')
    startInvertInput       = __bitprop(0x38, 2, 8, 1, 'invert flag')
    startDebounce          = __bitprop(0x38, 2, 9, 1, 'debounce flag')
    
    stopSourceReg          = __regprop(0x3A, 2,       'stop driver source and flags')
    stopSource             = __srcprop(0x3A,          'stop source')
    stopInvertInput        = __bitprop(0x3A, 2, 8, 1, 'invert flag')
    stopDebounce           = __bitprop(0x3A, 2, 9, 1, 'debounce flag')
    
    shutterSourceReg       = __regprop(0x3C, 2,       'shutter driver source and flags')
    shutterSource          = __srcprop(0x3C,          'shutter source')
    shutterInvertInput     = __bitprop(0x3C, 2, 8, 1, 'invert flag')
    shutterDebounce        = __bitprop(0x3C, 2, 9, 1, 'debounce flag')
    
    cpuIntSourceReg        = __regprop(0x3E, 2,       'cpuInt driver source and flags')
    cpuIntSource           = __srcprop(0x3E,          'cpuInt source')
    cpuIntInvertInput      = __bitprop(0x3E, 2, 8, 1, 'invert flag')
    cpuIntDebounce         = __bitprop(0x3E, 2, 9, 1, 'debounce flag')



    #------------------------------------------------------------------------------------------------------
    # Source register controls
    validSourceControlNames = ['io1', 'io2', 'combOr1', 'combOr2', 'combOr3', 'combAnd', 'combXOr', 'delay',
                               'toggleSet', 'toggleClear', 'toggleFlip', 'gate', 'start', 'stop', 'shutter', 'cpuInt']

    def setSourceConfiguration(self, name, structure):
        '''This helper function connects a given source to the signal with the given
        flags
        
        The 'name' field must be one of:
            io1, io2, combOr1, combOr2, combOr3, combAnd, combXOr, delay, toggleSet,
            toggleClear, toggleFlip, gate, start, stop, shutter or cpuInt

        the structure is a dictionary with the following optional fields:
        'source' - a string or integer stating which source is to be connected
            The following are valid: 'none', 'io1', 'io2', 'io3', 'comb',
               'software', 'delay', 'toggle', 'shutter', 'recording', 'dispFrame',
               'startRec', 'endRec', 'nextSeg', 'timingIo', 'alwaysHigh'
            Alternatively this can be a value of 0-15
        'invert' - is the input to be inverted
        'debounce' - is the debounce signal enabled
        'driveStrength' - only valid for 'io1' and 'io2' - this sets the drive
            strength on the IO
        '''
        prop = self.__class__.__dict__.get('%sSourceReg'%(name))
        if not prop:
            raise ValueError('name is not valid: %s' % (name))
        source = structure.get('source', 'NONE')
        raw = source if (type(source) == int) else self.SOURCENUMBERS.get(source, 0) 
        raw |= int(structure.get('invert',   0)) << 8
        raw |= int(structure.get('debounce', 0)) << 9
        if name in ['io1', 'io2']:
            raw |= int(structure.get('driveStrength', 0)) << 12
        prop.fset(self, raw)

    def getSourceConfiguration(self, name):
        '''This returns a dictionary containing the named signal and all parameters
        for it.

        'name' must be one of: 
            io1, io2, combOr1, combOr2, combOr3, combAnd, combXOr, delay, toggleSet,
            toggleClear, toggleFlip, gate, start, stop, shutter or cpuInt

        the returned dictionary matches the requirements for setSourceConfiguration.
        '''
        propSource        = self.__class__.__dict__.get('%sSource'%(name))
        propInvertInput   = self.__class__.__dict__.get('%sDebounce'%(name))
        propDebounce      = self.__class__.__dict__.get('%sInvertInput'%(name))
        propDriveStrength = self.__class__.__dict__.get('%sDriveStrength'%(name))
        if not propSource:
            raise ValueError('name is not valid: %s' % (name))

        structure = {}
        if (propSource):        structure['source']   = propSource.fget(self)
        if (propInvertInput):   structure['invert']   = propInvertInput.fget(self)
        if (propDebounce):      structure['debounce'] = propDebounce.fget(self)
        
        # special case, io
        if name in ['io1', 'io2']:
            if (propDriveStrength): structure['driveStrength'] = propDriveStrength.fget(self)

        # special case, tack on the delay config
        if name == 'delay':
            delayConfig = self.getDelayConfiguration()
            for key,value in delayConfig.items():
                structure[key] = value
        return structure

    #------------------------------------------------------------------------------------------------------
    # Input specific controls
    validInputControls = ['io1In', 'io2In']

    def setIoThreshold(self, name, threshold):
        '''Currently not implemented.

        TODO: add the stuff to change the PWM that generates these thresholds
        '''
        pass
    def getIoThreshold(self, name):
        '''Currently not implemented.

        TODO: add the stuff to change the PWM that generates these thresholds
        '''
        return 2.5
    

    def getInputConfiguration(self, name):
        '''Currently doesn't do anything but will be the way of getting the
        current IO Input thresholds.
        '''
        return {'threshold': self.getIoThreshold(name)}

    def setInputConfiguration(self, name, structure):
        '''Currently doesn't do anything but will be the way of setting the
        IO Input thresholds.
        '''
        threshold = structure.get('threshold', 2.5)
        self.setIoThreshold(name, threshold)

    #------------------------------------------------------------------------------------------------------
    # delay block controls
    validDelayControls = ['delay']
    
    delayControl      = __regprop(0x80, 2,       'delay block control bits')
    delayClockEnable  = __bitprop(0x80, 2, 0, 1, 'clock enable')
    delayOutputEnable = __bitprop(0x80, 2, 1, 1, 'output enable')
    delayFlush        = __bitprop(0x80, 2, 2, 1, 'flush the delay block - may be needed if long time bases are configured')
    delayDivider      = __regprop(0x84, 4,       'divider portion of clock prescaler')
    delayCount        = __regprop(0x88, 2,       'counter or upper part of fraction in clock prescaler')

    def setDelayConfiguration(self, structure):
        '''This configures the delay block prescaler and bucket bregade for delaying incoming signals.

        the structure must be a dictionary with the following keys:
        'delay' - float value in seconds of the total delay.

        Note: due to hardware the value may not exactly match the one given. The implementation uses
        a power of two divider search rather than a fractional search.
        '''
        value = structure.get('delay', 0.0)

        divider = 1
        delay = self.IO_BLOCK_FREQUENCY * value
        while delay > 2**14:
            delay //= 2
            divider *= 2

        self.delayClockEnable  = False
        self.delayOutputEnable = True
        self.delayFlush        = True
        self.delayDivider      = int(divider)
        self.delayCount        = int(delay)
        self.delayFlush        = False
        self.delayClockEnable  = True

    def getDelayConfiguration(self):
        '''This returns the current structure of the delay function bucket bregade and prescaler.
        
        The returned dictionary matches the input required for setDelayConfiguration.

        Note: due to being generated by what's in the FPGA, the delay value may not exactly
        match what was set
        '''
        return {'delay':(self.delayDivider * self.delayCount) / self.IO_BLOCK_FREQUENCY}
        
                               
    # some counters to help figure out what's going on
    io1TimeSinceRising       = __regprop_ro(0xC0, 4, 'time since rising edge')
    io1TimeSinceFaling       = __regprop_ro(0xC4, 4, 'time since falling edge')
    io2TimeSinceRising       = __regprop_ro(0xC8, 4, 'time since rising edge')
    io2TimeSinceFaling       = __regprop_ro(0xCc, 4, 'time since falling edge')
    startTimeSinceRising     = __regprop_ro(0xD0, 4, 'time since rising edge')
    startTimeSinceFaling     = __regprop_ro(0xD4, 4, 'time since falling edge')
    stopTimeSinceRising      = __regprop_ro(0xD8, 4, 'time since rising edge')
    stopTimeSinceFaling      = __regprop_ro(0xDC, 4, 'time since falling edge')
    shutterTimeSinceRising   = __regprop_ro(0xE0, 4, 'time since rising edge')
    shutterTimeSinceFaling   = __regprop_ro(0xE4, 4, 'time since falling edge')
    toggleTimeSinceRising    = __regprop_ro(0xE8, 4, 'time since rising edge')
    toggleTimeSinceFaling    = __regprop_ro(0xEC, 4, 'time since falling edge')
    interruptTimeSinceRising = __regprop_ro(0xF0, 4, 'time since rising edge')
    intteruptTimeSinceFaling = __regprop_ro(0xF4, 4, 'time since falling edge')
    
    
    @property
    def version(self):
        '''The version and subversion of the module within the FPGA.
        '''
        return '%d.%d'%(self.version_reg, self.subversion_reg)

    def getCapabilities(self):
        '''Once done this will return a structure that defines the capabilities of the IO block.
        
        Generally this will list what outputs or signals are available as well as what inputs
        they can be mapped to. As well, any special features of the signals will be listed (drive
        strength, for instance).

        This will also define special function blocks such as the delay line, combinatorial block
        and toggle block.
        '''
        return {'error':'Not yet implemented - poke otter'}
    
    def setConfiguration(self, structure):
        '''This takes a dictionary as it's input and configures the io block.

        This generally itterates through all keys in the dict and calls the set____Configuration
        functions for each of the blocks found.
        '''
        if type(structure) != dict:
            raise TypeError('structure needs to be a dict')
        for name, value in structure.items():
            if name in self.validSourceControlNames:
                self.setSourceConfiguration(name, value)
            elif name in validInputControls:
                self.setInputConfiguration(name, value)

            if name in validDelayControls: # this is both a valid source control and it's own thing (so no elif)
                self.configureDelay(value)

    def getConfiguration(self):
        '''This returns a dictionary of the current configuration of the IO block.

        This is very verbose as it lists all of the modules, signals and options. The structure
        returned is formatted to be able to be used on setConfiguration.
        '''
        structure = {}
        for name in self.validSourceControlNames:
            structure[name] = self.getSourceConfiguration(name)
        for name in self.validInputControls:
            structure[name] = self.getInputConfiguration(name)
        return structure
        
    
    def __init__(self, offset=0x6000, size=0x100):
        super().__init__(offset, size)


