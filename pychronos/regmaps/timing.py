import time
import pychronos
import logging
from . import ioInterface

class timing(pychronos.fpgamap):
    """Return a new map of the FPGA timing register space.
    
    Return a new map of the FPGA timing register space. This map provides
    structured read and write access to the registers which control the
    signals to the image sensor to generate the waveforms for frame timing
    and exposure control.
    
    Parameters
    ----------
    offset : `int`, optional
        Starting offset of the register map (FPGA_TIMING_BASE, by default)
    size : `int`, optional
        Length of the register map (FPGA_MAP_SIZE, by default)
    """
    def __init__(self, offset=pychronos.FPGA_TIMING_BASE, size=0x500):
        super().__init__(offset, size)

    @property
    def program(self):
        """Timing Program"""
        return pychronos.arrayview(self, offset=0x10, size=4, count=256)
    @program.setter
    def program(self, value):
        aview = pychronos.arrayview(self, offset=0x10, size=4, count=256)
        for i in range(0, len(value)):
            aview[i] = value[i]
    
    def __regprop(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        fset=lambda self, value: self.regWrite(offset, size, value),
                        doc = docstring)

    def __bitprop(offset, size, mask, docstring):
        return property(fget=lambda self: self.regRead(offset, size, mask),
                        fset=lambda self, value: self.regWrite(offset, size, value, mask),
                        doc = docstring)

    ## Register definitions
    id =            __regprop(0x00, 2, "Timing Module Identifier")
    version =       __regprop(0x02, 2, "Timing Module Version")
    subver =        __regprop(0x04, 2, "Timing Module Sub Version")
    status =        __regprop(0x06, 2, "Status Register")
    control =       __regprop(0x08, 2, "Control Register")
    pulsedAbnLowPeriod  = __regprop(0x0A, 2, "Pulsed-ABN low period")
    pulsedAbnHighPeriod = __regprop(0x0C, 2, "Pulsed-ABN high period")
    minLines            = __regprop(0x0E, 2, "Number of wavetable periods to delay when TIMING_WAIT_FOR_NLINES is used")
    
    # Status Register Bits
    inUseControlPage            = __bitprop(0x06, 2, 0x0008, 'which "page" in the control memory is currently active - the other one is being edited')
    exposureIsEnabled           = __bitprop(0x06, 2, 0x0040, 'if "1" either exposureEnabled is 1 or external shutter signal is enabled')
    frameRequestNotDone         = __bitprop(0x06, 2, 0x0080, 'when requestFrame called, this goes high until the request is completed')
    currentlyWaitingForActive   = __bitprop(0x06, 2, 0x0100, 'if "1" the timing generator is currently waiting for active signal level')
    currentlyWaitingForInactive = __bitprop(0x06, 2, 0x0200, 'if "1" the timing generator is currently waiting for inactive signal level')
    pageSwapState               = __bitprop(0x06, 2, 0xF000, 'state of the state machine that runs the copy on page flip')

    # Control Register Bits
    inhibitTiming       = __bitprop(0x08, 2, 0x0001, 'this enables the new timing engine core')
    requestFlip         = __bitprop(0x08, 2, 0x0002, 'indicates to the engine to flip on next reset or end of frame; self-clears')
    resetSignal         = __bitprop(0x08, 2, 0x0004, 'rising edge resets the internals')
    exposureEnabled     = __bitprop(0x08, 2, 0x0010, 'if enabled, timing engine will wait until exposure signal (level sensitive) or requestFrame (edge sensitive) goes high')
    exposure            = __bitprop(0x08, 2, 0x0020, 'if exposureEnabled is set, this enables the timing engine. If exposureEnabled is clear, this bit is ignored')
    requestFrame        = __bitprop(0x08, 2, 0x0080, 'if exposureEnabled is set, this will request a single frame from the timing engine')
    useAbnPulsedMode    = __bitprop(0x08, 2, 0x0100, 'use ABN pulsed mode')
    invertAbnPulsedMode = __bitprop(0x08, 2, 0x0200, 'invert the ABN signal when in pulsed mode')
    wavetableLatch      = __bitprop(0x08, 2, 0x0400, 'causes change to happen only on hsync period between wavetables')

    ABN         = 0x01000000
    ABN2        = 0x02000000
    TXN         = 0x08000000
    PRSTN       = 0x04000000
    IODRIVE     = 0x10000000
    NONE        = 0x00000000
    
    TIMING_RESTART           = 0x00000000
    TIMING_WAIT_FOR_ACTIVE   = 0x00FFFFFF
    TIMING_WAIT_FOR_INACTIVE = 0x00FFFFFE
    TIMING_WAIT_FOR_NLINES   = 0x00FFFFFC
    TIMING_MAX_CLOCKS        = 0x00FFFFFD
    
    @property
    def busy(self):
        """The state of the timing program page swap engine"""
        return self.pageSwapState != 0x1
    
    def reset(self):
        """Perform a reset of the timing engine"""
        self.resetSignal = 1
        # Reset is edge-sensitive, so best to disable reset when done.
        self.resetSignal = 0

    def waitForIdle(self, timeout=0.01):
        """Wait for the page flip engine to become idle"""
        start = time.time()
        while time.time() < (start + timeout):
            if not self.busy:
                return True
        return False

    def flip(self, timeout=0.01, force=False):
        """Activate the timing program by performing a page flip
        
        The timing program is double buffered so that the user can edit the current program
        in memory without affecting the currently executing program. When the user has finished
        editing, the program can be activated by requesting a page flip to swap their program
        with the one being executed by the FPGA.

        Parameters
        ----------
        timeout : `float`, optional
            How long to wait for the page flip to complete, in seconds. (default 0.01)
        """
        self.inhibitTiming = 0

        # Force a flip if the timeout is negative. This is guaranteed to flip, but may
        # result in a deadlocked sensor if the timing signals change at the wrong time.
        if (timeout < 0):
            self.requestFlip = 1
            self.reset()
            return

        # Request a flip of the timing program.
        self.requestFlip = 1
        if self.waitForIdle(timeout):
            return
        
        # Flip did not complete, is it waiting on a trigger event?
        if self.currentlyWaitingForActive or self.currentlyWaitingForInactive:
            logging.debug('flip state machine waiting on external trigger')
            io = ioInterface.ioInterface()
            origTrig = io.shutterTriggersFrame
            io.shutterTriggersFrame = False
            self.exposureEnabled = 0
            if self.waitForIdle(timeout):
                io.shutterTriggersFrame = origTrig
                return
            io.shutterTriggersFrame = origTrig
        
        # Flip is still deadlocked, the only thing left to do is force a reset
        logging.warning('flip state machine deadlocked - forcing a reset')
        self.reset()

    def runProgram(self, prog, timeout=0.01):
        """Load a program into the timing engine and run it.
        
        The timing program is double buffered so that the user can edit the current program
        in memory without affecting the currently executing program. When the user has finished
        editing, the program can be activated by requesting a page flip to swap their program
        with the one being executed by the FPGA.

        Parameters
        ----------
        prog : `list` of `int`
            The timing program to run, as a list of instructions.
        timeout : `float`, optional
            How long to wait for the page flip to complete, in seconds. (default 0.01)
        """
        self.waitForIdle(timeout)
        for pc in range(0, len(prog)):
            self.program[pc] = prog[pc]
        self.flip(timeout)
        self.decompile(prog)

    def decompile(self, prog=None):
        """Decompile a timing program and log some info about it.
        
        Parameters
        ----------
        prog : `list` of `int`, optional
            A timing program to decompile, as a list of instructions.
            (default: read the current from the FPGA)
        """
        if prog is None:
            prog = self.program
        
        # Setup some initial state and then parse the program.
        intClocks = 0
        frameClocks = 0
        for pc in range(0, len(prog)):
            cmd = prog[pc]

            # Assemble a flags field with the IO levels.
            iostr = ''
            iostr += 'A' if (cmd & self.ABN) else '-'
            iostr += 'a' if (cmd & self.ABN2) else '-'
            iostr += 'T' if (cmd & self.TXN) else '-'
            iostr += 'P' if (cmd & self.PRSTN) else '-'
            iostr += 'I' if (cmd & self.IODRIVE) else '-'

            delay = cmd & 0xffffff
            if (delay == self.TIMING_RESTART):
                break
            elif (delay == self.TIMING_WAIT_FOR_ACTIVE):
                logging.debug("%s wait(trig)", iostr)
            elif (delay == self.TIMING_WAIT_FOR_INACTIVE):
                logging.debug("%s wait(~trig)", iostr)
            elif (delay == self.TIMING_WAIT_FOR_NLINES):
                logging.debug("%s sync(%d)", iostr, self.minLines)
            else:
                if (cmd & self.ABN):
                    intClocks = 0
                elif not (cmd & self.TXN):
                    intClocks += delay
                frameClocks += delay
                logging.debug("%s wait(%d)" % (iostr, delay))
        
        logging.debug("frameClocks=%d intClocks=%d", frameClocks, intClocks)