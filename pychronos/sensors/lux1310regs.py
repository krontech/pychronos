# LUX1310 SCI Register Definitions
import pychronos
from pychronos.regmaps import sensor

class adcArrayView(sensor.sciArrayView):
    """ADC offset arrayview helper class"""
    def __getitem__(self, key):
        value = self.parent.sciRead(self.offset + key)
        if (value & 0x400):
            return -(value & 0x3ff)
        else:
            return value

    def __setitem__(self, key, value):
        if (value < 0):
            value = int(-value) & 0x3ff
            self.parent.sciWrite(self.offset + key, value | 0x400)
        else:
            value = int(value) & 0x3ff
            self.parent.sciWrite(self.offset + key, value)

class lux1310regs(sensor):
    """Return a new map of the image sensor register space.
    
    Return a new map of the LUX1310 image sensor register space. This map
    provides structured read and write access to the registers via the SCI
    communication channel.
    
    Parameters
    ----------
    offset : `int`, optional
        Starting offset of the register map (FPGA_SENSOR_BASE, by default)
    size : `int`, optional
        Length of the register map (FPGA_MAP_SIZE, by default)
    """
    def __init__(self, offset=pychronos.FPGA_SENSOR_BASE, size=0x100):
        super().__init__(offset, size)
    
    # Helper to define an SCI register as a property.
    def __sciprop(offset, mask, docstring):
        return property(fget=lambda self: self.sciRead(offset, mask),
                        fset=lambda self, value: self.sciWrite(offset, value, mask),
                        doc = docstring)
    
    @property
    def reg(self):
        """Raw image sensor registers"""
        return sensor.sciArrayView(self, 0x00, 0x7F)
    
    @property
    def regAdcOs(self):
        """ADC offsets"""
        return adcArrayView(self, 0x3A, 16)
    
    revChip =           __sciprop(0x00, 0x00ff, "Chip Revision Number")
    regChipId =         __sciprop(0x00, 0xff00, "Chip ID Number")
    regTimingEn =       __sciprop(0x01, 0x0001, "Enable timing engine access to wavetable")
    regSofDelay =       __sciprop(0x02, 0x00ff, "Delay from TXN rising edge to start of frame")
    regHblank =         __sciprop(0x03, 0xffff, "Horizontal blanking period")
    regRoiNb =          __sciprop(0x04, 0x0007, "Number of regions of interest when in ROI mode")
    regRoiEn =          __sciprop(0x04, 0x0008, "Enable regions of interest")
    regDrkColRd =       __sciprop(0x04, 0x0100, "Enable dark column readout")
    regVflip =          __sciprop(0x04, 0x1000, "Vertical image flip")
    regHflip =          __sciprop(0x04, 0x2000, "Horizontal image flip")
    regXstart =         __sciprop(0x05, 0x07ff, "Start of standard window (X direction)")
    regXend =           __sciprop(0x06, 0x07ff, "End of standard window (X direction)")
    regYstart =         __sciprop(0x07, 0x07ff, "Start of standard window (Y direction)")
    regYend =           __sciprop(0x08, 0x07ff, "End of standard window (Y direction)")
    ## TODO: Need special arrayview for ROI sections.
    regDrkRowsStAddr =  __sciprop(0x29, 0x07ff, "Address of the first dark row to readout")
    regNbDrkRows =      __sciprop(0x29, 0xf000, "Number of dark rows to output")
    regNextRowAddrOvr = __sciprop(0x2A, 0x07ff, "Specify next row address to be read out")
    regNextRowOvrEn =   __sciprop(0x2A, 0x1000, "Enable next row to be specified through SCI")
    regInterRoiSp =     __sciprop(0x2B, 0x0001, "Add a clock of hblack at ROI transitions")
    regClkSelScip =     __sciprop(0x2C, 0x0003, "Select clock frequency for SCI interface")
    regClkSelFt =       __sciprop(0x2C, 0x000c, "Select clock frequency for frame timer")
    regFtTrigNbPulse =  __sciprop(0x31, 0xffff, "Number of TXN pulses per frame")
    regFtRstNbPulse =   __sciprop(0x34, 0xffff, "Number of ABN pulses per frame")
    regAbn2En =         __sciprop(0x35, 0x0010, "Enable ABN2 usage for odd row exposure")
    regRdoutDly =       __sciprop(0x37, 0xffff, "Delay from start of row to start of readout")
    regAdcCalEn =       __sciprop(0x39, 0x0001, "Enable ADC offset registers during vertical blanking")
    regAdcOsSeqWidth =  __sciprop(0x4A, 0x03ff, "Time to spend applying each of the ADC offsets")
    regPclkLinevalid =  __sciprop(0x4B, 0x0fff, "PCLK channel output when valid pixels")
    regPclkVblank =     __sciprop(0x4C, 0x0fff, "PCLK channel output during vertical blanking")
    regPclkHblank =     __sciprop(0x4D, 0x0fff, "PCLK channel output during horizontal blanking")
    regPclkOpticalBlack = __sciprop(0x4E, 0x0fff, "PCLK channel output when output is dark pixels")
    regMono =           __sciprop(0x4F, 0x0001, "Monochrome/color selection (binning mode only)")
    regRow2En =         __sciprop(0x4F, 0x0010, "Enable 2-row averaging")
    regPoutsel =        __sciprop(0x50, 0x0003, "Pixel output select (test mode)")
    regInvertAnalog =   __sciprop(0x50, 0x0010, "Invert polarity of signal chain")
    regGlobalShutter =  __sciprop(0x50, 0x0100, "Global shutter mode")
    regGainSelSamp =    __sciprop(0x51, 0x0fff, "Gain selection sampling cap")
    regGainSelFb =      __sciprop(0x52, 0x007f, "Gain selection feedback cap")
    regGainBit =        __sciprop(0x53, 0x0007, "Serial gain")
    regColbin2 =        __sciprop(0x55, 0x0001, "2-column binning")
    regTstPat =         __sciprop(0x56, 0x0003, "Test pattern selection")
    regCustPat =        __sciprop(0x57, 0x0fff, "Custom pattern for ADC training pattern")
    regMuxMode =        __sciprop(0x58, 0x0003, "ADC channel multiplex mode")
    regPwrEnSerializerB = __sciprop(0x59, 0xffff, "Power enable for LVDS channels 15 to 0")
    regDacIlv =         __sciprop(0x5A, 0x0007, "Current control for LVDS data channels")
    regMsbFirstData =   __sciprop(0x5A, 0x0008, "Output MSB first for data channels")
    regPclkInv  =       __sciprop(0x5A, 0x0010, "Invert PCLK output")
    regTermbData =      __sciprop(0x5A, 0x0020, "Enable on-chip termination resistor for data channels")
    regDclkInv =        __sciprop(0x5A, 0x0040, "Invert DCLK output")
    regTermbClk =       __sciprop(0x5A, 0x0080, "Enable on-chip termination resistor for clock channel")
    regTermbRxclk =     __sciprop(0x5B, 0x1000, "Enable on-chip termination resistor for serializer")
    regPwrenDclkB =     __sciprop(0x60, 0x0001, "DCLK channel power enable (active low)")
    regPwrenPclkB =     __sciprop(0x60, 0x0002, "PCLK channel power enable (active low)")
    regPwrenBiasB =     __sciprop(0x60, 0x0004, "Bias generator power enable (active low)")
    regPwrenDrvB =      __sciprop(0x60, 0x0008, "Analog drivers power enable (active low)")
    regPwrenAdcB =      __sciprop(0x60, 0x0010, "ADC power enable (active low)")
    regSelVcmi =        __sciprop(0x62, 0x000f, "Reserved")
    regSelVcmo =        __sciprop(0x62, 0x00f0, "Reserved")
    regSelVcmp =        __sciprop(0x62, 0x0f00, "ADC positive reference voltage setting")
    regSelVcmn =        __sciprop(0x62, 0xf000, "ADC negative reference voltage setting")
    regSelVdumrst =     __sciprop(0x63, 0x03e0, "Electrical signal levels for gain calibration")
    regHidyEn =         __sciprop(0x67, 0x0001, "Enable high dynamic range operation")
    regHidyTrigNbPulse = __sciprop(0x68, 0xffff, "Number of PRSTN pulses per frame before triggering HIDY timing")
    regSelVdr1Width =   __sciprop(0x69, 0xffff, "Width of the first HDR voltage pulse")
    regSelVdr2Width =   __sciprop(0x6A, 0xffff, "Width of the second HDR voltage pulse")
    regSelVdr3Width =   __sciprop(0x6B, 0xffff, "Width of the third HDR voltage pulse")
    regLvDelay =        __sciprop(0x71, 0x003f, "Linevalid delay to match ADC latency")
    regSerSync =        __sciprop(0x7c, 0x0001, "Synchronized the serializers")
    regClkSync =        __sciprop(0x7d, 0x0001, "Re-synchronizes the SCIP clock divider")
    regSresetB =        __sciprop(0x7e, 0x0001, "Soft reset: resets all registers")
    # Some undocumented registers.
    regStateIdleCtrl0 = __sciprop(0x2d, 0xffff, "Idle control register 0")
    regStateIdleCtrl1 = __sciprop(0x2e, 0xffff, "Idle control register 1")
    regStateIdleCtrl2 = __sciprop(0x2d, 0xffff, "Idle control register 2")
    regAdcClockCtrl =   __sciprop(0x5c, 0xffff, "ADC clock control register")
    regIntClockTiming = __sciprop(0x74, 0xffff, "Internal clock timing register")
    regWavetableSize =  __sciprop(0x7a, 0xffff, "Wavetable size")

    def wavetable(self, wavetab):
        """Helper function to write a wavetable into the image sensor registers
        using the Serial Communication Interface protocol.

        Parameters
        ----------
        wavetab : `bytes`
            Wavetable to write.
        """
        # Clear RW and reset the FIFO.
        self.sciControl = self.SCI_RESET
        self.sciAddress = 0x7F
        self.sciDataLen = len(wavetab)
        for b in wavetab:
            self.sciFifoWrite = int(b)

        # Start the write and wait for completion.
        self.sciControl = self.SCI_RUN
        while (self.sciControl & self.SCI_RUN) != 0:
            pass
