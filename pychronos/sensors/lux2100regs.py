# LUX1310 SCI Register Definitions
import pychronos
from pychronos.regmaps import sensor

class lux2100regs(sensor):
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

    # Helper to read a register after a bank switch
    def sciBankRead(self, offset, bank, mask=0):
        self.sciWrite(0x04, bank, 0)
        return self.sciRead(offset, mask)
    
    def sciBankWrite(self, offset, bank, value, mask=0):
        self.sciWrite(0x04, bank, 0)
        self.sciWrite(offset, value, mask)
    
    # Helper to define an SCI register as a property.
    def __sensorprop(offset, mask, docstring):
        return property(fget=lambda self: self.sciBankRead(offset, 0, mask),
                        fset=lambda self, value: self.sciBankWrite(offset, 0, value, mask),
                        doc = docstring)
    
    def __dataprop(offset, mask, docstring):
        return property(fget=lambda self: self.sciBankRead(offset, 1, mask),
                        fset=lambda self, value: self.sciBankWrite(offset, 1, value, mask),
                        doc = docstring)

    revChip =           __sensorprop(0x00, 0x00ff, "Chip Revision Number")
    regChipId =         __sensorprop(0x00, 0xff00, "Chip ID Number")
    regTimingEn =       __sensorprop(0x01, 0x0001, "Allows timing engine to access wavetable")
    regGlobalShutter =  __sensorprop(0x01, 0x0010, "Global shutter mode")
    regFtrEn =          __sensorprop(0x01, 0x0100, "Runs the sensor when in rolling shutter mode")
    regFtRstEn =        __sensorprop(0x01, 0x1000, "Enables rolling reset triggered by ABN")
    regRoiSel =         __sensorprop(0x02, 0x00ff, "Bitmask selecting which ROI to activate")
    regInterRoiSp =     __sensorprop(0x02, 0x0100, "Add a clock of hblank at ROI transitions")
    regDrkColRd =       __sensorprop(0x02, 0x0200, "Enables dark column readout")
    regVflip =          __sensorprop(0x02, 0x0400, "Vertical image flip")
    regHflip =          __sensorprop(0x02, 0x0800, "Horizontal image flip")
    regClkSelFt =       __sensorprop(0x02, 0x3000, "Choose clock frequency for frametimer")
    regHblank =         __sensorprop(0x03, 0,      "Horizontal blanking")
    ## TODO: Special case for regDpScipEn
    regShadowRegs =     __sensorprop(0x05, 0x0001, "Force shadowing registers")
    regXstart =         __sensorprop(0x06, 0x0fff, "Start of standard window (X direction)")
    regXend =           __sensorprop(0x07, 0x0fff, "End of standard window (X direction)")
    regYstart =         __sensorprop(0x08, 0x0fff, "Start of standard window (Y direction)")
    regYend =           __sensorprop(0x09, 0x0fff, "End of standard window (Y direction)")
    ## TODO: Need special arrayview for ROI sections.
    regDrkRowsStAddrTop = __sensorprop(0x2A, 0x0fff, "Address of the first dark row to readout (top of the array)")
    regNbDrkRowsTop =   __sensorprop(0x2B, 0x001f, "Number of dark rows to output (top of the array)")
    regNextRowAddrOvr = __sensorprop(0x2C, 0x0fff, "Specify next row address to be read out")
    regNextRowOvrEn =   __sensorprop(0x2C, 0x1000, "Enable next row to be specified through SCIP")
    regSofDelay =       __sensorprop(0x30, 0xff00, "Delay from TXN rising edge to start of frame")
    regFtTrigNbPulse =  __sensorprop(0x31, 0,      "Number of TXN pulses per frame")
    regFtRstNbPulse =   __sensorprop(0x32, 0,      "Number of ABN pulses per frame")
    regRdoutDly =       __sensorprop(0x34, 0,      "Delay from start of row to start of readout")
    regRowTime =        __sensorprop(0x35, 0,      "Row time (units of pixel clock)")
    regAbnSel =         __sensorprop(0x36, 0x0001, "Select ABN or ABN2 to trigger fast rolling reset")
    regAbn2En =         __sensorprop(0x36, 0x0010, "Enable ABN2 usage")
    regAbn2AltPat =     __sensorprop(0x36, 0x0100, "Alternate ABN1/2 every other row")
    regAbn2Ld =         __sensorprop(0x36, 0x1000, "Load updated exposure control")
    regXorg =           __sensorprop(0x40, 0x0fff, "X origin offset")
    regYorg =           __sensorprop(0x41, 0x0fff, "Y origin offset")
    regPclkLinevalid =  __sensorprop(0x52, 0x0fff, "Pclk channel output when valid pixels")
    regPclkVblank =     __sensorprop(0x53, 0x0fff, "Pclk channel output during vertical blanking")
    regPclkHblank =     __sensorprop(0x54, 0x0fff, "Pclk channel output during horizontal blanking")
    regPclkOpticalBlack = __sensorprop(0x55, 0x0fff, "Pclk channel output when output is dark pixels")
    regMono =           __sensorprop(0x56, 0x0001, "Monochrome/color selection")
    regRowbin2 =        __sensorprop(0x56, 0x0010, "2-row binning")
    regRow2En =         __sensorprop(0x56, 0x0020, "2-row averaging")
    regColbin2 =        __sensorprop(0x56, 0x0100, "2-column binning")
    regColbin4 =        __sensorprop(0x56, 0x0200, "4-column binning")
    regPoutsel =        __sensorprop(0x56, 0x3000, "Pixel output sel (test mode)")
    regInvertAnalog =   __sensorprop(0x56, 0x8000, "Invert polarity of signal chain")
    regGainSelSamp =    __sensorprop(0x57, 0x0fff, "Gain selection sampling cap")
    regGainSelFb =      __sensorprop(0x58, 0x007f, "Gain selection feedback cap")
    regSerialGain =     __sensorprop(0x58, 0x0700, "Serial gain")
    regLvDelay =        __sensorprop(0x5B, 0x003f, "Line valid delay to match ADC latency")
    regCustPat =        __sensorprop(0x5E, 0x0fff, "Custom pattern for ADC training pattern")
    regTstPat =         __sensorprop(0x5E, 0x7000, "Pattern switching")
    regPwrEnSerializerB = __sensorprop(0x5F, 0x000f, "Power enable for LVDS channels 31 to 0")
    regMuxMode =        __sensorprop(0x5F, 0x0030, "Sets number of output channels to use")
    regDacIlv =         __sensorprop(0x60, 0x0007, "Current control for LVDS channels")
    regPclkInv =        __sensorprop(0x60, 0x0010, "Invert Pclk output")
    regDclkInv =        __sensorprop(0x60, 0x0020, "Invert Dclk output")
    regTermbData =      __sensorprop(0x60, 0x0100, "Enable on-chip termination resistor on data and pclk channels")
    regTermbClk =       __sensorprop(0x60, 0x0200, "Enable on-chip termination resistor on dclk channel")
    regSelVlnkeepRst =  __sensorprop(0x67, 0x1f00, "???")
    regSelVdum =        __sensorprop(0x67, 0x6000, "???")
    regSelVdumrst =     __sensorprop(0x69, 0x03e0, "???")
    regSelVlnkeep =     __sensorprop(0x69, 0x7c00, "???")
    regHidyEn =         __sensorprop(0x6D, 0x0001, "Enables high dynamic range operation")
    regGlbFlushEn =     __sensorprop(0x6D, 0x0010, "Enables global flush")
    regHidyTrigNbPulse = __sensorprop(0x6E, 0,     "Number of PRSTN pulses per frame before triggering hidy")
    regSelVdr1Width =   __sensorprop(0x6F, 0,      "???")
    regSelVdr2Width =   __sensorprop(0x70, 0,      "???")
    regSelVdr3Width =   __sensorprop(0x71, 0,      "???")
    regIcolCapEn =      __sensorprop(0x76, 0x00f0, "Selects which CLR/CLS caps to include during readout")
    regSresetB =        __sensorprop(0x7e, 0x0001, "Soft reset: resets all registers")
    regSerSync =        __sensorprop(0x73, 0x0010, "Synchronizes the serializers")
    regSerialGainV2 =   __sensorprop(0x76, 0x00ff, "Serial gain for V2 sensors")

    # Data path registers.
    regDpId =           __dataprop(0x00, 0x0001, "Datapath Identifier")
    regBlockId =        __dataprop(0x00, 0xfffe, "Block Identifier")
    regCrcEn =          __dataprop(0x01, 0x0001, "Insert CRC as first data of hblank")
    regGainEnable =     __dataprop(0x01, 0x0010, "Enable per-channel gain")
    regOddEvenSel =     __dataprop(0x01, 0x0100, "Apply oddeven offset to odd(1) or even rows(0)")
    regOddEvenOsEn =    __dataprop(0x01, 0x1000, "Enables applying odd/even row offsets")
    regNbBitSel =       __dataprop(0x02, 0x0003, "Number of bits per pixel to output")
    regMsbFirstData =   __dataprop(0x02, 0x0010, "Output msb first for data channels")
    regCustDigPat =     __dataprop(0x03, 0x0fff, "Custom pattern for blanking output")
    regDigPatSel =      __dataprop(0x03, 0x3000, "Digital pattern insertion selection")
    ## TODO: Special case for regDpScipEn
    regSelRdoutDly =    __dataprop(0x05, 0x007f, "Delay to match ADC latency")
    regLatchDlyLow =    __dataprop(0x06, 0x0001, "Delay for latch signal in high-speed data deserializers (low channels)")
    regLatchDlyHigh =   __dataprop(0x46, 0x0001, "Delay for latch signal in high-speed data deserializers (high channels)")
    regCalStart =       __dataprop(0x0A, 0x0001, "Start ADC offset calibration from scratch")
    regRecalStart =     __dataprop(0x0A, 0x0010, "Restart ADC offset calibration from current setting")
    regNbBitsSamplesAvg = __dataprop(0x0B, 0x000f, "Number of samples to average for offset cal")
    regNbBitsOsCalIter =  __dataprop(0x0B, 0x00f0, "Max number of offset cal iterations")
    regAdcOsSeqWidth =  __dataprop(0x0C, 0x003f, "Time to spend on applying each of the ADC offsets")
    regOsTarget =       __dataprop(0x0D, 0x0fff, "Dark value target for ADC offset calibration")
    regAdcOsEn =        __dataprop(0x0E, 0x0001, "Enables applying ADC offset registers during vertical blanking")
    regAdcOsSignLow =   __dataprop(0x0F, 0xffff, "Sign for ADC offsets (low channels)")
    regAdcOsSignHigh =  __dataprop(0x4F, 0xffff, "Sign for ADC offsets (high channels)")
    
    # Datapath selection register.
    @property
    def regDpScipEn(self):
        """Register page communication selection"""
        return self.sciRead(0x04, mask=0x0003)
    @regDpScipEn.setter
    def regDpScipEn(self, value):
        return self.sciWrite(0x04, value, mask=0x0003)
 
    class sciBankedArrayView(sensor.sciArrayView):
        """Helper class to create an array view of one bank of the SCI
        register space that can be indexed or iterated upon to read and
        write the image sensor registers.

        Parameters
        ----------
        parent : `sensor`
            Parent sensor object for the SCI register mapping.
        offset : `int`
            Starting address of the first register in the array.
        bank : `int`
            SCI register bank to iterate upon.
        length : `int`
            Number of registers in the array.
        """
        def __init__(self, parent, offset, bank=0, length=0x7F):
            super().__init__(parent, offset, length=length)
            self.bank = bank

        def __getitem__(self, key):
            # Special case for strange register split in DP0/DP1
            if (self.bank and (key >= 16)):
                return self.parent.sciBankRead(self.offset + key + 0x40, self.bank)
            else:
                return self.parent.sciBankRead(self.offset + key + 0x00, self.bank)
                
        def __setitem__(self, key, value):
            # Special case for strange register split in DP0/DP1
            if (self.bank and (key >= 16)):
                return self.parent.sciBankWrite(self.offset + key + 0x40, self.bank, value)
            else:
                return self.parent.sciBankWrite(self.offset + key + 0x00, self.bank, value)

    class adcOffsetArrayView:
        channelMap = [
            0,  8, 16, 24, 4, 12, 20, 28, # Data channels 0 to 7
            2, 10, 18, 26, 6, 14, 22, 30, # Data channels 8 to 15
            1,  9, 17, 25, 5, 13, 21, 29, # Data channels 16 to 23
            3, 11, 19, 27, 7, 15, 23, 31  # Data channels 24 to 31
        ]

        def __init__(self, parent):
            self.parent = parent

        def __len__(self):
            return 32

        """ADC offset arrayview helper class"""
        def __getitem__(self, key):
            self.parent.regDpScipEn = 1 # Switch to datapath.
            channel = self.channelMap[key]
            baseoff = 0x00
            if (channel >= 16):
                baseoff = 0x40
                channel -= 16

            signbits = self.parent.sciRead(0x0f + baseoff)
            offset = self.parent.sciRead(0x10 + baseoff + channel) & 0x3ff
            if (signbits & (1 << channel)):
                return -offset
            else:
                return offset

        def __setitem__(self, key, value):
            self.parent.regDpScipEn = 1 # Switch to datapath.
            channel = self.channelMap[key]
            baseoff = 0x00
            if (channel >= 16):
                baseoff = 0x40
                channel -= 16

            signbits = self.parent.sciRead(0x0f + baseoff)
            if (value < 0):
                self.parent.sciWrite(0x0f + baseoff, signbits | (1 << channel))
                self.parent.sciWrite(0x10 + baseoff + channel, -int(value))
            else:
                self.parent.sciWrite(0x0f + baseoff, signbits & ~(1 << channel))
                self.parent.sciWrite(0x10 + baseoff + channel, int(value))

        # Iterator protocol.
        class __adcIterator:
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
            return self.__adcIterator(self)

    @property
    def regSensor(self):
        """Raw SCI sensor registers"""
        return self.sciBankedArrayView(self, 0x00, 0, 0x7F)
    
    @property
    def regData(self):
        """Raw SCI data registers"""
        return self.sciBankedArrayView(self, 0x00, 1, 0x7F)
    
    @property
    def regAdcOs(self):
        """ADC offsets"""
        return self.adcOffsetArrayView(self)
    
    @property
    def regGainSetval(self):
        """Per-channel digital gain"""
        return self.sciBankedArrayView(self, 0x20, 1, 32)
    
    @property
    def regOddEvenRowOs(self):
        """Per-channel offset for odd or even rows only"""
        return self.adcOffsetArrayView(self, 0x30, 1, 32)

    def wavetable(self, wavetab):
        """Helper function to write a wavetable into the image sensor registers
        using the Serial Communication Interface protocol.

        Parameters
        ----------
        wavetab : `bytes`
            Wavetable to write.
        """
        # Switch to the sensor register bank.
        self.sciWrite(0x04, 0, 0)

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
