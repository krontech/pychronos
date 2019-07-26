#Luxima LUX1310 Image Sensor Class
import pychronos
import time
import math
import copy
import numpy
import logging
import json

from pychronos.error import *
from pychronos.regmaps import sequencer, ioInterface
from pychronos.sensors import api, frameGeometry
from . import lux1310regs, lux1310wt, lux1310timing

class lux1310(api):
    """Driver for the Luxima LUX1310 image sensor.

    The board dictionary may provide the following pins:
        lux1310-spidev: Path to the spidev driver for the voltage DAC.
        lux1310-dac-cs: Path to the chip select for the voltage DAC.
        lux1310-color: Path to the GPIO to read for color detection.

    Parameters
    ----------
    board : `dict`
        Name/value pairs listing the hardware wiring of the board.
    """

    # Image sensor geometry constraints
    MAX_HRES = 1280
    MAX_VRES = 1024
    MIN_HRES = 192
    MIN_VRES = 96
    HRES_INCREMENT = 16
    VRES_INCREMENT = 2
    MAX_VDARK = 8
    BITS_PER_PIXEL = 12

    # Expected constants
    LUX1310_CHIP_ID = 0xDA
    LUX1310_SOF_DELAY = 0x0f
    LUX1310_LV_DELAY = 0x07
    LUX1310_MIN_HBLANK = 2
    LUX1310_SENSOR_HZ = 90000000
    ADC_CHANNELS = 16
    ADC_FOOTROOM = 32
    ADC_OFFSET_MIN = -1023
    ADC_OFFSET_MAX = 1023

    # These are actually more of an FPGA thing...
    COL_GAIN_FRAC_BITS = 12
    COL_CURVE_FRAC_BITS = 21

    # Constants for the DAC configuration
    DAC_VDR3 = 0    # Pixel Driver High Dynamic Range Voltage 3
    DAC_VABL = 1    # Pixel Anti-Blooming Low Voltage
    DAC_VDR1 = 2    # Pixel Driver High Dynamic Range Voltage 1
    DAC_VDR2 = 3    # Pixel Driver High Dynamic Range Voltage 2
    DAC_VRSTB = 4   # Pixel Reset High Voltage 2
    DAC_VRSTH = 5   # Pixel Driver Reset High Voltage
    DAC_VRSTL = 6   # Pixel Driver Reset Low Voltage
    DAC_VRST = 7    # Pixel Reset High Voltage

    # Gain networks for some DAC outputs. 
    dacmap = {      # mul           div
        DAC_VDR3:   ( 1,            1 ),
        DAC_VABL:   ( 100 + 232,    100 ),
        DAC_VDR1:   ( 1,            1 ),
        DAC_VDR2:   ( 1,            1 ),
        DAC_VRSTB:  ( 1,            1 ),
        DAC_VRSTH:  ( 499,          499 + 100 ),
        DAC_VRSTL:  ( 100 + 232,    100 ),
        DAC_VRST:   ( 499,          499 + 100 )
    }

    DAC_FULL_SCALE  = 4095
    DAC_VREF        = 3.3
    DAC_AUTOUPDATE  = 0x9

    @property
    def name(self):
        return "LUX1310"
    
    @property
    def cfaPattern(self):
        try:
            fp = open(self.board["lux1310-color"])
            if int(fp.read()) == 1:
                return 'GRBG'
            else:
                return None
        except:
            return None
    
    @property
    def baseIso(self):
        if self.cfaPattern:
            return 320
        else:
            return 740

    @property
    def hMin(self):
        return self.MIN_HRES
    
    @property
    def hIncrement(self):
        return self.HRES_INCREMENT

    @property
    def vMin(self):
        return self.MIN_VRES
    
    @property
    def vIncrement(self):
        return self.VRES_INCREMENT

    def __init__(self, board={
            "lux1310-spidev":  "/dev/spidev3.0",
            "lux1310-dac-cs": "/sys/class/gpio/gpio33/value",
            "lux1310-color":  "/sys/class/gpio/gpio34/value"} ):
        ## Hardware Resources
        self.spidev = board["lux1310-spidev"]
        self.spics = board["lux1310-dac-cs"]
        self.board = board
        self.regs = lux1310regs.lux1310regs()
        self.wavetables = lux1310wt.wavetables
        self.timing = lux1310timing.lux1310timing()
        self.io     = ioInterface()
        
        ## ADC Calibration state
        self.adcOffsets = [0] * self.ADC_CHANNELS
        self.__nTrigFrames = 5

        ## Save the real resolution for when cal is in progress.
        self.fSizeReal = None

        self.currentProgram = self.timing.PROGRAM_STANDARD
        self.frameClocks = int(0.001 * self.LUX1310_SENSOR_HZ)
        self.exposureClocks = int(self.frameClocks * 0.95)
        
        super().__init__()

    def writeDAC(self, dac, voltage):
        """Write the DAC voltage"""
        # Convert the DAC value
        mul, div = self.dacmap[dac]
        dacval = int((voltage * self.DAC_FULL_SCALE * mul) / (self.DAC_VREF * div))
        if ((dacval < 0)  or (dacval > self.DAC_FULL_SCALE)):
            raise ValueError("DAC Voltage out of range")
        dacval |= (dac << 12)
        
        # Write the DAC value to the SPI.
        pychronos.writespi(device=self.spidev, csel=self.spics, mode=1, bitsPerWord=16,
            data=bytearray([(dacval & 0x00ff) >> 0, (dacval & 0xff00) >> 8]))

    #--------------------------------------------
    # Sensor Configuration and Control API
    #--------------------------------------------
    def reset(self, fSize=None):
        # Enable the timing engine, but disable integration during setup.
        self.timing.enabled = True
        self.timing.programInterm()
        
        # Configure the DAC to autoupdate when written.
        pychronos.writespi(device=self.spidev, csel=self.spics, mode=1, bitsPerWord=16,
            data=bytearray([0, self.DAC_AUTOUPDATE << 4]))

        # Initialize the DAC voltage levels.
        self.writeDAC(self.DAC_VABL, 0.3)
        self.writeDAC(self.DAC_VRSTB, 2.7)
        self.writeDAC(self.DAC_VRST, 3.3)
        self.writeDAC(self.DAC_VRSTL, 0.7)
        self.writeDAC(self.DAC_VRSTH, 3.6)
        self.writeDAC(self.DAC_VDR1, 2.5)
        self.writeDAC(self.DAC_VDR2, 2.0)
        self.writeDAC(self.DAC_VDR3, 1.5)
        time.sleep(0.01) # Settling time

        # Force a reset of the image sensor.
        self.regs.control |= self.regs.RESET
        self.regs.control &= ~self.regs.RESET
        time.sleep(0.001)

        # Reset the SCI interface.
        self.regs.regSresetB = 0
        rev = self.regs.regChipId
        if (rev != self.LUX1310_CHIP_ID):
            logging.error("LUX1310 regChipId returned an invalid ID (%s)", hex(rev))
            return False
        else:
            logging.info("Initializing LUX1310 silicon revision %s", self.regs.revChip)
        
        # Setup ADC training.
        self.regs.regCustPat = 0xFC0    # Set custom pattern for ADC training.
        self.regs.regTstPat = 2         # Enable test pattern
        self.regs.regPclkVblank = 0xFC0 # Set PCLK channel output during vertical blank
        self.regs.reg[0x5A] = 0xE1      # Configure for inverted DCLK output
        self.__autoPhaseCal()

        # Return to normal data mode
        self.regs.regPclkVblank = 0xf00         # PCLK channel output during vertical blanking
        self.regs.regPclkOpticalBlack = 0xfc0   # PCLK channel output during dark pixel readout
        self.regs.regTstPat = False             # Disable test pattern

        # Setup for 80-clock wavetable
        self.regs.regRdoutDly = 80              # Non-overlapping readout delay
        self.regs.regWavetableSize = 80         # Wavetable size

        # Set internal control registers to fine tune the performance of the sensor
        self.regs.regLvDelay = self.LUX1310_LV_DELAY    # Line valid delay to match internal ADC latency
        self.regs.regHblank = self.LUX1310_MIN_HBLANK   # Set horizontal blanking period

        # Undocumented internal registers from Luxima
        self.regs.reg[0x2D] = 0xE08E    # State for idle controls
        self.regs.reg[0x2E] = 0xFC1F    # State for idle controls
        self.regs.reg[0x2F] = 0x0003    # State for idle controls
        self.regs.reg[0x5C] = 0x2202    # ADC clock controls
        self.regs.reg[0x62] = 0x5A76    # ADC range to match pixel saturation level
        self.regs.reg[0x74] = 0x041F    # Internal clock timing
        self.regs.reg[0x66] = 0x0845    # Internal current control
        if (self.regs.revChip == 2):
            self.regs.reg[0x5B] = 0x307F # Internal control register
            self.regs.reg[0x7B] = 0x3007 # Internal control register
        elif (self.regs.revChip == 1):
            self.regs.reg[0x5B] = 0x301F # Internal control register
            self.regs.reg[0x7B] = 0x3001 # Internal control register
        else:
            # Unknown version - use silicon rev1 configuration
            logging.error("Found LUX1310 sensor, unknown silicon revision: %s", self.regs.revChip)
            self.regs.reg[0x5B] = 0x301F # Internal control register
            self.regs.reg[0x7B] = 0x3001 # Internal control register

        for x in range(0, self.ADC_CHANNELS):
            self.regs.regAdcOs[x] = 0
        self.regs.regAdcCalEn = True

        # Configure for nominal gain.
        self.regs.regGainSelSamp = 0x007f
        self.regs.regGainSelFb = 0x007f
        self.regs.regGainBit = 0x03

        # Load the default (longest) wavetable and enable the timing engine.
        self.regs.wavetable(self.wavetables[0].wavetab)
        self.regs.regTimingEn = True
        time.sleep(0.01)

        # Start the FPGA timing engine using the standard timing program.
        self.currentProgram = self.timing.PROGRAM_STANDARD
        self.frameClocks = int(0.001 * self.LUX1310_SENSOR_HZ)
        self.exposureClocks = int(self.frameClocks * 0.95)
        self.timing.programStandard(self.frameClocks, self.exposureClocks)
        return True

    ## TODO: I think this whole function is unnecessary on the LUX1310 FPGA
    ## builds, even in the C++ camApp it doesn't run to completion, and the
    ## return codes are ignored.
    def __autoPhaseCal(self):
        """Private function - calibrate the FPGA data acquisition channels"""
        self.regs.clkPhase = 0
        self.regs.clkPhase = 1
        self.regs.clkPhase = 0
        logging.info("Phase calibration dataCorrect=%s", self.regs.dataCorrect)
    
    #--------------------------------------------
    # Frame Geometry Configuration Functions
    #--------------------------------------------
    def getMaxGeometry(self):
        size = frameGeometry(
            hRes=self.MAX_HRES, vRes=self.MAX_VRES,
            hOffset=0, vOffset=0,
            vDarkRows=self.MAX_VDARK,
            bitDepth=self.BITS_PER_PIXEL)
        size.minFrameTime = self.getMinFrameClocks(size) / self.LUX1310_SENSOR_HZ
        return size
    
    def getCurrentGeometry(self):
        # If calibration is in progress, return the saved geometry instead.
        if self.fSizeReal:
            return copy.deepcopy(self.fSizeReal)
        
        fSize = self.getMaxGeometry()
        fSize.hRes = self.regs.regXend - 0x20 + 1
        fSize.hOffset = self.regs.regXstart - 0x20
        fSize.hRes -= fSize.hOffset
        fSize.vOffset = self.regs.regYstart
        fSize.vRes = self.regs.regYend - fSize.vOffset + 1
        fSize.vDarkRows = self.regs.regNbDrkRows
        fSize.minFrameTime = self.getMinFrameClocks(fSize) / self.LUX1310_SENSOR_HZ
        return fSize

    def isValidResolution(self, size):
            # Enforce resolution limits.
        if ((size.hRes < self.MIN_HRES) or (size.hRes + size.hOffset) > self.MAX_HRES):
            return False
        if ((size.vRes < self.MIN_VRES) or (size.vRes + size.vOffset) > self.MAX_VRES):
            return False
        if (size.vDarkRows > self.MAX_VDARK):
            return False
        if (size.bitDepth != self.BITS_PER_PIXEL):
            return False
        
        # Enforce minimum pixel increments.
        if ((size.hRes % self.HRES_INCREMENT) != 0):
            return False
        if ((size.vRes % self.VRES_INCREMENT) != 0):
            return False
        if ((size.vDarkRows % self.VRES_INCREMENT) != 0):
            return False
        
        # Otherwise, the resultion and offset are valid.
        return True
    
    def updateWavetable(self, size, frameClocks, gaincal=False):
        # Select the longest wavetable that gives a frame period longer than
        # the target frame period. Note that the wavetables are sorted by length
        # in descending order.
        wavetab = None
        for x in self.wavetables:
            wavetab = x
            if (frameClocks >= self.getMinFrameClocks(size, x.clocks)):
                break
        
        logging.debug("Selecting WT%d for %dx%d", wavetab.clocks, size.hRes, size.vRes)

        # If a suitable wavetable exists, then load it.
        if (wavetab):
            self.regs.regTimingEn = False
            self.regs.regRdoutDly = wavetab.clocks
            self.regs.reg[0x7A] = wavetab.clocks
            if (gaincal):
                self.regs.wavetable(wavetab.gaintab)
            else:
                self.regs.wavetable(wavetab.wavetab)
            self.regs.regTimingEn = True
            self.regs.startDelay = wavetab.abnDelay
            self.regs.linePeriod = max((size.hRes // self.HRES_INCREMENT) + 2, wavetab.clocks + 3) - 1

            # set the pulsed pattern timing
            self.timing.setPulsedPattern(wavetab.clocks)
        # Otherwise, the frame period was probably too short for this resolution.
        else:
            raise ValueError("Frame period too short, no suitable wavetable found")

    def updateReadoutWindow(self, size):
        # Configure the image sensor resolution
        hStartBlocks = size.hOffset // self.HRES_INCREMENT
        hWidthBlocks = size.hRes // self.HRES_INCREMENT
        self.regs.regXstart = 0x20 + hStartBlocks * self.HRES_INCREMENT
        self.regs.regXend = 0x20 + (hStartBlocks + hWidthBlocks) * self.HRES_INCREMENT - 1
        self.regs.regYstart = size.vOffset
        self.regs.regYend = size.vOffset + size.vRes - 1
        self.regs.regDrkRowsStAddr = self.MAX_VRES + self.MAX_VDARK - size.vDarkRows + 4
        self.regs.regNbDrkRows = size.vDarkRows

    def setResolution(self, size):
        if (not self.isValidResolution(size)):
            raise ValueError("Invalid frame resolution")
        
        # Select the minimum frame period if not specified.
        minPeriod, maxPeriod = self.getPeriodRange(size)
        if not size.minFrameTime:
            fClocks = self.getMinFrameClocks(size)
        elif ((size.minFrameTime * self.LUX1310_SENSOR_HZ) >= minPeriod):
            fClocks = size.minFrameTime * self.LUX1310_SENSOR_HZ
        else:
            fClocks = self.getMinFrameClocks(size)

        # Disable the FPGA timing engine and wait for the current readout to end.
        self.timing.programInterm()
        time.sleep(0.01) # Extra delay to allow frame readout to finish. 

        # Switch to the desired resolution pick the best matching wavetable.
        self.updateReadoutWindow(size)
        self.updateWavetable(size, frameClocks=fClocks)
        time.sleep(0.01)

        # set the minimum frame period in the timing engine (set using wavetable periods or lines)
        self.timing.minLines = size.vRes

        # Set the frame period and the maximum shutter after changing resolution.
        self.frameClocks = fClocks
        self.exposureClocks = int(self.frameClocks * 0.95)
        self.currentProgram = self.timing.PROGRAM_STANDARD
        self.timing.programStandard(self.frameClocks, self.exposureClocks)
    
    #--------------------------------------------
    # Frame Timing Configuration Functions
    #--------------------------------------------
    def getMinFrameClocks(self, size, wtSize=0):
        # Select the longest wavetable that fits within the line readout time,
        # or fall back to the shortest wavetable for extremely small resolutions.
        if (wtSize == 0):
            ideal = (size.hRes // self.HRES_INCREMENT) + self.LUX1310_MIN_HBLANK - 3
            for x in self.wavetables:
                wtSize = x.clocks
                if (wtSize <= ideal):
                    break

        # Compute the minimum number of 90MHz LUX1310 sensor clocks per frame.
        # Refer to section 7.1 of the LUX1310 datasheet version v3.0
        tRead = size.hRes // self.HRES_INCREMENT
        tTx = 25        # hard-coded to 25 clocks in the FPGA, should be at least 350ns
        tRow = max(tRead + self.LUX1310_MIN_HBLANK, wtSize + 3)
        tFovf = self.LUX1310_SOF_DELAY + wtSize + self.LUX1310_LV_DELAY + 10
        tFovb = 41      # Duration between PRSTN falling and TXN falling (I think)
        tFrame = tRow * (size.vRes + size.vDarkRows) + tTx + tFovf + tFovb - self.LUX1310_MIN_HBLANK

        return tFrame
    
    def getPeriodRange(self, fSize):
        # If a frame time was provided, find the longest matching wavetable.
        if (fSize.minFrameTime):
            fClocks = fSize.minFrameTime * self.LUX1310_SENSOR_HZ
            for x in self.wavetables:
                wtFrameClocks = self.getMinFrameClocks(fSize, x.clocks)
                if (wtFrameClocks <= fClocks):
                    return (wtFrameClocks / self.LUX1310_SENSOR_HZ, 0)
            # This frame time could not be met.
            raise ValueError("Invalid frame time for resolution given")
        else:
            fClocks = self.getMinFrameClocks(fSize)
            return (fClocks / self.LUX1310_SENSOR_HZ, 0)
    
    def getCurrentPeriod(self):
        return self.frameClocks / self.LUX1310_SENSOR_HZ

    def setFramePeriod(self, fPeriod):
        # TODO: Sanity-check the frame period.
        self.currentProgram = self.timing.PROGRAM_STANDARD
        self.frameClocks = math.ceil(fPeriod * self.LUX1310_SENSOR_HZ)
        self.timing.programStandard(self.frameClocks, self.exposureClocks)
    
    def getExposureRange(self, fSize, fPeriod):
        # Defaulting to 1us minimum exposure and infinite maximum exposure.
        # TODO: Need a better handle on the exposure overhead.
        return (1.0 / 1000000, fPeriod - (500 / self.LUX1310_SENSOR_HZ))

    def getCurrentExposure(self):
        return self.exposureClocks / self.LUX1310_SENSOR_HZ
    
    def setExposureProgram(self, expPeriod):
        # TODO: Sanity-check the exposure time.
        if (expPeriod < 1.0 / 1000000):
            expPeriod = 1.0 / 1000000
        
        # Disable HDR modes.
        self.regs.regHidyEn = False

        self.currentProgram = self.timing.PROGRAM_STANDARD
        self.exposureClocks = math.ceil(expPeriod * self.LUX1310_SENSOR_HZ)
        self.timing.programStandard(self.frameClocks, self.exposureClocks)

    #--------------------------------------------
    # Advanced Exposure and Timing Functions 
    #--------------------------------------------
    def getSupportedExposurePrograms(self):
        return ("normal", "frameTrigger", "shutterGating", "hdr2slope", "hdr3slope")
    
    def setShutterGatingProgram(self):
        # Disable HDR modes.
        self.regs.regHidyEn = False

        self.currentProgram = self.timing.PROGRAM_SHUTTER_GATING
        self.timing.programShutterGating()
    
    def setFrameTriggerProgram(self, expPeriod):
        if (expPeriod < 1.0 / 1000000):
            expPeriod = 1.0 / 1000000
        
        # Disable HDR modes.
        self.regs.regHidyEn = False

        self.currentProgram = self.timing.PROGRAM_FRAME_TRIG
        self.exposureClocks = math.ceil(expPeriod * self.LUX1310_SENSOR_HZ)
        self.timing.programTriggerFrames(self.frameClocks, self.exposureClocks)

    def setHdrExposureProgram(self, expPeriod, numIntegrations=2):
        if numIntegrations == 2:
            # Reprogram the HDR DACs.
            self.writeDAC(self.DAC_VDR1, 2.7)
            self.writeDAC(self.DAC_VDR2, 2.7)
            self.writeDAC(self.DAC_VDR3, 2.7)

            self.exposureClocks = math.ceil(expPeriod * self.LUX1310_SENSOR_HZ)
            expFirst = self.exposureClocks * 0.9
            expSecond = self.exposureClocks * 0.09

            # Configure the VDR pulse timing widths.
            self.regs.regFtTrigNbPulse = 2
            self.regs.regSelVdr1Width  = int(expSecond) + 17 + 50
            self.regs.regSelVdr2Width  = 0
            self.regs.regSelVdr3Width  = 0
            self.regs.regHidyEn        = True

            self.currentProgram = self.timing.PROGRAM_2POINT_HDR
            self.timing.programHDR_2slope(self.frameClocks, expFirst, expSecond)
        elif numIntegrations == 3:
            # Reprogram the HDR DACs.
            self.writeDAC(self.DAC_VDR1, 2.5)
            self.writeDAC(self.DAC_VDR2, 2.0)
            self.writeDAC(self.DAC_VDR3, 2.0)

            self.exposureClocks = math.ceil(expPeriod * self.LUX1310_SENSOR_HZ)
            expFirst = self.exposureClocks * 0.9
            expSecond = self.exposureClocks * 0.09
            expThird = self.exposureClocks * 0.009

            self.regs.regFtTrigNbPulse = 3
            self.regs.regSelVdr1Width  = (15 + 45 + 60)
            self.regs.regSelVdr2Width  = int(expSecond + expThird)
            self.regs.regSelVdr3Width  = 0
            self.regs.regHidyEn        = True

            self.currentProgram = self.timing.PROGRAM_3POINT_HDR
            self.timing.programHDR_3slope(self.frameClocks, expFirst, expSecond, expThird)
        else:
            raise NotImplementedError("HDR timing with %s-slope integrations are not implemented." % (numIntegrations))

    #--------------------------------------------
    # Sensor Analog Calibration Functions
    #--------------------------------------------
    def getColorMatrix(self, cTempK=5500):
        if not self.cfaPattern:
            # Identity matrix for monochrome cameras.
            return [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
        else:
            # CIECAM16/D55
            return [ 1.9147, -0.5768, -0.2342, 
                    -0.3056,  1.3895, -0.0969,
                     0.1272, -0.9531,  1.6492]
    
    def getWhiteBalance(self, cTempK=5500):
        if not self.cfaPattern:
            return [1.0, 1.0, 1.0]
        else:
            return [1.5226, 1.0723, 1.5655]

    @property
    def maxGain(self):
        return 16
    
    def setGain(self, gain):
        gainConfig = {  # VRSTB, VRST,  VRSTH,  Sampling Cap, Feedback Cap, Serial Gain
            1:          ( 2.7,   3.3,   3.6,    0x007f,       0x007f,       0x3),
            2:          ( 2.7,   3.3,   3.6,    0x0fff,       0x007f,       0x3),
            4:          ( 2.7,   3.3,   3.6,    0x0fff,       0x007f,       0x0),
            8:          ( 1.7,   2.3,   2.6,    0x0fff,       0x0007,       0x0),
            16:         ( 1.7,   2.3,   2.6,    0x0fff,       0x0001,       0x0),
        }
        if (not int(gain) in gainConfig):
            raise ValueError("Unsupported image gain setting")
        
        vrstb, vrst, vrsth, samp, feedback, sgain = gainConfig[int(gain)]
        self.writeDAC(self.DAC_VRSTB, vrstb)
        self.writeDAC(self.DAC_VRST, vrst)
        self.writeDAC(self.DAC_VRSTH, vrsth)
        self.regs.regGainSelSamp = samp
        self.regs.regGainSelFb = feedback
        self.regs.regGainBit = sgain
    
    def getCurrentGain(self):
        gsMap = {
            0: 2.0,
            1: 1.456,
            2: 1.0,
            3: 0.763
        }
        sampnbits = 4
        fbacknbits = 1
        gsernbits = 0

        x = self.regs.regGainSelSamp
        while (x != 0):
            sampnbits += (x & 1)
            x >>= 1
        
        x = self.regs.regGainSelFb
        while (x != 0):
            fbacknbits += (x & 1)
            x >>= 1

        x = self.regs.regGainBit
        while (x != 0):
            gsernbits += (x & 1)
            x >>= 1
        
        return (sampnbits / fbacknbits) * gsMap[gsernbits]
    
    def autoAdcOffsetIteration(self, fSize, numFrames=4):
        # Read out the calibration frames.
        fAverage = numpy.zeros((fSize.vDarkRows * fSize.hRes // self.ADC_CHANNELS, self.ADC_CHANNELS), dtype=numpy.uint32)
        seq = sequencer()
        for x in range(0, numFrames):
            yield from seq.startLiveReadout(fSize.hRes, fSize.vDarkRows)
            if not seq.liveResult:
                logging.error("ADC offset training failed to read frame")
                return
            fAverage += numpy.reshape(seq.liveResult, (-1, self.ADC_CHANNELS))
        
        # Train the ADC offsets for a target of Average = Footroom + StandardDeviation
        fAverage //= numFrames
        adcAverage = numpy.average(fAverage, 0)
        adcStdDev = numpy.std(fAverage, 0)
        for col in range(0, self.ADC_CHANNELS):
            self.adcOffsets[col] -= (adcAverage[col] - adcStdDev[col] - self.ADC_FOOTROOM) / 2
            if (self.adcOffsets[col] < self.ADC_OFFSET_MIN):
                self.adcOffsets[col] = self.ADC_OFFSET_MIN
            elif (self.adcOffsets[col] > self.ADC_OFFSET_MAX):
                self.adcOffsets[col] = self.ADC_OFFSET_MAX
            
            # Update the image sensor
            self.regs.regAdcOs[col] = self.adcOffsets[col]

    def autoAdcOffsetCal(self, fSize, iterations=16):
        tRefresh = (self.frameClocks * 3) / self.LUX1310_SENSOR_HZ
        tRefresh += (1/60)

        # Clear out the ADC offsets
        for i in range(0, self.ADC_CHANNELS):
            self.adcOffsets[i] = 0
            self.regs.regAdcOs[i] = 0
        
        # Enable ADC calibration and iterate on the offsets.
        self.regs.regAdcCalEn = True
        for i in range(0, iterations):
            yield tRefresh
            yield from self.autoAdcOffsetIteration(fSize)

    def autoAdcGainCal(self, fSize):
        # Setup some math constants
        numRows = 64
        tRefresh = (self.frameClocks * 10) / self.LUX1310_SENSOR_HZ
        pixFullScale = (1 << fSize.bitDepth)

        seq = sequencer()
        colGainRegs = pychronos.fpgamap(pychronos.FPGA_COL_GAIN_BASE, 0x1000)
        colCurveRegs = pychronos.fpgamap(pychronos.FPGA_COL_CURVE_BASE, 0x1000)

        # Disable the FPGA timing engine and load the gain calibration wavetable. 
        self.timing.programInterm()
        time.sleep(0.1) # Extra delay to allow frame readout to finish. 
        self.updateWavetable(fSize, frameClocks=self.frameClocks, gaincal=True)
        self.timing.programStandard(self.frameClocks, self.exposureClocks)

        # Search for a dummy voltage high reference point.
        vhigh = 31
        while (vhigh > 0):
            self.regs.regSelVdumrst = vhigh
            yield tRefresh

            # Read a frame and compute the column averages and minimum.
            yield from seq.startLiveReadout(fSize.hRes, numRows)
            if not seq.liveResult:
                raise CalibrationError("Failed to acquire frames during calibration")
            frame = numpy.reshape(seq.liveResult, (-1, self.ADC_CHANNELS))
            highColumns = numpy.average(frame, 0)

            # High voltage should be less than 7/8ths of full scale.
            if (numpy.amax(highColumns) <= (pixFullScale - (pixFullScale / 8))):
                break
            else:
                vhigh -= 1
        
        # Search for a dummy voltage low reference point.
        vlow = 0
        while (vlow < vhigh):
            self.regs.regSelVdumrst = vlow
            yield tRefresh
            
            # Read a frame and compute the column averages and minimum.
            yield from seq.startLiveReadout(fSize.hRes, numRows)
            if not seq.liveResult:
                raise CalibrationError("Failed to acquire frames during calibration")
            frame = numpy.reshape(seq.liveResult, (-1, self.ADC_CHANNELS))
            lowColumns = numpy.average(frame, 0)

            # Find the minum voltage that does not clip.
            if (numpy.amin(lowColumns) >= self.ADC_FOOTROOM):
                break
            else:
                vlow += 1

        # Sample the midpoint, which should be somewhere around quarter scale.
        vmid = (vhigh + 3*vlow) // 4
        self.regs.regSelVdumrst = vmid
        yield tRefresh

        # Read a frame out of the live display.
        yield from seq.startLiveReadout(fSize.hRes, numRows)
        if not seq.liveResult:
            raise CalibrationError("Failed to acquire frames during calibration")
        frame = numpy.reshape(seq.liveResult, (-1, self.ADC_CHANNELS))
        midColumns = numpy.average(frame, 0)

        logging.debug("ADC Gain calibration voltages=[%s, %s, %s]" % (vlow, vmid, vhigh))
        logging.debug("ADC Gain calibration averages=[%s, %s, %s]" %
                (numpy.average(lowColumns), numpy.average(midColumns), numpy.average(highColumns)))

        # Determine which column has the strongest response and sanity-check the gain
        # measurements. If things are out of range, then give up on gain calibration
        # and apply a gain of 1.0 instead.
        maxColumn = 0
        for col in range(0, self.ADC_CHANNELS):
            minrange = (pixFullScale // 16)
            diff = highColumns[col] - lowColumns[col]
            if ((highColumns[col] <= (midColumns[col] + minrange)) or (midColumns[col] <= (lowColumns[col] + minrange))):
                for x in range(0, self.MAX_HRES):
                    colGainRegs.mem16[x] = (1 << self.COL_GAIN_FRAC_BITS)
                    colCurveRegs.mem16[x] = 0
                raise CalibrationError("ADC Auto calibration range error")
            if (diff > maxColumn):
                maxColumn = diff

        # Compute the 2-point calibration coefficient.
        diff = (highColumns - lowColumns)
        gain2pt = numpy.full(self.ADC_CHANNELS, maxColumn) / diff

        # Predict the ADC to be linear with dummy voltage and find the error.
        predict = lowColumns + (diff * (vmid - vlow) / (vhigh - vlow))
        err2pt = midColumns - predict

        logging.debug("ADC Columns 2-point gain: %s" % (gain2pt))
        logging.debug("ADC Columns 2-point error: %s" % (err2pt))

        # Add a parabola to compensate for the curvature. This parabola should have
        # zeros at the high and low measurement points, and a curvature to compensate
        # for the error at the middle range. Such a parabola is therefore defined by:
        #
        #  f(X) = a*(X - Xlow)*(X - Xhigh), and
        #  f(Xmid) = -error
        #
        # Solving for the curvature gives:
        #
        #  a = error / ((Xmid - Xlow) * (Xhigh - Xmid))
        #
        # The resulting 3-point calibration function is therefore:
        #
        #  Y = a*X^2 + (b - a*Xhigh - a*Xlow)*X + c
        #  a = three-point curvature correction.
        #  b = two-point gain correction.
        #  c = some constant (black level).
        self.curve3pt = err2pt / ((midColumns - lowColumns) * (highColumns - midColumns))
        self.gain3pt = gain2pt - self.curve3pt * (highColumns + lowColumns)

        logging.debug("ADC Columns 3-point gain: %s" % (self.gain3pt))
        logging.debug("ADC Columns 3-point curve: %s" % (self.curve3pt))

        # Load and enable the 3-point calibration.
        colGainRegs = pychronos.fpgamap(pychronos.FPGA_COL_GAIN_BASE, 0x1000)
        colCurveRegs = pychronos.fpgamap(pychronos.FPGA_COL_CURVE_BASE, 0x1000)
        for col in range(self.MAX_HRES):
            curvature = int(self.curve3pt[col % self.ADC_CHANNELS] * (1 << self.COL_CURVE_FRAC_BITS))
            colGainRegs.mem16[col] = int(self.gain3pt[col % self.ADC_CHANNELS] * (1 << self.COL_GAIN_FRAC_BITS))
            if (curvature > 0):
                colCurveRegs.mem16[col] = curvature
            else:
                colCurveRegs.mem16[col] = (0x10000 + curvature) & 0xffff
        display = pychronos.regmaps.display()
        display.gainControl |= display.GAINCTL_3POINT

    def startAnalogCal(self, saveLocation=None):     
        # Retrieve the current resolution and frame period.
        self.fSizeReal = self.getCurrentGeometry()
        fSizeCal = copy.deepcopy(self.fSizeReal)
        fPeriod = self.frameClocks / self.LUX1310_SENSOR_HZ

        # Enable black bars if not already done.
        if (fSizeCal.vDarkRows == 0):
            logging.debug("Enabling dark pixel readout")
            fSizeCal.vDarkRows = self.MAX_VDARK // 2
            fSizeCal.vOffset += fSizeCal.vDarkRows
            fSizeCal.vRes -= fSizeCal.vDarkRows

            # Disable the FPGA timing engine and apply the changes.
            self.timing.programInterm()
            time.sleep(0.01) # Extra delay to allow frame readout to finish. 
            self.updateReadoutWindow(fSizeCal)
            self.regs.regHidyEn = False
            self.timing.programStandard(self.frameClocks, self.exposureClocks)
        
        # Perform ADC offset calibration using the optical black regions.
        logging.debug('Starting ADC offset calibration')
        yield from self.autoAdcOffsetCal(fSizeCal)

        # Perform ADC column gain calibration using the dummy voltage.
        logging.debug('Starting ADC gain calibration')
        yield from self.autoAdcGainCal(fSizeCal)

        # Save the analog calibration results if a location was provided.
        if saveLocation:
            caldata = {
                "offsets": self.adcOffsets,
                "gain": list(self.gain3pt),
                "curve": list(self.curve3pt)
            }
            with open(self.calFilename(saveLocation + '/lux1310cal', '.json'), 'w') as fp:
                json.dump(caldata, fp)

        # Restore the frame period and wavetable.
        logging.debug("Restoring sensor configuration")
        self.timing.programInterm()
        time.sleep(0.01) # Extra delay to allow frame readout to finish. 
        self.updateReadoutWindow(self.fSizeReal)
        self.updateWavetable(self.fSizeReal, frameClocks=self.frameClocks, gaincal=False)
        self.fSizeReal = None

        # Restore the timing program
        if (self.currentProgram == self.timing.PROGRAM_STANDARD):
            self.timing.programStandard(self.frameClocks, self.exposureClocks)
        elif (self.currentProgram == self.timing.PROGRAM_SHUTTER_GATING):
            self.timing.programShutterGating()
        elif (self.currentProgram == self.timing.PROGRAM_FRAME_TRIG):
            self.timing.programTriggerFrames(self.frameClocks, self.exposureClocks)
        elif (self.currentProgram == self.timing.PROGRAM_2POINT_HDR):
            self.setHdrExposureProgram(self.exposureClocks / self.LUX1310_SENSOR_HZ, 2)
        elif (self.currentProgram == self.timing.PROGRAM_3POINT_HDR):
            self.setHdrExposureProgram(self.exposureClocks / self.LUX1310_SENSOR_HZ, 3)
        else:
            logging.error("Invalid timing program, reverting to standard exposure")
            self.timing.programStandard(self.frameClocks, self.exposureClocks)

    def loadAnalogCal(self, calLocation):
        loadSuccessful = False
        loadFilename = self.calFilename(calLocation + '/lux1310cal', '.json')
        try:
            with open(loadFilename, 'r') as fp:
                # Read calibration from the file.
                logging.info("Loading lux1310 calibration data from %s", loadFilename)
                calData = json.load(fp)
                self.adcOffsets = list(calData['offsets'])
                self.gain3pt = numpy.array(calData['gain'], dtype=numpy.float)
                self.curve3pt = numpy.array(calData['curve'], dtype=numpy.float)
                if (len(self.adcOffsets) < self.ADC_CHANNELS):
                    raise ValueError("Invalid array size for ADC offset calibration data")
                if (len(self.gain3pt) < self.ADC_CHANNELS):
                    raise ValueError("Invalid array size for ADC gain calibration data")
                if (len(self.curve3pt) < self.ADC_CHANNELS):
                    raise ValueError("Invalid array size for ADC gain calibration data")
                loadSuccessful = True
        except Exception as err:
            # Load some sensible defaults.
            logging.error("Failed to load lux1310 calibration data from %s", loadFilename)
            self.adcOffsets = [0] * self.ADC_CHANNELS
            self.gain3pt = numpy.full(self.ADC_CHANNELS, 1.0)
            self.curve3pt = numpy.full(self.ADC_CHANNELS, 0.0)

        # Program the ADC offsets.
        for col in range(self.ADC_CHANNELS):
            self.regs.regAdcOs[col] = self.adcOffsets[col]

        # Load and enable the 3-point calibration.
        colGainRegs = pychronos.fpgamap(pychronos.FPGA_COL_GAIN_BASE, 0x1000)
        colCurveRegs = pychronos.fpgamap(pychronos.FPGA_COL_CURVE_BASE, 0x1000)
        for col in range(self.MAX_HRES):
            curvature = int(self.curve3pt[col % self.ADC_CHANNELS] * (1 << self.COL_CURVE_FRAC_BITS))
            colGainRegs.mem16[col] = int(self.gain3pt[col % self.ADC_CHANNELS] * (1 << self.COL_GAIN_FRAC_BITS))
            if (curvature > 0):
                colCurveRegs.mem16[col] = curvature
            else:
                colCurveRegs.mem16[col] = (0x10000 + curvature) & 0xffff
        display = pychronos.regmaps.display()
        display.gainControl |= display.GAINCTL_3POINT

        return loadSuccessful

    def calFilename(self, prefix, extension=""):
        wtClocks = self.regs.regRdoutDly
        gain = int(self.getCurrentGain())
        return "%s_G%d_WT%d%s" % (prefix, gain, wtClocks, extension)
