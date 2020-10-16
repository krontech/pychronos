#Luxima LUX2100 Image Sensor Class
import pychronos
import time
import os
import math
import copy
import numpy
import logging
import json
import shutil

from pychronos.error import *
from pychronos.regmaps import sequencer, ioInterface
from pychronos.sensors import api, frameGeometry
from . import lux2100regs, lux2100wt, lux2100timing

from struct import pack ## added for exporting gain data

BB_ENABLED_NF = 0

class lux2100(api):
    """Driver for the Luxima LUX2100 image sensor.

    The board dictionary may provide the following pins:
        lux2100-spidev: Path to the spidev driver for the voltage DAC.
        lux2100-dac-cs: Path to the chip select for the voltage DAC.
        lux2100-color: Path to the GPIO to read for color detection.

    Parameters
    ----------
    board : `dict`
        Name/value pairs listing the hardware wiring of the board.
    """

    # Image sensor geometry constraints
    MAX_HRES = 1920
    MAX_VRES = 1080
    MIN_HRES = 640
    MIN_VRES = 96
    HRES_INCREMENT = 32
    VRES_INCREMENT = 2
    MAX_VDARK = 8
    BITS_PER_PIXEL = 12

    VLOW_BOUNDARY = 8
    VHIGH_BOUNDARY = 8
    HLEFT_DARK = 32 ## default: 32
    HRIGHT_DARK = 32

    # Expected constants
    LUX2100_CHIP_ID = 0x28
    LUX2100_SOF_DELAY = 10
    LUX2100_LV_DELAY = 8
    LUX2100_MIN_HBLANK = 2
    LUX2100_SENSOR_HZ = 75000000
    ADC_CHANNELS = 32
    ADC_FOOTROOM = 64
    ADC_OFFSET_MIN = -1023
    ADC_OFFSET_MAX = 1023

    # These are actually more of an FPGA thing...
    COL_GAIN_FRAC_BITS = 12
    COL_CURVE_FRAC_BITS = 21

    # Constants for the DAC configuration
    DAC_VABL = 0        # Pixel Anti-Blooming Low Voltage
    DAC_VTX2L = 1       # Row Driver Low Voltage
    DAC_VTXH = 2        # Row Driver High Voltage
    DAC_VDR1 = 3        # Pixel Driver High Dynamic Range Voltage 1
    DAC_VRSTH = 4       # Pixel Reset High Voltage
    DAC_VDR3 = 6        # Pixel Driver High Dynamic Range Voltage 3
    DAC_VRDH = 7        # Row Driver Spare Voltage
    DAC_VTX2H = 9       # Row Driver High Voltage
    DAC_VTXL = 11       # Row Driver Low Voltage
    DAC_VPIX_OP = 12    # Pixel supply op-amp voltage
    DAC_VDR2 = 13       # Pixel Driver High Dynamic Range Voltage 2
    DAC_VPIX_LDO = 14   # Pixel supply LDO voltage
    DAC_VRSTPIX = 15    # Pixel reset voltage

    # Gain networks for some DAC outputs. 
    dacmap = {          # mul       div         offset
        DAC_VABL:       ( 1,        1,          None),
        DAC_VTX2L:      ( 200,      100,        None),
        DAC_VTXH:       ( 499,      499 + 100,  None),
        DAC_VDR1:       ( 1,        1,          None),
        DAC_VRSTH:      ( 499,      499 + 100,  None),
        DAC_VDR3:       ( 1,        1,          None),
        DAC_VRDH:       ( 1,        1,          None),
        DAC_VTX2H:      ( 499,      499 + 100,  None),
        DAC_VTXL:       ( 316,      100,        0.5485),
        DAC_VPIX_OP:    ( 499,      499 + 100,  None),
        DAC_VDR2:       ( 1,        1,          None),
        DAC_VPIX_LDO:   ( 422,      165,        3.68),
        DAC_VRSTPIX:    ( 1,        1,          None),
    }

    DAC_FULL_SCALE  = 4095
    DAC_VREF        = 3.3
    DAC_AUTOUPDATE  = 0x9

    @property
    def name(self):
        return "LUX2100"
    
    @property
    def cfaPattern(self):
        try:
            fp = open(self.board["lux2100-color"])
            if int(fp.read()) == 1:
                return 'GRBG'
            else:
                return None
        except:
            return None
    
    @property
    def baseIso(self):
        if self.cfaPattern:
            return 580
        else:
            return 1160

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
            "lux2100-spidev":  "/dev/spidev3.0",
            "lux2100-dac-cs": "/sys/class/gpio/gpio33/value",
            "lux2100-color":  "/sys/class/gpio/gpio34/value"} ):
        ## Hardware Resources
        self.spidev = board["lux2100-spidev"]
        self.spics = board["lux2100-dac-cs"]
        self.board = board
        self.regs = lux2100regs.lux2100regs()
        self.wavetables = lux2100wt.wavetables
        self.timing = lux2100timing.lux2100timing()
        self.io     = ioInterface()
        
        ## ADC Calibration state
        self.adcOffsets = [0] * self.ADC_CHANNELS

        ## Save the real resolution for when cal is in progress.
        self.fSizeReal = None

        self.currentProgram = self.timing.PROGRAM_STANDARD
        self.frameClocks = int(0.001 * self.LUX2100_SENSOR_HZ)
        self.exposureClocks = int(self.frameClocks * 0.95)

        super().__init__()

    def writeDAC(self, dac, voltage):
        """Write the DAC voltage"""
        # Convert the DAC value
        dacdata=bytearray([0, self.DAC_AUTOUPDATE << 4, 0, self.DAC_AUTOUPDATE << 4])
        mul, div, offs = self.dacmap[dac]
        if (offs):
            voltage = offs - voltage
        
        dacval = int((voltage * self.DAC_FULL_SCALE * mul) / (self.DAC_VREF * div))
        if ((dacval < 0)  or (dacval > self.DAC_FULL_SCALE)):
            raise ValueError("DAC Voltage out of range")
        
        if (dac < 8):
            dacval |= (dac << 12)
            dacdata[2] = (dacval & 0x00ff) >> 0
            dacdata[3] = (dacval & 0xff00) >> 8
        else:
            dacval |= ((dac-8) << 12)
            dacdata[0] = (dacval & 0x00ff) >> 0
            dacdata[1] = (dacval & 0xff00) >> 8
        
        # Write the DAC value to the SPI.
        pychronos.writespi(device=self.spidev, csel=self.spics, mode=1, bitsPerWord=16, data=dacdata)

    #--------------------------------------------
    # Sensor Configuration and Control API
    #--------------------------------------------
    def __autoPhaseCal(self):
        self.regs.clkPhase = 0
        self.regs.clkPhase = 1
        self.regs.clkPhase = 0
        logging.info("Phase calibration dataCorrect=%s", self.regs.dataCorrect)
    
    def reset(self, fSize=None):
        # Enable the timing engine, but disable integration during setup.
        self.timing.programInterm()
        self.timing.reset()
        
        # Configure the DAC to autoupdate when written.
        pychronos.writespi(device=self.spidev, csel=self.spics, mode=1, bitsPerWord=16,
            data=bytearray([0, self.DAC_AUTOUPDATE << 4, 0, self.DAC_AUTOUPDATE << 4]))

        # Initialize the DAC voltage levels.
        self.writeDAC(self.DAC_VRSTH,   3.3) ## 2.5 - 3.6 ## default: 3.3
        self.writeDAC(self.DAC_VTX2L,   0.0) ## 0.0 - 1.0 ## default: 0.0
        self.writeDAC(self.DAC_VRSTPIX, 2.5) ## 0.3 - 2.5 ## default: 2.0
        self.writeDAC(self.DAC_VABL,    0.0) ## 0.3 - 2.5 ## default: 0.0
        self.writeDAC(self.DAC_VTXH,    3.3) ## 3.0 - 3.6 ## default: 3.3
        self.writeDAC(self.DAC_VTX2H,   3.3) ## 3.0 - 3.6 ## default: 3.3
        self.writeDAC(self.DAC_VTXL,    0.0) ## -0.5 - 0.0 ## default: 0.0

        self.writeDAC(self.DAC_VDR1,    2.5) ## 0.5 - 2.5 ## default: 2.5
        self.writeDAC(self.DAC_VDR2,    2.0) ## 0.5 - 2.5 ## default: 2.0
        self.writeDAC(self.DAC_VDR3,    1.5) ## 0.5 - 2.5 ## default: 1.5
        self.writeDAC(self.DAC_VRDH,    0.0) ## 0.0 ## default: 0.0

        self.writeDAC(self.DAC_VPIX_LDO, 3.3) ## ?? default 3.3
        self.writeDAC(self.DAC_VPIX_OP, 0.0) ## ?? default 0.0
        time.sleep(0.01) # Settling time

        # Force a reset of the image sensor.
        self.regs.control |= self.regs.RESET
        self.regs.control &= ~self.regs.RESET
        time.sleep(0.001)

        # Reset the SCI interface.
        self.regs.regSresetB = 0
        rev = self.regs.regChipId
        if (rev != self.LUX2100_CHIP_ID):
            logging.error("LUX2100 regChipId returned an invalid ID (%s)", hex(rev))
            return False
        else:
            logging.info("Initializing LUX2100 silicon revision %s", self.regs.revChip)

        # Set an extra serial gain register if rev is 2
        if (self.regs.revChip == 2):
#            self.regs.regSerialGainV2 = 0x0079
            self.regs.regSerialGainV2 = 0x0019

        # Setup ADC training.
        self.regs.regPclkVblank = 0xFC0 # Set blanking pattern for ADC training.
        self.regs.regCustDigPat = 0xFC0 # Set custom data pattern for ADC training.
        self.regs.regDigPatSel = 1      # Enable custom data pattern.
        self.__autoPhaseCal()

        # Return to normal data mode
        self.regs.regPclkVblank = 0xf00         # PCLK channel output during vertical blanking
        self.regs.regPclkOpticalBlack = 0xFE0   # PCLK channel output during dark pixel readout << Nicholas
        self.regs.regDigPatSel = 0              # Disable custom data pattern for pixel readout.
        self.regs.regSelRdoutDly = 7            # Delay to match ADC latency


#        self.regs.regDrkColRd = True ## enable the dark columns (dark side bands) << Nicholas (NF) << doesn't work (don't seem to need)


        # Setup for 66-clock wavetable and 1080p with binning.
        self.regs.regRdoutDly = 66
        self.regs.regMono = True
        self.regs.regRow2En = True
        self.regs.regColbin2 = True
        self.regs.regPoutsel = 2
        self.regs.regInvertAnalog = True
        self.regs.regXstart = self.HLEFT_DARK * 2
        self.regs.regXend = (self.HLEFT_DARK + self.MAX_HRES - BB_ENABLED_NF) * 2 - 1 ## Nicholas
        self.regs.regYstart = self.VLOW_BOUNDARY * 2
        self.regs.regYend = (self.VLOW_BOUNDARY + self.MAX_VRES - 1) * 2

        # Set internal control registers to fine tune the performance of the sensor
        self.regs.regHblank = self.LUX2100_MIN_HBLANK   # Set horizontal blanking period
        self.regs.regLvDelay = self.LUX2100_LV_DELAY    # Line valid delay to match internal ADC latency
        self.regs.regSofDelay = self.LUX2100_SOF_DELAY

        # Undocumented internal registers from Luxima
        self.regs.regSensor[0x5F] = 0x0000 # internal control register
        self.regs.regSensor[0x78] = 0x0803 # internal clock timing
        self.regs.regSensor[0x2D] = 0x008C # state for idle controls
        self.regs.regSensor[0x2E] = 0x0000 # state for idle controls
        self.regs.regSensor[0x2F] = 0x0040 # state for idle controls
        self.regs.regSensor[0x62] = 0x2603 # ADC clock controls
        self.regs.regSensor[0x60] = 0x0300 # enable on chip termination
        self.regs.regSensor[0x79] = 0x0003 # internal control register
        self.regs.regSensor[0x7D] = 0x0001 # internal control register
        self.regs.regSensor[0x6A] = 0xAA88 # internal control register
        self.regs.regSensor[0x6B] = 0xAC88 # internal control register
        self.regs.regSensor[0x6C] = 0x8AAA # internal control register


        logging.info ("\n\n>>NF>>")
        logging.info ("Register 0x02 is: 0x{:04x}".format(self.regs.regSensor[0x02]))
#        logging.info ("x_start (0x06) is: 0x{:04x}".format(self.regs.regSensor[0x06]))
#        logging.info ("x_end (0x07) is: 0x{:04x}".format(self.regs.regSensor[0x07]))
        logging.info ("Trying to enable Black Bar read out")

#        self.regs.regSensor[0x02] |= 0x0200 ## enable the dark columns >> Nicholas

#        self.regs.regDrkRowsStAddr = (self.VLOW_BOUNDARY - 8) * 2
#        self.regs.regNbDrkRowsTop = 8 * 2

        logging.info ("New value is: %i" % self.regs.regSensor[0x02])

        logging.info ("The lineValid token (0x52) is: 0x{:04x}".format(self.regs.regSensor[0x052]))

        logging.info ("<<NF<<\n\n")


        self.regs.regData[0x05] = 0x0007 # delay to match ADC latency
        # Configure for nominal gain.
#        self.regs.regGainSelSamp = 0x007f ## register 0x57
#        self.regs.regGainSelFb = 0x007f ## register 0x58
#        self.regs.regSerialGain = 0x03 ## register 0x58
        self.regs.regGainSelSamp = 0x003f ## register 0x57
        self.regs.regGainSelFb = 0x0001 ## register 0x58
        self.regs.regSerialGain = 0x07 ## register 0x58


        # Enable ADC offset correction.
        for x in range(0, self.ADC_CHANNELS):
            self.regs.regAdcOs[x] = 0
        self.regs.regAdcOsEn = True

        # Load the default (longest) wavetable and enable the timing engine.
        self.regs.wavetable(self.wavetables[0].wavetab)
        self.regs.regTimingEn = True
        self.__currentWavetable = self.wavetables[0]
        time.sleep(0.01)

        # Start the FPGA timing engine using the standard timing program.
        self.currentProgram = self.timing.PROGRAM_STANDARD
        self.frameClocks = int(0.001 * self.LUX2100_SENSOR_HZ)
        self.exposureClocks = int(self.frameClocks * 0.95)
        self.timing.programStandard(self.frameClocks, self.exposureClocks)
        return True

    #--------------------------------------------
    # Frame Geometry Configuration Functions
    #--------------------------------------------
    def getMaxGeometry(self):
        size = frameGeometry(
            hRes=self.MAX_HRES, vRes=self.MAX_VRES,
            hOffset=0, vOffset=0,
            vDarkRows=self.MAX_VDARK,
            bitDepth=self.BITS_PER_PIXEL)
        size.minFrameTime = self.getMinFrameClocks(size, self.wavetables[0].clocks) / self.LUX2100_SENSOR_HZ
        return size
    
    def getCurrentGeometry(self):
        # If calibration is in progress, return the saved geometry instead.
        if self.fSizeReal:
            return copy.deepcopy(self.fSizeReal)
        
        fSize = self.getMaxGeometry()
        fSize.hOffset = (self.regs.regXstart // 2) - self.HLEFT_DARK
        fSize.hRes = ((self.regs.regXend ) // 2) - self.HLEFT_DARK - fSize.hOffset + 1
        fSize.vOffset = (self.regs.regYstart // 2) - self.VLOW_BOUNDARY
        fSize.vRes = (self.regs.regYend // 2) - self.VLOW_BOUNDARY - fSize.vOffset + 1
        fSize.vDarkRows = self.regs.regNbDrkRowsTop // 2
        fSize.minFrameTime = self.getMinFrameClocks(fSize, self.__currentWavetable.clocks) / self.LUX2100_SENSOR_HZ
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
    
    def selectWavetable(self, size):
        # Select the shortest wavetable that does not result in
        # idle clocks during the row readout time.
        ideal = (size.hRes // self.HRES_INCREMENT) + self.LUX2100_MIN_HBLANK - 3
        minClocks = math.ceil(size.minFrameTime * self.LUX2100_SENSOR_HZ)
        choice = self.wavetables[0]
        wtSize = 0
        for x in self.wavetables:
            # If this wavetable is shorter than the minimum readout time, our search is done.
            if (x.clocks < ideal):
                break
            # This is the best wavetable so far.
            choice = x
            # If the user requested a slower framerate, we can stop here.
            if (self.getMinFrameClocks(size, x.clocks) <= minClocks):
                break
        return choice

    def updateWavetable(self, size, frameClocks):
        self.__currentWavetable = self.selectWavetable(size)
        
        logging.debug("Selecting WT%d for %dx%d", self.__currentWavetable.clocks, size.hRes, size.vRes)

        # If a suitable wavetable exists, then load it.
        self.regs.regRdoutDly = self.__currentWavetable.clocks
        self.regs.wavetable(self.__currentWavetable.wavetab)
        
        linePeriod = max((size.hRes // self.HRES_INCREMENT) + 2, self.__currentWavetable.clocks + 3)
        self.regs.startDelay = self.__currentWavetable.abnDelay
        self.regs.linePeriod = linePeriod - 1
        self.timing.lineDelay = self.__currentWavetable.abnDelay
        self.timing.linePeriod = linePeriod

    def updateReadoutWindow(self, size):
        # Configure the image sensor resolution
        hStartBlocks = size.hOffset // self.HRES_INCREMENT
        hEndBlocks = hStartBlocks + size.hRes // self.HRES_INCREMENT
        vLastRow = self.MAX_VRES + self.VLOW_BOUNDARY + self.VHIGH_BOUNDARY + self.MAX_VDARK
        
        # Binned operation - everything is x2 because its really a 4K sensor
        self.regs.regXstart = (self.HLEFT_DARK + hStartBlocks * self.HRES_INCREMENT) * 2
        self.regs.regXend = (self.HLEFT_DARK + hEndBlocks * self.HRES_INCREMENT) * 2 - 1
        self.regs.regYstart = (self.VLOW_BOUNDARY + size.vOffset) * 2
        self.regs.regYend = (self.VLOW_BOUNDARY + size.vOffset + size.vRes - 1) * 2
        self.regs.regDrkRowsStAddr = (vLastRow - size.vDarkRows) * 2
        self.regs.regNbDrkRowsTop = size.vDarkRows * 2

    def setResolution(self, size):
        if (not self.isValidResolution(size)):
            raise ValueError("Invalid frame resolution")
        
        # Select the minimum frame period if not specified.
        wavetab = self.selectWavetable(size)
        minClocks = self.getMinFrameClocks(size, wavetab.clocks)
        if (size.minFrameTime > (size.minFrameTime * self.LUX2100_SENSOR_HZ)):
            fClocks = math.ceil(size.minFrameTime * self.LUX2100_SENSOR_HZ)
        else:
            fClocks = minClocks

        # Disable the FPGA timing engine and wait for the current readout to end.
        self.timing.programInterm()
        time.sleep(0.01) # Extra delay to allow frame readout to finish. 

        # Switch to the desired resolution pick the best matching wavetable.
        self.regs.regTimingEn = False
        self.updateReadoutWindow(size)
        self.updateWavetable(size, frameClocks=fClocks)
        self.regs.regTimingEn = True
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
    def getMinFrameClocks(self, size, wtSize):
        # TODO: Doublecheck this against the LUX2100 datasheet.
        # Compute the minimum number of 75MHz LUX2100 sensor clocks per frame.
        # Refer to section 7.1 of the LUX1310 datasheet version v3.0
        tRead = size.hRes // self.HRES_INCREMENT
        tTx = 50        # hard-coded to 50 clocks in the FPGA, should be at least 350ns
        tRow = max(tRead + self.LUX2100_MIN_HBLANK, wtSize + 3)
        tFovf = self.LUX2100_SOF_DELAY + wtSize + self.LUX2100_LV_DELAY + 10
        tFovb = 50      # Duration between PRSTN falling and TXN falling (I think)
        tFrame = tRow * (size.vRes + size.vDarkRows) + tTx + tFovf + tFovb - self.LUX2100_MIN_HBLANK

        # Add the overhead to flush the deserializer FIFO to DDR memory, otherwise
        # we will get frame corruption at the maximum framerate. I am reasonably
        # sure that this is a bug in the FPGA.
        tFrame += 280   # Approximately 3.75us

        return tFrame
    
    def getPeriodRange(self, fSize):
        # If a frame time was provided, find the longest matching wavetable.
        wavetab = self.selectWavetable(fSize)
        fClocks = self.getMinFrameClocks(fSize, wavetab.clocks)
        return (fClocks / self.LUX2100_SENSOR_HZ, 0)
    
    def getCurrentPeriod(self):
        return self.frameClocks / self.LUX2100_SENSOR_HZ

    def setFramePeriod(self, fPeriod):
        # TODO: Sanity-check the frame period.
        self.currentProgram = self.timing.PROGRAM_STANDARD
        self.frameClocks = math.ceil(fPeriod * self.LUX2100_SENSOR_HZ)
        self.timing.programStandard(self.frameClocks, self.exposureClocks)
    
    def getExposureRange(self, fSize):
        # Defaulting to 1us minimum exposure and infinite maximum exposure.
        if (fSize.minFrameTime):
            return (1.0 / 1000000, fSize.minFrameTime - (500 / self.LUX2100_SENSOR_HZ))
        else:
            wavetab = self.selectWavetable(fSize)
            fClocks = self.getMinFrameClocks(fSize, wavetab.clocks)
            return (1.0 / 1000000, (fClocks - 500) / self.LUX2100_SENSOR_HZ)

    def getCurrentExposure(self):
        return self.exposureClocks / self.LUX2100_SENSOR_HZ
    
    def setExposureProgram(self, expPeriod):
        # TODO: Sanity-check the exposure time.
        if (expPeriod < 1.0 / 1000000):
            expPeriod = 1.0 / 1000000
        
        self.currentProgram = self.timing.PROGRAM_STANDARD
        self.exposureClocks = math.ceil(expPeriod * self.LUX2100_SENSOR_HZ)
        self.timing.programStandard(self.frameClocks, self.exposureClocks)

    #--------------------------------------------
    # Advanced Exposure and Timing Functions 
    #--------------------------------------------
    def getSupportedExposurePrograms(self):
        return ("normal", "frameTrigger", "shutterGating")

    def setShutterGatingProgram(self):
        self.currentProgram = self.timing.PROGRAM_SHUTTER_GATING
        self.timing.programShutterGating()
    
    def setFrameTriggerProgram(self, expPeriod):
        if (expPeriod < 1.0 / 1000000):
            expPeriod = 1.0 / 1000000
        
        self.currentProgram = self.timing.PROGRAM_FRAME_TRIG
        self.exposureClocks = math.ceil(expPeriod * self.LUX2100_SENSOR_HZ)
        self.timing.programTriggerFrames(self.frameClocks, self.exposureClocks)
    
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
    
    @property
    def wbPresets(self):
        return {
            3200: [1.02, 1.00, 1.91],
            4600: [1.22, 1.00, 1.74],
            5250: [1.30, 1.00, 1.61],
            5600: [1.35, 1.00, 1.584],
            6500: [1.42, 1.00, 1.46],
            8000: [1.53, 1.00, 1.35],
        }

    @property
    def maxGain(self):
        return 16
    
    def DISABLEsetGain(self, gain):
        gainConfig = {  # Sampling Cap, Feedback Cap, Serial Gain
            1:          ( 0x007f,       0x007f,       0x3),
            2:          ( 0x01ff,       0x000f,       0x3),
            4:          ( 0x0fff,       0x001f,       0x1),
            8:          ( 0x0fff,       0x001f,       0x0),
            16:         ( 0x0fff,       0x000f,       0x0),
        }
        if (not int(gain) in gainConfig):
            raise ValueError("Unsupported image gain setting")
        
        samp, feedback, sgain = gainConfig[int(gain)]
        self.regs.regGainSelSamp = samp
        self.regs.regGainSelFb = feedback
        self.regs.regSerialGain = sgain

    def setGain(self, gain):
        gainConfig = { ## sampling caps, feedback caps, serial caps, icol caps
#            1:          ( 0x000f,   0x0001,       0x7,  0x1 ), ## new settings for x1
#            1:          ( 0x001f,   0x000f,       0x7,  0x3 ), ## alternate acceptable x1 settings

            1:          ( 0x003f,   0x0001,       0x7,  0x1 ), ## new settings for x1 (gain is ~1.25)
#            1:          ( 0x07ff,   0x0003,       0x7,  0x1 ), ## alternate acceptable x1 settings (gain is ~1.25)
#            1:          ( 0x0001,   0x0000,       0x7,  0x1 ), ## alternate acceptable x1 settings (gain is ~1.25)

            2:          ( 0x0fff,   0x0001,       0x7,  0x1 ), ## new settings for x2
#            2:          ( 0x0fff,   0x0007,       0x7,  0x3 ), ## alternate acceptable x2 settings

            4:          ( 0x0fff,   0x0001,       0x7,  0x3 ), ## new settings for x4 (gain is ~4.0)
#            4:          ( 0x07ff,   0x0003,       0x7,  0x7 ), ## alternate acceptable x4 settings

            8:          ( 0x07ff,   0x0000,       0x7,  0x3 ), ## new settings for x8 (gain is ~7.5)
#            8:          ( 0x003f,   0x0000,       0x7,  0x7 ), ## alternate acceptable x8 settings

            16:         ( 0x01ff,   0x0000,       0x7,  0x7 ), ## actual gain ~9.75
#            16:         ( 0x03ff,   0x0000,       0x7,  0x7 ), ## actual gain ~10.5 (alternate acceptable settings)
#            16:         ( 0x001f,   0x0000,       0x7,  0xf ), ## actual gain ~9.0 (alternate acceptable settings)

            10:         ( 0x007f,   0x007f,       0x3,  0x7 ), ## previous default x1 setting
            20:         ( 0x01ff,   0x000f,       0x3,  0x7 ), ## previous default x2 setting
            40:         ( 0x0fff,   0x001f,       0x1,  0x7 ), ## previous default x4 setting
            80:         ( 0x0fff,   0x001f,       0x0,  0x7 ), ## previous default x8 setting
            160:        ( 0x0fff,   0x000f,       0x0,  0x7 ), ## previous default x16 setting

        }
        logging.info (">> NF >> Setting Analog Gain to: %i" % gain)

        if (not int(gain) in gainConfig):
            raise ValueError("Unsupported image gain setting")

        samp, feedback, sgain, icol = gainConfig[int(gain)] ## copy one of the lines above
        self.regs.regGainSelSamp = samp ## sample caps setting
        self.regs.regGainSelFb = feedback ## feedback caps setting
        self.regs.regSerialGain = sgain ## serial gain register caps setting
##        self.regs.regIcolCapEn = icol ## icol caps setting

         ## For some reason, when I try writing to just the "regIcolCapEn" register, it very occasionally
         ## completely messes up the image. This seems to be due to the other bits in this register getting
         ## affected somehow. Perhaps writing to it ("regIcolCapEn") only works most of the time. This
         ## is the same register as "regIcolCapEn", but I can write an entire byte (with regIcolCapEn, it
         ## only writes the upper nibble [mask is 0x00f0]).
        self.regs.regSerialGainV2 = ( 0x0010 * icol ) + 0x0009 ## icol caps setting (also write some of the "reserved" mystery bits)



    def getCurrentGain(self):
        sampnbits = 4
        fbacknbits = 1
        gsernbits = 1
        icolnbits = 0

        x = self.regs.regGainSelSamp
        while (x != 0):
            sampnbits += (x & 1)
            x >>= 1
        
        x = self.regs.regGainSelFb
        while (x != 0):
            fbacknbits += (x & 1)
            x >>= 1

        x = self.regs.regSerialGain
        while (x != 0):
            gsernbits += (x & 1)
            x >>= 1
       
        x = self.regs.regIcolCapEn
        while (x != 0):
            icolnbits += (x & 1)
            x >>= 1

        return (sampnbits * icolnbits) / (gsernbits * fbacknbits)

    def __backupSettings(self):
        # Save the sensor settings, before they get messed up by calibration.
        self.fSizeReal = self.getCurrentGeometry()

    def __restoreSettings(self):
        # Restore the sensor settings after they get messed up by calibration.
        logging.debug("Restoring sensor configuration")
        self.timing.programInterm()
        time.sleep(0.01) # Extra delay to allow frame readout to finish. 
        self.regs.regTimingEn = False
        self.updateReadoutWindow(self.fSizeReal)
        self.regs.regTimingEn = True
        time.sleep(0.01)
        self.fSizeReal = None

        # Restore the timing program
        if (self.currentProgram == self.timing.PROGRAM_STANDARD):
            self.timing.programStandard(self.frameClocks, self.exposureClocks)
        elif (self.currentProgram == self.timing.PROGRAM_SHUTTER_GATING):
            self.timing.programShutterGating()
        elif (self.currentProgram == self.timing.PROGRAM_FRAME_TRIG):
            self.timing.programTriggerFrames(self.frameClocks, self.exposureClocks)
        else:
            logging.error("Invalid timing program, reverting to standard exposure")
            self.timing.programStandard(self.frameClocks, self.exposureClocks)

    def startAnalogCal(self, saveLocation=None):
        logging.info('Starting ADC gain calibration')

        # Setup some math constants
        numRows = 64
        fSize = self.getCurrentGeometry()
        tRefresh = (self.frameClocks * 10) / self.LUX2100_SENSOR_HZ
        pixFullScale = (1 << fSize.bitDepth)

        seq = sequencer()
        colGainRegs = pychronos.fpgamap(pychronos.FPGA_COL_GAIN_BASE, 0x1000)
        colCurveRegs = pychronos.fpgamap(pychronos.FPGA_COL_CURVE_BASE, 0x1000)

        ## Nicholas >> turn off the black-bar data from the sensor (and therefore the black-bar corrections)
        self.regs.regSensor[0x02] &= ~0x0200 ## disable the dark columns >> Nicholas


        # Enable the analog test mode.
        self.regs.regPoutsel = 3
        self.regs.regSelVdum = 3
        self.regs.regSelVlnkeepRst = 0

        # Search for a dummy voltage high reference point.
        vhigh = 31 ## Nicholas >> played with this from default of 31
        while (vhigh > 0):
            self.regs.regSelVlnkeepRst = vhigh
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
            self.regs.regSelVlnkeepRst = vlow
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


        ## Nicholas >> turn back on the black-bar data from the sensor (and therefore the black-bar corrections)
#        self.regs.regSensor[0x02] |= 0x0200 ## enable the dark columns >> Nicholas


        # Disable the analog test mode.
        self.regs.regSelVlnkeepRst = 30
        self.regs.regSelVdum = 0
        self.regs.regPoutsel = 2

        logging.info("ADC Gain calibration voltages=[%s, %s]" % (vlow, vhigh))
        logging.info("ADC Gain calibration averages=[%s, %s]" %
                (numpy.average(lowColumns), numpy.average(highColumns)))

        # Determine which column has the strongest response and sanity-check the gain
        # measurements. If things are out of range, then give up on gain calibration
        # and apply a gain of 1.0 instead.
        maxColumn = 0
        for col in range(0, self.ADC_CHANNELS):
            logging.info(">> NF >> ADC calibration column: %i" % col)
            minrange = (pixFullScale // 16)
            diff = highColumns[col] - lowColumns[col]
            if (highColumns[col] <= (lowColumns[col] + minrange)):
                for x in range(0, self.MAX_HRES):
                    colGainRegs.mem16[x] = (1 << self.COL_GAIN_FRAC_BITS)
                    colCurveRegs.mem16[x] = 0
                raise CalibrationError("ADC Auto calibration range error")
            if (diff > maxColumn):
                maxColumn = diff

        # Compute the 2-point calibration coefficient.
        diff = (highColumns - lowColumns)
        gain2pt = numpy.full(self.ADC_CHANNELS, maxColumn) / diff
        logging.info("ADC Columns 2-point gain: %s" % (gain2pt))
        
        # Load the 2-point gain calibration 
        colGainRegs = pychronos.fpgamap(pychronos.FPGA_COL_GAIN_BASE, 0x1000)
        colCurveRegs = pychronos.fpgamap(pychronos.FPGA_COL_CURVE_BASE, 0x1000)
        for col in range(self.MAX_HRES):
            colGainRegs.mem16[col] = int(gain2pt[col % self.ADC_CHANNELS] * (1 << self.COL_GAIN_FRAC_BITS))
            colCurveRegs.mem16[col] = 0
        
        display = pychronos.regmaps.display()
        display.gainControl &= ~display.GAINCTL_3POINT

        # Save the black calibration results if a location was provided.
        if saveLocation:
            colGainData = numpy.asarray(gain2pt, dtype=numpy.float)
            colGainData.tofile(self.calFilename(saveLocation + '/colGain', ".bin"))

    def loadAnalogCal(self, calLocation):
        # Load calibration!
        display = pychronos.regmaps.display()
        colGainRegs = pychronos.fpgamap(pychronos.FPGA_COL_GAIN_BASE, 0x1000)
        colCurveRegs = pychronos.fpgamap(pychronos.FPGA_COL_CURVE_BASE, 0x1000)
      
        # Load the factory column gain calibration, using one column gain coefficient per horizontal pixel.
        # Generate the calibration filename.
        wtClocks = self.regs.regRdoutDly
        gain = int(self.getCurrentGain())
        if gain > 8:
            gain = 16 # HACK: G16 is a bit wonky, and actually comes out closer to 10
        filename = calLocation + "/factory_colGain_G%d_WT%d.bin" % (gain, wtClocks)
        try:
            logging.info("Loading column gain calibration from %s", filename)
            colGainData = numpy.fromfile(filename, dtype=numpy.int32, count=self.MAX_HRES)
            for col in range(0, self.MAX_HRES):
                colGainRegs.mem16[col] = int(colGainData[col])
                colCurveRegs.mem16[col] = 0
            display.gainControl &= ~display.GAINCTL_3POINT
            return True   
        except Exception as err:
            logging.info("Couldn't load factory col gain, falling back to auto 2-point col gain.")         

        # If the factory calibration file is missing, fall back to 2-point cal data, using one column gain coefficient per ADC channel.
        # Generate the calibration filename.
        filename = calLocation + self.calFilename("/colGain", ".bin")
        try:
            logging.info("Loading column gain calibration from %s", filename)
            colGainData = numpy.fromfile(filename, dtype=numpy.float, count=self.ADC_CHANNELS)
            for col in range(0, self.MAX_HRES):
                colGainRegs.mem16[col] = int(colGainData[col % self.ADC_CHANNELS] * (1 << self.COL_GAIN_FRAC_BITS))
                colCurveRegs.mem16[col] = 0
            display.gainControl &= ~display.GAINCTL_3POINT
            return True
        except Exception as err:
            logging.info("Clearing column gain calibration data")
            for col in range(0, self.MAX_HRES):
                colGainRegs.mem16[col] = (1 << self.COL_GAIN_FRAC_BITS)
                colCurveRegs.mem16[col] = 0
            display.gainControl &= ~display.GAINCTL_3POINT
            return False
    
    def autoAdcOffsetIteration(self, fSize, numFrames=4):
        # Read out the calibration frames.
        fAverage = numpy.zeros((fSize.hRes // self.ADC_CHANNELS * 4, self.ADC_CHANNELS), dtype=numpy.uint32)
        seq = sequencer()
        for x in range(0, numFrames):
##            yield from seq.startLiveReadout(fSize.hRes, 1)
            yield from seq.startLiveReadout(fSize.hRes, 4)
            if not seq.liveResult:
                logging.error("ADC offset training failed to read frame")
                return
            fAverage += numpy.reshape(seq.liveResult, (-1, self.ADC_CHANNELS))
        
#        fAverage = fAverage[fAverage.shape[0]/2:, :] ## ignore the first half of the bars (keep only the lower 4)
        # Train the ADC offsets for a target of Average = Footroom + StandardDeviation
        fAverage //= numFrames
        adcAverage = numpy.average(fAverage, 0) ## average of each column
        adcStdDev = numpy.std(fAverage, 0) ## standard deviation of each column
        for col in range(0, self.ADC_CHANNELS):
            self.adcOffsets[col] -= (adcAverage[col] - adcStdDev[col] - self.ADC_FOOTROOM) / 2
#            self.adcOffsets[col] -= (adcAverage[col] - self.ADC_FOOTROOM) / 2 ## Nicholas >> try without subtracting the standard deviation
            logging.info(">> NF >> column: %i,  adcAverage: %i,  offsets: %i" % (col, adcAverage[col], self.adcOffsets[col]))
            if (self.adcOffsets[col] < self.ADC_OFFSET_MIN):
                self.adcOffsets[col] = self.ADC_OFFSET_MIN
            elif (self.adcOffsets[col] > self.ADC_OFFSET_MAX):
                self.adcOffsets[col] = self.ADC_OFFSET_MAX
            
            # Update the image sensor
            self.regs.regAdcOs[col] = self.adcOffsets[col]
    
    def startBlackCal(self, saveLocation=None):
        logging.info('Starting ADC offset calibration')
        self.__backupSettings()

        # Retrieve the current resolution and frame period.
        fSize = copy.deepcopy(self.fSizeReal)
        fPeriod = self.frameClocks / self.LUX2100_SENSOR_HZ
        iterations=16

## >> note from Nicholas >> the black bars along the top of the sensor are pretty close to useless, so I'm experimenting with not even using them
        # Enable black bars if not already done.
#        if (fSize.vDarkRows == 0):
#            logging.info("Enabling dark pixel readout")
#            fSize.vDarkRows = self.MAX_VDARK // 2
#            fSize.vOffset += fSize.vDarkRows
#            fSize.vRes -= fSize.vDarkRows

            # Disable the FPGA timing engine and apply the changes.
#            self.timing.programInterm()
#            time.sleep(0.01) # Extra delay to allow frame readout to finish. 
#            self.regs.regTimingEn = False
#            self.updateReadoutWindow(fSize)
#            self.regs.regTimingEn = True
#            time.sleep(0.01)
#            self.timing.programStandard(self.frameClocks, self.exposureClocks)
        
        # Perform ADC offset calibration using the optical black regions.
        tRefresh = (self.frameClocks * 3) / self.LUX2100_SENSOR_HZ
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
        
        # Save the black calibration results if a location was provided.
        if saveLocation:
            filename = "/offset_%dx%doff%dx%d" % (self.fSizeReal.hRes, self.fSizeReal.vRes, self.fSizeReal.hOffset, self.fSizeReal.vOffset)
            adcOffsetData = numpy.asarray(self.adcOffsets, dtype=numpy.int16)
            logging.info(">>NF>> adcOffsetData is size: %s" % adcOffsetData.shape)
            adcOffsetData.tofile(self.calFilename(saveLocation + filename, ".bin"))

        # Restore the frame period and wavetable.
        self.__restoreSettings()
    
    def loadBlackCal(self, calLocation, factoryLocation=None):
        # Generate the calibration filename.
        fSize = self.getCurrentGeometry()
        suffix = self.calFilename("/offset_%dx%doff%dx%d" % (fSize.hRes, fSize.vRes, fSize.hOffset, fSize.vOffset), ".bin")
        filename = calLocation + suffix
        if (factoryLocation and not os.path.isfile(filename)):
            filename = factoryLocation + suffix

        # Load calibration!
        try:
            logging.info("Loading offset calibration from %s", filename)
            adcOffsetData = numpy.fromfile(filename, dtype=numpy.int16, count=self.ADC_CHANNELS)
#            adcOffsetData = [0] * ADC_CHANNELS ## Nicholas
            for i in range(0, self.ADC_CHANNELS):
                self.adcOffsets[i] = adcOffsetData[i]
                self.regs.regAdcOs[i] = adcOffsetData[i]
            return True
        except Exception as err:
            logging.info("Clearing offset calibration data")
            for i in range(0, self.ADC_CHANNELS):
                self.adcOffsets[i] = 0
                self.regs.regAdcOs[i] = 0
            return False

    def calFilename(self, prefix, extension=""):
        wtClocks = self.regs.regRdoutDly
        gain = int(self.getCurrentGain())
        if gain > 8:
            gain = 16 # HACK: G16 is a bit wonky and comes out closer to 10
        return "%s_G%d_WT%d%s" % (prefix, gain, wtClocks, extension)

    def IGNOREstartFlatFieldExport(self, saveLocation='/media/sda1'):
        logging.info('Starting flat-field export')

        display = pychronos.regmaps.display()
        sensor = pychronos.regmaps.sensor()
        seq = sequencer()
        hRes = pychronos.sensors.lux2100().getMaxGeometry().hRes
        vRes = pychronos.sensors.lux2100().getMaxGeometry().vRes

        # Attempt to use the serial number as the save folder if valid
        serialNum = pychronos.camera(self).cameraSerial
        try:
            int(serialNum)
        except ValueError:
            logging.error("Serial Number invalid, defaulting to Chronos21")
            serialNum = 'Chronos21'

        gainSampCap =   [0x007F, 0x01FF, 0x0FFF, 0x0FFF, 0x0FFF]
        gainSerFbCap =  [0x037F, 0x030F, 0x011F, 0x001F, 0x000F]

        adcTestModeVoltages = [
            [0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34],   #x1, 0 dB
            [0x2D, 0x2E, 0x2F, 0x30, 0x31],                     #x2, 6 dB
            [0x2D, 0x2E, 0x2F, 0x30],                           #x4, 12 dB
            [0x2D, 0x2E, 0x2F],                                 #x8, 18 dB
            [0x2D, 0x2E, 0x2F]                                  #x16,24 dB
        ]


        ## Nicholas >> turn off the black-bar data from the sensor (and therefore the black-bar corrections)
        self.regs.regSensor[0x02] &= ~0x0200 ## disable the dark columns >> Nicholas

        # Disable certain FPGA overlays to get raw sensor values.
        display.pipeline |= display.BYPASS_GAIN | display.BYPASS_GAMMA_TABLE | display.BYPASS_DEMOSAIC | display.BYPASS_FPN

        # Enter test mode.
        self.regs.regPoutsel = 1

        for gain in range(0, len(adcTestModeVoltages)):
            # Create a folder for each level of analog gain.

            gainPath = saveLocation + "/%s/x%d/" % (serialNum, 1 << gain)
            try:
                os.makedirs(gainPath, exist_ok=True)
                logging.info('Saving flat fields to ' + gainPath)
            except OSError as err:
                logging.error("Could not create per-gain folders while exporting cal data.")
                return False

            # Set analog gain value
            self.regs.regGainSelSamp = gainSampCap[gain]
            self.regs.regGainSelFb = gainSerFbCap[gain]
#            self.setGain(2**gain) ## set the analog gain level

            # Iterate and collect flat-fields at each level of ADC test voltage step.
            for intensity in range(0, len(adcTestModeVoltages[gain])):

                # Inject a test voltage into the ADCs, see lux2100 datasheet p.30.
                testVoltage = (adcTestModeVoltages[gain][intensity] << 8) + 0x0011
                sensor.sciWrite(0x67, testVoltage)

                # Get the average of 3 frames.
                numFrameSamples = 8
                fAverage = numpy.zeros((vRes, hRes), dtype=numpy.uint16)

                for i in range(0, numFrameSamples):
                    yield from seq.startLiveReadout(hRes, vRes)
                    if not seq.liveResult:
                        logging.error("Flat field export failed to read frame.")
                        return False
                    fAverage += numpy.asarray(seq.liveResult)

                # Save a .raw frame as a numpy array, which guarantees platform independence for the data order.
                fName = gainPath + "%d.npy" % intensity
                outFrame = numpy.array(fAverage / numFrameSamples, dtype=numpy.uint16)
                numpy.save(fName, outFrame)

        ## Nicholas >> turn back on the black-bar data from the sensor (and therefore the black-bar corrections)
#        self.regs.regSensor[0x02] |= 0x0200 ## enable the dark columns >> Nicholas

        return True

    def startFlatFieldExport(self):
        logging.info('Starting Nicholas\'s On-Camera Calibration Calculation')

        display = pychronos.regmaps.display()
        sensor = pychronos.regmaps.sensor()
        seq = sequencer()
        hRes = pychronos.sensors.lux2100().getMaxGeometry().hRes
        vRes = pychronos.sensors.lux2100().getMaxGeometry().vRes


        adcTestModeVoltages = [
#            [0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34],   #x1, 0 dB
#            [0x2D, 0x2E, 0x2F, 0x30, 0x31],                     #x2, 6 dB
#            [0x2D, 0x2E, 0x2F, 0x30],                           #x4, 12 dB
#            [0x2D, 0x2E, 0x2F],                                 #x8, 18 dB
#            [0x2D, 0x2E, 0x2F]                                  #x16,24 dB

 ## on-cam calibration settings 1
            [0x2E, 0x34], #x1, 0dB
            [0x2E, 0x31], #x2, 6dB
            [0x2E, 0x30], #x4, 12dB
            [0x2E, 0x2F], #x8, 18dB
            [0x2E, 0x2F]  #x16, 24dB

 ## on-cam calibration settings 2
#            [0x2D, 0x34], #x1, 0 dB
#            [0x2D, 0x31], #x2, 6 dB
#            [0x2D, 0x30], #x4, 12 dB
#            [0x2D, 0x2F], #x8, 18 dB
#            [0x2D, 0x2F]  #x16,24 dB

 ## on-cam calibration settings 3
#            [0x2D, 0x2E], #x1, 0 dB
#            [0x2D, 0x2E], #x2, 6 dB
#            [0x2D, 0x2E], #x4, 12 dB
#            [0x2D, 0x2E], #x8, 18 dB
#            [0x2D, 0x2E]  #x16,24 dB
        ]

        bright = 1
        dim = 0

        ## Nicholas >> turn off the black-bar data from the sensor (and therefore the black-bar corrections)
        self.regs.regSensor[0x02] &= ~0x0200 ## disable the dark columns >> Nicholas

        # Disable certain FPGA overlays to get raw sensor values.
        display.pipeline |= display.BYPASS_GAIN | display.BYPASS_GAMMA_TABLE | display.BYPASS_DEMOSAIC | display.BYPASS_FPN | display.BYPASS_COLOR_MATRIX

        # Enter test mode.
        self.regs.regPoutsel = 1

        for gain in range(0, len(adcTestModeVoltages)):

            logging.info("Starting tests for gain: x%i" % (2**gain))

            # Set analog gain value
            self.setGain(2**gain) ## set the analog gain level


            # Inject a test voltage into the ADCs, see lux2100 datasheet p.30.
            testVoltage = (adcTestModeVoltages[gain][dim] << 8) + 0x0011
            sensor.sciWrite(0x67, testVoltage)

            # Get the average of some frames.
            numFrameSamples = 16
            dimAverage = numpy.zeros((numFrameSamples, hRes)) ## set up a numpy array to store the column averages for each frame

            for i in range(0, numFrameSamples):
                logging.info("Getting <Dim> sample %i of %i" % (i, numFrameSamples))
                yield from seq.startLiveReadout(hRes, vRes)
                if not seq.liveResult:
                    logging.error("NF Calibration failed to read frame.")
                    return False
                dimAverage[i, :] = numpy.mean(numpy.asarray(seq.liveResult), axis=0) ## keep the mean of each column

            dimAverage = numpy.mean(dimAverage, axis=0) ## average all the samples for the total average value of each column


            testVoltage = (adcTestModeVoltages[gain][bright] << 8) + 0x011
            sensor.sciWrite(0x67, testVoltage)

            brightAverage = numpy.zeros((numFrameSamples, hRes)) ## set up a numpy array to store the column averages for each frame

            for i in range(0, numFrameSamples):
                logging.info("Getting <Bright> sample %i of %i" % (i, numFrameSamples))
                yield from seq.startLiveReadout(hRes, vRes)
                if not seq.liveResult:
                    logging.error("NF Calibration failed to read frame.")
                    return False
                brightAverage[i, :] = numpy.mean(numpy.asarray(seq.liveResult), axis=0) ## keep the mean of each column

            brightAverage = numpy.mean(brightAverage, axis=0) ## average all the samples for the total average value of each column


            logging.info ("now, I'll compute the gain")

            columnGain = brightAverage - dimAverage ## find the difference between these brightness levels

            columnGain = numpy.mean(columnGain) / columnGain ## calculate the gain for the sensor

            logging.info("got values of: %s" % columnGain)

            logging.info("Gain calculated; ensuring values are within range")

            columnGain[columnGain > 1.9] = 1 ## if the gain is too high, just use a gain of 1

            columnGain[columnGain < 0.5] = 1 ## if the gain is too low, just use a gain of 1


            logging.info("Now, I'm re-packing the data into a file for export")

            saveGainArray = (4096 * columnGain).astype('int32') ## convert to a fixed-point number that can be stored and decoded later

            outputFileName = "/var/camera/cal/factory_colGain_G%i_WT66.bin" % (2**gain) ## I think this should work...

            with open(outputFileName, "wb") as colGainFileNF: ## save the file
                colGainFileNF.write(pack("<7680B", *bytearray(saveGainArray)))


        ## Nicholas >> turn back on the black-bar data from the sensor (and therefore the black-bar corrections)
#        self.regs.regSensor[0x02] |= 0x0200 ## enable the dark columns >> Nicholas

        logging.info("Finished Nicholas\'s calibration, now I'm putting things back")

        self.regs.regSelVlnkeepRst = 30
        self.regs.regSelVdum = 0
        self.regs.regPoutsel = 2

        display.pipeline &= ~(display.BYPASS_GAIN | display.BYPASS_GAMMA_TABLE | display.BYPASS_DEMOSAIC | display.BYPASS_FPN | display.BYPASS_COLOR_MATRIX)

        return True


    def importColGains(self, sourceLocation='/media/sda1', calLocation='/var/camera/cal'):
        # Copy column gain calibration data into camera.
        gainLvls = [1, 2, 4, 8, 16]
        wtClocks = [66, 45, 35, 25]
        copyCount = 0

        # Expect the source folder to be named as the serial number of the current camera
        serialNum = pychronos.camera(self).cameraSerial
        try:
            int(serialNum)
            sourceLocation = sourceLocation + '/' + serialNum
        except ValueError:
            logging.error("Invalid serial number. Will attempt to import from %s", sourceLocation)

        for wt in range(0, len(wtClocks)):
            for gain in range(0, len(gainLvls)):
                #Build filename string
                fName = 'factory_colGain_G%d_WT%d.bin' % (gainLvls[gain], wtClocks[wt])
                try:
                    shutil.copy(os.path.join(sourceLocation, fName), os.path.join(calLocation, fName))
                    logging.info("Copied %s from %s to %s", fName, sourceLocation, calLocation)
                    copyCount += 1
                except OSError as err:
                    logging.error("Could not copy %s from %s to %s", fName, sourceLocation, calLocation)

        if (copyCount == len(gainLvls) * len(wtClocks)):
            return True
        else:
            return False
