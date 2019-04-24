#Luxima LUX1310 Image Sensor Class
import pychronos
import time
import math
import copy
import numpy
import logging

from pychronos.regmaps import sequencer
from pychronos.sensors import api, frameGeometry

class lux2100(api):
    """Driver for the Luxima LUX1310 image sensor.

    The board dictionary may provide the following pins:
        lux2100-spidev: Path to the spidev driver for the voltage DAC.
        lux2100-dac-cs: Path to the chip select for the voltage DAC.

    Parameters
    ----------
    board : `dict`
        Name/value pairs listing the hardware wiring of the board.
    """

    # Image sensor geometry constraints
    MAX_HRES = 1920
    MAX_VRES = 1080
    MIN_HRES = 1920
    MIN_VRES = 1080
    HRES_INCREMENT = 32
    VRES_INCREMENT = 2
    MAX_VDARK = 8
    BITS_PER_PIXEL = 12

    LUX2100_SENSOR_HZ = 75000000
    LUX2100_TIMING_HZ = 100000000
    ADC_CHANNELS = 32
    ADC_OFFSET_MIN = -1023
    ADC_OFFSET_MAX = 1023

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
    }

    DAC_FULL_SCALE  = 4095
    DAC_VREF        = 3.3
    DAC_AUTOUPDATE  = 0x9

    def writeDAC(self, dac, voltage):
        """Write the DAC voltage"""
        # Convert the DAC value
        dacdata=bytearray([0, DAC_AUTOUPDATE << 4, 0, DAC_AUTOUPDATE << 4])
        mul, div, offs = self.dacmap[dac]
        if (offs):
            voltage = offs - voltage
        
        dacval = int((voltage * self.DAC_FULL_SCALE * mul) / (self.DAC_VREF * div))
        if ((dacval < 0)  or (dacval > self.DAC_FULL_SCALE)):
            raise ValueError("DAC Voltage out of range")
        
        if (dac < 8):
            dacval |= (dac << 12)
            dacdata[0] = (dacval & 0x00ff) >> 0
            dacdata[1] = (dacval & 0xff00) >> 8
        else:
            dacval |= ((dac-8) << 12)
            dacdata[2] = (dacval & 0x00ff) >> 0
            dacdata[3] = (dacval & 0xff00) >> 8
        
        # Write the DAC value to the SPI.
        pychronos.writespi(device=self.spidev, csel=self.spics, mode=1, bitsPerWord=16, data=dacdata)

    @property
    def name(self):
        return "LUX2100"
    
    @property
    def cfaPattern(self):
        # TODO: Color Detection
        return None

    def __init__(self, board={
            "lux2100-spidev":  "/dev/spidev3.0",
            "lux2100-dac-cs": "/sys/class/gpio/gpio33/value"} ):
        ## Hardware Resources
        self.spidev = board["lux2100-spidev"]
        self.spics = board["lux2100-dac-cs"]
        self.board = board
        self.regs = lux1310regs.lux1310regs()

        ## ADC Calibration state
        self.adcOffsets = [0] * self.HRES_INCREMENT

        super().__init__()

    @property
    def name(self):
        return "LUX2100"
    
    @property
    def cfaPattern(self):
        # FIXME: Need a way to check for color/mono
        return ['R', 'G', 'G', 'B']
    
    #--------------------------------------------
    # Sensor Configuration and Control API
    #--------------------------------------------
    def reset(self, fSize=None):
        # TODO: Implement Me!
        pass

    #--------------------------------------------
    # Frame Geometry Configuration Functions
    #--------------------------------------------
    def getMaxGeometry(self):
        return frameGeometry(
            hRes=self.MAX_HRES, vRes=self.MAX_VRES,
            hOffset=0, vOffset=0,
            vDarkRows=self.MAX_VDARK,
            bitDepth=self.BITS_PER_PIXEL)
        
    def getCurrentGeometry(self):
        return frameGeometry(
            hRes=self.MAX_HRES, vRes=self.MAX_VRES,
            hOffset=0, vOffset=0, vDarkRows=0,
            bitDepth=self.BITS_PER_PIXEL)
    
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

        # Convert from LUX1310 sensor clocks to FPGA timing clocks.
        return (tFrame * self.LUX2100_TIMING_HZ) // self.LUX1310_SENSOR_HZ
    
    def getPeriodRange(self, fSize):
        # TODO: Need to validate the frame size.
        # TODO: Probably need to enforce some maximum frame period.
        clocks = self.getMinFrameClocks(fSize)
        return (clocks / self.LUX2100_TIMING_HZ, 0)
    
    def getCurrentPeriod(self):
        return self.timing.frameTime / self.LUX2100_TIMING_HZ

    def setFramePeriod(self, fPeriod):
        # TODO: Sanity-check the frame period.
        logging.debug('frame time: %f', math.ceil(fPeriod * self.LUX2100_TIMING_HZ))
        self.timing.frameTime = math.ceil(fPeriod * self.LUX2100_TIMING_HZ)
    
    def getExposureRange(self, fSize, fPeriod):
        # Defaulting to 1us minimum exposure and infinite maximum exposure.
        # TODO: Need a better handle on the exposure overhead.
        return (1.0 / 1000000, fPeriod - (500 / self.LUX2100_TIMING_HZ))

    def getCurrentExposure(self):
        return self.timing.integrationTime / self.LUX2100_TIMING_HZ
    
    def setExposurePeriod(self, expPeriod):
        # TODO: Sanity-check the exposure time.
        self.timing.integrationTime = math.ceil(expPeriod * self.LUX2100_TIMING_HZ)
