# Chronos High-Speed Camera class
import fcntl
import time
import os
import numpy
import logging
import datetime

import pychronos
import pychronos.regmaps as regmaps
import pychronos.spd as spd

from . import utils

__propertiesACameraPropertyCanHave = {'notify', 'save'}
def camProperty(notify=False, save=False, prio=0):
    """@camProperty: Like @property, but include metadata.
        
        The camera properties, themselves, have certain
        properties we care about. They are all gettable,
        which we specify with a getter using @property,
        and can be settable. However, they can also emit a
        notification signal when they change, which is not
        a decorated feature. Some properties are derived
        from other properties, and we don't want to save
        those to disk. This decorator notes these last two
        properties for use later.
        
        'notify': The property emits update events when it
                  is changed (default: False)
        'save':   The property can be saved to disk to
                  preserve the configuration of the camera
                  class (default: False)
        'prio':   The sort order to apply when setting multiple
                  properties at once. High numbers should be
                  set first (default: 0)
        
        Examples:
            set:
                @camProperty(notify=True, save=True)
                def exposurePeriod(self):
                    return ...
            get:
                type(self.camera).exposurePeriod
                    .fget.isNotifiable
        """
    def camPropertyAnnotate(fn, *args, **kwargs):
        """Helper function for camProperty decorator."""
        setattr(fn, 'notifies', notify)
        setattr(fn, 'saveable', save)
        setattr(fn, 'prio', prio)
        return property(fn, *args, **kwargs)
    
    return camPropertyAnnotate




class CameraError(RuntimeError):
    pass

# Parameter priority groups
PARAM_PRIO_RESOLUTION = 3
PARAM_PRIO_FRAME_TIME = 2
PARAM_PRIO_EXPOSURE   = 1

# Recording LEDs
REC_LED_FRONT = "/sys/class/gpio/gpio41/value"
REC_LED_BACK = "/sys/class/gpio/gpio25/value"

class camera:
    BYTES_PER_WORD = 32
    FRAME_ALIGN_WORDS = 64

    MAX_FRAME_WORDS = 0x10000
    CAL_REGION_START = 0
    CAL_REGION_FRAMES = 3
    LIVE_REGION_START = (CAL_REGION_START + MAX_FRAME_WORDS * CAL_REGION_FRAMES)
    LIVE_REGION_FRAMES = 3
    REC_REGION_START = (LIVE_REGION_START + MAX_FRAME_WORDS * LIVE_REGION_FRAMES)
    FPN_ADDRESS = CAL_REGION_START

    def __init__(self, sensor, onChange=None):
        self.sensor = sensor
        self.configFile = None
        self.dimmSize = [0, 0]

        # Setup internal defaults.
        self.__state = 'idle'
        self.__recMode = 'normal'
        self.__recMaxFrames = 0
        self.__recSegments = 1
        self.__recPreBurst = 1
        self.__exposureMode = 'normal'
        self.__exposurePeriod = self.sensor.getCurrentExposure()
        self.__tallyMode = 'auto'
        self.__wbCustom = [1.0, 1.0, 1.0]

        # Probe the SODIMMs
        for slot in range(0, len(self.dimmSize)):
            spdData = spd.spdRead(slot)
            if (spdData):
                self.dimmSize[slot] = spdData.size
        
        self.onChange = onChange
        self.description = "Chronos SN:%s" % (self.cameraSerial)
        self.idNumber = 0

        self.ioInterface = regmaps.ioInterface()
        
    
    def __propChange(self, name):
        """Quick and dirty wrapper to throw an on-change event by name"""
        try:
            self.onChange(name, getattr(self, name))
        except Exception as e:
            logging.debug("onChange handler failed: %s", e)
            pass
    
    def __setState(self, newState):
        """Internal helper to perform state transitions, or throw exceptions if busy"""
        if (self.__state == 'idle'):
            # Any transition from IDLE is valid.
            self.__state = newState
            self.__propChange('state')
        elif (newState == 'idle'):
            # Any transition to IDLE is valid.
            self.__state = newState
            self.__propChange('state')
        elif (newState == 'reset'):
            # A transition to RESET should abort any ongoing operations.
            self.__state = newState
            self.__propChange('state')
        else:
            # Otherwise, the state change can't happen because the camera is busy.
            raise CameraError("State change failed, camera is busy")
        
        # If the camera tally mode is 'auto' then also control the LED.
        if self.__tallyMode == 'auto':
            with open(REC_LED_FRONT, 'w') as fp:
                fp.write('1' if newState == 'recording' else '0')
            with open(REC_LED_BACK, 'w') as fp:
                fp.write('1' if newState == 'recording' else '0')
    
    def __checkState(self, *args):
        """Internal helper to check if the camera is in a valid state for the operation, or throw exceptions if busy"""
        if not self.__state in args:
            raise CameraError("Camera busy in state '%s'" % (self.__state))

    def setOnChange(self, handler):
        self.onChange = handler

    #===============================================================================================
    # API Methods: Reset Group
    #===============================================================================================
    def softReset(self, bitstream=None):
        """Reset the camera and initialize the FPGA and image sensor.
        
        Parameters
        ----------
        bitstream : str, optional
            File path to the FPGA bitstream to load, or None to perform
            only a soft-reset of the FPGA.
        
        Yields
        ------
        float :
            The sleep time, in seconds, between steps of the reset proceedure.
        
        Examples
        --------
        This function returns a generator iterator with the sleep time between steps
        of the reset proceedure. The caller can perform a complete reset as follows:

        state = camera.softReset()
        for delay in state:
            time.sleep(delay)
        """
        # If the current state is neither `idle` nor `recording`, then switch
        # to the `reset` state to force any outstanding generators to complete.
        # Give the generators up to half a second to finish and then continue
        # with the reset proceedure.
        if self.__state != 'idle' and self.__state != 'recording':
            self.__setState('reset')
            for x in range(0,5):
                if (self.__state == 'idle'):
                    break
                yield 0.1

        # Setup the FPGA if a bitstream was provided.
        if (bitstream):
            os.system("cam-loader %s" % (bitstream))
            config = regmaps.config()
            config.sysReset = 1
            yield 0.2
            logging.info("Loaded FPGA Version %s.%s", config.version, config.subver)
        else:
            config = regmaps.config()
            config.sysReset = 1
            yield 0.2
            logging.info("Detected FPGA Version %s.%s", config.version, config.subver)

        # Setup memory
        self.setupMemory()

        # Setup live display
        sensorRegs = regmaps.sensor()
        sensorRegs.fifoStart = 0x100
        sensorRegs.fifoStop = 0x100

        sequencerRegs = regmaps.sequencer()
        sequencerRegs.liveAddr[0] = self.LIVE_REGION_START + self.MAX_FRAME_WORDS * 0
        sequencerRegs.liveAddr[1] = self.LIVE_REGION_START + self.MAX_FRAME_WORDS * 1
        sequencerRegs.liveAddr[2] = self.LIVE_REGION_START + self.MAX_FRAME_WORDS * 2
        self.geometry = self.sensor.getMaxGeometry()
        self.geometry.vDarkRows = 0
        self.setupRecordRegion(self.geometry, self.REC_REGION_START)
        sequencerRegs.frameSize = self.geometry.size() // self.BYTES_PER_WORD
        
        # Enable video readout
        displayRegs = regmaps.display()
        displayRegs.control &= ~displayRegs.READOUT_INHIBIT

        # Load a default calibration
        colGainRegs = pychronos.fpgamap(pychronos.FPGA_COL_GAIN_BASE, 0x1000)
        colCurveRegs = pychronos.fpgamap(pychronos.FPGA_COL_CURVE_BASE, 0x1000)
        for x in range(0, self.geometry.hRes):
            colGainRegs.mem16[x] = (1 << 12)
            colCurveRegs.mem16[0] = 0

        # Reboot the sensor and return to the default resolution.
        self.sensor.reset()
        self.sensor.setResolution(self.geometry)
        self.wbMatrix = self.sensor.getWhiteBalance()
        self.colorMatrix = self.sensor.getColorMatrix()
        self.__setState('idle')

    def setupMemory(self):
        configRegs = regmaps.config()

        # We require at least one DIMM to be present.
        if ((self.dimmSize[0] + self.dimmSize[1]) == 0):
            raise RuntimeError("Memory configuration failed, no DIMMs detected")

        if (self.dimmSize[1] > self.dimmSize[0]):
            # Swap DIMMs to put the largest one first.
            configRegs.mmuConfig = configRegs.MMU_INVERT_CS
        elif (self.dimmSize[0] < (16 << 30)):
            # Stuff DIMMs together if less than 16GB
            configRegs.mmuConfig = configRegs.MMU_SWITCH_STUFFED

    # Reconfigure the recording region for a given frame size.
    def setupRecordRegion(self, fSize, startAddr, frameCount=0):
        ## Figure out the frame size, in words.
        fSizeWords = (fSize.size() + self.BYTES_PER_WORD - 1) // self.BYTES_PER_WORD
        fSizeWords //= self.FRAME_ALIGN_WORDS
        fSizeWords *= self.FRAME_ALIGN_WORDS

        seq = regmaps.sequencer()
        seq.frameSize = fSizeWords
        seq.regionStart = startAddr
        if (frameCount != 0):
            # Setup the desired number of frames if specified.
            seq.regionStop = startAddr + (frameCount * fSizeWords)
        else:
            # Otherwise, setup the maximum available memory.
            ramSizeWords = (self.dimmSize[0] + self.dimmSize[1]) // self.BYTES_PER_WORD
            seq.regionStop = (ramSizeWords // fSizeWords) * fSizeWords
    
    # Return the length of memory (in frames) minus calibration overhead.
    def getRecordingMaxFrames(self, fSize):
        ramSizeWords = (self.dimmSize[0] + self.dimmSize[1]) // self.BYTES_PER_WORD - self.REC_REGION_START
        fSizeWords = (fSize.size() + self.BYTES_PER_WORD - 1) // self.BYTES_PER_WORD
        fSizeWords //= self.FRAME_ALIGN_WORDS
        fSizeWords *= self.FRAME_ALIGN_WORDS
        return ramSizeWords // fSizeWords
    
    #===============================================================================================
    # API Methods: Recording Group
    #===============================================================================================
    def startCustomRecording(self, program):
        """Program the recording sequencer and start recording.

        This variant of startRecording takes a recording program as a list of
        `seqprogram` classes describing the steps for the recording sequencer
        to takes as frames are acquired from the image sensor.

        Parameters
        ----------
        program : `list` of `seqprogram`
            List of recording sequencer commands to execute for this recording.

        Yields
        ------
        float :
            The sleep time, in seconds, between steps of the recording.
        
        Examples
        --------
        This function returns a generator iterator with the sleep time between the
        steps of the recording proceedure. The caller may use this for cooperative
        multithreading, or can complete the calibration sychronously as follows:
        
        state = camera.startRecording()
        for delay in state:
            time.sleep(delay)
        """
        seq = regmaps.sequencer()
        # Setup the nextStates into a loop, just in case the caller forgot and then
        # load the program into the recording sequencer.
        for i in range(0, len(program)):
            program[i].nextState = (i + 1) % len(program)
            seq.program[i] = program[i]
        
        # Begin recording.
        self.__setState('recording')
        seq.control |= seq.START_REC
        seq.control &= ~seq.START_REC

        # Yield until recording is complete.
        yield 0.1
        while (seq.status & seq.ACTIVE_REC) != 0:
            yield 0.1
        self.__setState('idle')

    def startRecording(self, mode=None):
        """Program the recording sequencer and start recording.

        Parameters
        ----------
        mode : `string`, optional
            One of 'normal', 'segmented' or 'burst'.

        Yields
        ------
        float :
            The sleep time, in seconds, between steps of the recording.
        
        Examples
        --------
        This function returns a generator iterator with the sleep time between the
        steps of the recording proceedure. The caller may use this for cooperative
        multithreading, or can complete the calibration sychronously as follows:
        
        state = camera.startRecording()
        for delay in state:
            time.sleep(delay)
        """
        if not mode:
            mode = self.__recMode

        if mode == 'normal':
            # Record into a single segment until the trigger event.
            cmd = regmaps.seqcommand(blockSize=self.recMaxFrames,
                            blkTermFull=True, blkTermRising=True,
                            recTermMemory=True, recTermBlockEnd=True)
            yield from self.startCustomRecording([cmd])
        elif mode == 'segmented':
            # Record into segments 
            cmd = regmaps.seqcommand(blockSize=self.recMaxFrames / self.recSegments,
                            blkTermFull=True, blkTermRising=True,
                            recTermMemory=True, recTermBlockEnd=(self.recSegments > 1))
            yield from self.startCustomRecording([cmd])
        elif mode == 'burst':
            # When trigger is inactive, save the pre-record into a ring buffer.
            precmd = regmaps.seqcommand(blockSize=self.recPreBurst, blkTermHigh=True)
            # While trigger is active, save frames into the remaining memory.
            burstcmd = regmaps.seqcommand(blockSize=self.recMaxFrames - self.recPreBurst - 1,
                            blkTermLow=True, recTermMemory=True)
            yield from self.startCustomRecording([precmd, burstcmd])
        else:
            raise ValueError("recording mode of '%s' is not supported" % (mode))

    def softTrigger(self):
        """Signal a soft trigger event to the recording sequencer."""
        seq = regmaps.sequencer()
        seq.control |= seq.SW_TRIG
        seq.control &= ~seq.SW_TRIG

    def stopRecording(self):
        """Terminate a recording if one is in progress."""
        seq = regmaps.sequencer()
        seq.control |= seq.STOP_REC
        seq.control &= ~seq.STOP_REC

    #===============================================================================================
    # API Methods: Calibration Group
    #===============================================================================================
    def startWhiteBalance(self, hStart=None, vStart=None):
        """Begin the white balance proceedure

        Take a white reference sample from the live video stream, and compute the
        white balance coefficients for the current lighting conditions.

        Parameters
        ----------
        hStart : `int`, optional
            Horizontal position at which the white reference should be taken.
        vStart : `int`, optional
            Veritcal position at which the white reference should be taken.
        
        Yields
        ------
        float :
            The sleep time, in seconds, between steps of the white balance proceedure.
        
        Examples
        --------
        This function returns a generator iterator with the sleep time between the
        steps of the white balance proceedure. The caller may use this for cooperative
        multithreading, or can complete the calibration sychronously as follows:

        state = camera.startWhiteBalance()
        for delay in state:
            time.sleep(delay)
        """
        hSamples = 32
        vSamples = 32
        fSize = self.sensor.getCurrentGeometry()
        if (hStart is None):
            hStart = (fSize.hRes - hSamples) // 2
        if (vStart is None):
            vStart = (fSize.vRes - vSamples) // 2
        
        # Grab a frame from the live buffer.
        # TODO: We only really need to read out a subset of the frame, but
        # dealing with the word alignment is sucky.
        self.__setState('whitebal')
        seq = regmaps.sequencer()
        yield from seq.startLiveReadout(fSize.hRes, fSize.vRes)
        frame = numpy.asarray(seq.liveResult)

        # Apply calibration to the averaged frames.
        colGainRegs = pychronos.fpgamap(pychronos.FPGA_COL_GAIN_BASE, fSize.hRes * 2)
        colCurveRegs = pychronos.fpgamap(pychronos.FPGA_COL_CURVE_BASE, fSize.hRes * 2)
        colOffsetRegs = pychronos.fpgamap(pychronos.FPGA_COL_OFFSET_BASE, fSize.hRes * 2)
        gain = numpy.asarray(colGainRegs.mem16, dtype=numpy.uint16) / (1 << 12)
        curve = numpy.asarray(colCurveRegs.mem16, dtype=numpy.int16) / (1 << 21)
        offsets = numpy.asarray(colOffsetRegs.mem16, dtype=numpy.int16)
        corrected = (frame * frame * curve) + (frame * gain) + offsets
        # TODO: Subtract the per-pixel FPN, which a pain in the butt because it's a 12-bit signed.
        
        # Sum up each of the R, G and B channels
        rSum = 0
        gSum = 0
        bSum = 0
        for row in range(vStart, vStart+vSamples):
            for col in range(hStart, hStart+hSamples):
                if (row & 1):
                    # Odd Rows - Blue/Green pixels
                    if (col & 1):
                        gSum += corrected[row][col]
                    else:
                        bSum += corrected[row][col] * 2
                else:
                    # Even Rows - Green/Blue pixels
                    if (col & 1):
                        rSum += corrected[row][col] * 2
                    else:
                        gSum += corrected[row][col]
        
        # Find the highest channel (probably green)
        maxSum = max(rSum, gSum, bSum)
        whiteBalance = [maxSum / rSum, maxSum / gSum, maxSum / bSum]
        logging.info("Computed White Balance = %s", whiteBalance)

        # Load it into the display block for immediate use.
        displayRegs = regmaps.display()
        self.wbCustom = whiteBalance
        self.wbMatrix = whiteBalance
        self.__setState('idle')
    
    def __startBlackCal(self, numFrames=16, useLiveBuffer=True):
        # get the resolution from the display properties
        fSize = self.sensor.getCurrentGeometry()
        display = regmaps.display()
        seq = regmaps.sequencer()

        fAverage = numpy.zeros((fSize.vRes, fSize.hRes))
        if (useLiveBuffer):
            # Readout and average the frames from the live buffer.
            for i in range(0, numFrames):
                # Breakout in case of a soft reset.
                if self.__state == 'reset':
                    self.__setState('idle')
                    return
                logging.debug('waiting for frame %d of %d', i+1, numFrames)
                yield from seq.startLiveReadout(fSize.hRes, fSize.vRes)
                fAverage += numpy.asarray(seq.liveResult)
        else:
            # Take a recording and read the results from the live buffer.
            program = [regmaps.seqcommand(blockSize=numFrames+1, recTermBlkEnd=True, recTermBlkFull=True)]
            logging.debug('making recording')
            yield from seq.startCustomRecording([program])
            addr = seq.regionStart
            for i in range(0, numFrames):
                fAverage += numpy.asarray(pychronos.readframe(addr, fSize.hRes, fSize.vRes))
                addr += seq.frameSize
                yield 0

        fAverage /= numFrames
        
        # Readout the column gain and linearity calibration.
        colGainRegs = pychronos.fpgamap(pychronos.FPGA_COL_GAIN_BASE, fSize.hRes * 2)
        colCurveRegs = pychronos.fpgamap(pychronos.FPGA_COL_CURVE_BASE, fSize.hRes * 2)
        colOffsetRegs = pychronos.fpgamap(pychronos.FPGA_COL_OFFSET_BASE, fSize.hRes * 2)
        gain = numpy.asarray(colGainRegs.mem16, dtype=numpy.uint16) / (1 << 12)
        curve = numpy.asarray(colCurveRegs.mem16, dtype=numpy.int16) / (1 << 21)
        offsets = numpy.asarray(colOffsetRegs.mem16, dtype=numpy.int16)
        yield 0

        # For each column, the average gives the DC component of the FPN, which
        # gets applied to the column calibration as the constant term. The column
        # calibration function is given by:
        #
        # f(x) = curve * x^2 + gain * x + offset
        #
        # For the FPN to be black, we expected f(fpn) == 0, and therefore:
        #
        # offset = -(curve * fpn^2 + gain * fpn)
        colAverage = numpy.average(fAverage, 0)
        colOffset = curve * (colAverage * colAverage) + gain * colAverage
        # TODO: Would be nice to have a write helper.
        for x in range(0, fSize.hRes):
            colOffsetRegs.mem16[x] = int(-colOffset[x]) & 0xffff
        yield 0

        # For each pixel, the AC component of the FPN can be found by subtracting
        # the column average, which we will load into the per-pixel FPN region as
        # a signed quantity.
        #
        # TODO: For even better calibration, this should actually take the slope
        # into consideration around the FPN, in which case we would also divide
        # by the derivative of f'(fpn) for the column.
        fpn = numpy.int16(fAverage - colAverage)
        pychronos.writeframe(display.fpnAddr, fpn)

        logging.info('black calibration complete: min=%d, max=%d, deviation=%d',
                     numpy.min(fAverage), numpy.max(fAverage), numpy.std(fAverage))
        self.__setState('idle')

    def __startZeroTimeBlackCal(self):
        """Begin the black calibration proceedure using a zero-time exposure.

        Black calibration is best performed with the lens cap or shutter closed,
        but in the absence of user intervention, an acceptable calibration can be
        achieved by taking a zero-time exposure instead.

        Parameters
        ----------
        numFrames : int, optional
            The number of frames to use for black calibration (default 16 frames)
        useLiveBuffer : bool, optional
            Whether to use the live display for black calibration (default True)

        Yields
        ------
        float :
            The sleep time, in seconds, between steps of the calibration proceedure.
        
        Examples
        --------
        This function returns a generator iterator with the sleep time between the
        steps of the black calibration proceedure. The caller may use this for
        cooperative multithreading, or can complete the calibration sychronously
        as follows:

        state = camera.startZeroTimeBlackCal()
        for delay in state:
            time.sleep(delay)
        """
        # Grab the current frame size and exposure.
        fSize = self.sensor.getCurrentGeometry()
        expPrev = self.sensor.getCurrentExposure()
        expMin, expMax = self.sensor.getExposureRange(fSize, self.sensor.getCurrentPeriod())
        logging.info('fSize: %s', fSize)
        logging.info('expPrev: %f, zeroTime: %f, period: %f', expPrev, expMin, self.sensor.getCurrentPeriod())

        # Reconfigure for the minimum exposure supported by the sensor.
        self.sensor.setExposureProgram(expMin)

        # Do a fast black cal from the live display buffer.
        # TODO: We might get better quality out of the zero-time cal by
        # testing a bunch of exposure durations and finding the actual
        # zero-time intercept.
        yield (3 / 60) # ensure the exposure time has taken effect.
        yield from self.__startBlackCal(numFrames=2, useLiveBuffer=True)

        # Restore the previous exposure settings.
        self.__setupExposure(self.__exposurePeriod, self.__exposureMode)
    
    def startCalibration(self, blackCal=False, analogCal=False, zeroTimeBlackCal=False):
        """Begin one or more calibration proceedures at the current settings.

        Black calibration takes a sequence of images with the lens cap or shutter
        closed and averages them to find the black level of the image sensor. This
        value can then be subtracted during playback to correct for image offset
        defects.

        Analog calibration consists of any automated image sensor calibration
        that can be performed quickly and autonomously without any setup from
        the user (eg: no closing of the aperture or calibration jigs).

        Parameters
        ----------
        blackCal : bool, optional
            Perform a full black calibration assuming the user has closed the
            aperture or lens cap. (default: false)
        analogCal : bool, optional
            Perform autonomous analog calibration. (default: false)
        zeroTimeBlackCal : bool, optional
            Perform a fast black calibration by reducing the exposure time and
            aperture to their minimum values. (default: false)
        
        Yields
        ------
        float :
            The sleep time, in seconds, between steps of the calibration proceedure.
        
        Examples
        --------
        This function returns a generator iterator with the sleep time between steps
        of the calibration proceedures. The caller may use this for cooperative
        multithreading, or can complete the calibration sychronously as follows:

        state = camera.startCalibration(blackCal=True)
        for delay in state:
            time.sleep(delay)
        """
        # Perform autonomous sensor calibration first.
        if analogCal:
            logging.info('starting analog calibration')
            self.__setState('analogcal')
            yield from self.sensor.startAnalogCal()
            self.__setState('idle')

        # Perform at most one black calibration.
        if blackCal:
            self.__setState('blackcal')
            logging.info('starting standard black calibration')
            yield from self.__startBlackCal()
            self.__setState('idle')
        elif zeroTimeBlackCal:
            self.__setState('blackcal')
            logging.info('starting zero time black calibration')
            yield from self.__startZeroTimeBlackCal()
            self.__setState('idle')

    #===============================================================================================
    # API Parameters: Configuration Dictionary
    @camProperty()
    def config(self):
        """Return a configuration dictionary of all saveable parameters"""
        result = {}
        for name in dir(type(self)):
            prop = getattr(type(self), name, None)
            if (isinstance(prop, property) and getattr(prop.fget, 'saveable', False)):
                result[name] = getattr(self, name)
        return result

    #===============================================================================================
    # API Parameters: Camera Info Group
    @camProperty()
    def cameraApiVersion(self):
        return pychronos.__version__
    
    @camProperty()
    def cameraFpgaVersion(self):
        config = regmaps.config()
        return "%d.%d" % (config.version, config.subver)
    
    @camProperty()
    def cameraMemoryGB(self):
        return (self.dimmSize[0] + self.dimmSize[1]) / (1 << 30)
    
    @camProperty()
    def cameraModel(self):
        ## HACK: This needs to be updated from somewhere.
        return "CR14-1.0"
    
    @camProperty()
    def cameraSerial(self):
        I2C_SLAVE = 0x0703 # From linux/i2c-dev.h
        EEPROM_ADDR = 0x54 # From the C++ app

        # Open the I2C bus and set the EEPROM address.
        fd = os.open("/dev/i2c-1", os.O_RDWR)
        fcntl.ioctl(fd, I2C_SLAVE, EEPROM_ADDR)

        # Set readout offset and read the serial number.
        os.write(fd, bytearray([0, 0]))
        serial = os.read(fd, 12)
        os.close(fd)
        try:
            return serial.decode("utf-8").strip('\0')
        except:
            return ""
    
    @camProperty(notify=True, save=True)
    def cameraDescription(self):
        return self.description
    @cameraDescription.setter
    def cameraDescription(self, value):
        if not isinstance(value, str):
            raise TypeError("cameraDescription must be a string")
        self.description = value
        self.__propChange("cameraDescription")

    @camProperty(notify=True, save=True)
    def cameraIDNumber(self):
        return self.idNumber
    @cameraIDNumber.setter
    def cameraIDNumber(self, value):
        if not isinstance(value, int):
            raise TypeError("cameraIDNumber must be an integer got %s instead" % (type(value)))
        self.idNumber = value
        self.__propChange("cameraIDNumber")
    
    @camProperty(notify=True, save=True)
    def cameraTallyMode(self):
        return self.__tallyMode
    @cameraTallyMode.setter
    def cameraTallyMode(self, value):
        # Update the LEDs and tally state.
        if (value == 'on'):
            ledstate = '1'
        elif (value == 'off'):
            ledstate = '0'
        elif (value == 'auto'):
            ledstate = '1' if self.__state == 'recording' else '0'
        else:
            raise ValueError("cameraTallyMode value of '%s' is not supported" % (value))

        with open(REC_LED_FRONT, 'w') as fp:
            fp.write(ledstate)
        with open(REC_LED_BACK, 'w') as fp:
            fp.write(ledstate)

        self.__tallyMode = value
        self.__propChange('cameraTallyMode')
    
    #===============================================================================================
    # API Parameters: Sensor Info Group
    @camProperty()
    def sensorName(self):
        return self.sensor.name
    
    @camProperty()
    def sensorColorPattern(self):
        if self.sensor.cfaPattern:
            return self.sensor.cfaPattern
        else:
            return "mono"

    @camProperty()
    def sensorBitDepth(self):
        fSize = self.sensor.getMaxGeometry()
        return fSize.bitDepth
    
    @camProperty()
    def sensorPixelRate(self):
        fSize = self.sensor.getMaxGeometry()
        return (fSize.vRes + fSize.vDarkRows) * fSize.hRes / fSize.minFrameTime
    
    @camProperty()
    def sensorISO(self):
        return self.sensor.baseISO
    
    @camProperty()
    def sensorMaxGain(self):
        return self.sensor.maxGain
    
    @camProperty()
    def sensorVMax(self):
        fSize = self.sensor.getMaxGeometry()
        return fSize.vRes
    
    @camProperty()
    def sensorVMin(self):
        return self.sensor.vMin

    @camProperty()
    def sensorVIncrement(self):
        return self.sensor.vIncrement

    @camProperty()
    def sensorHMax(self):
        fSize = self.sensor.getMaxGeometry()
        return fSize.hRes
    
    @camProperty()
    def sensorHMin(self):
        return self.sensor.hMin

    @camProperty()
    def sensorHIncrement(self):
        return self.sensor.hIncrement

    @camProperty()
    def sensorVDark(self):
        fSize = self.sensor.getMaxGeometry()
        return fSize.vDarkRows
    
    #===============================================================================================
    # API Parameters: Exposure Group
    def __setupExposure(self, expPeriod, expMode):
        """Internal helper to setup the exposure time and mode."""
        if expMode == 'normal':
            self.sensor.setExposureProgram(expPeriod)
        elif expMode == 'frameTrigger':
            self.sensor.setFrameTriggerProgram(expPeriod)
        elif expMode == 'shutterGating':
            self.sensor.setShutterGatingProgram()
        else:
            raise NotImplementedError()
        
        self.__exposurePeriod = expPeriod
        self.__exposureMode = expMode

    @camProperty(notify=True, save=True, prio=PARAM_PRIO_EXPOSURE)
    def exposurePeriod(self):
        return int(self.__exposurePeriod * 1000000000)
    @exposurePeriod.setter
    def exposurePeriod(self, value):
        self.__checkState('idle', 'recording')
        self.__setupExposure(value / 1000000000, self.__exposureMode)
        self.__propChange("exposurePeriod")

    @camProperty(prio=PARAM_PRIO_EXPOSURE)
    def exposurePercent(self):
        fSize = self.sensor.getCurrentGeometry()
        fPeriod = self.sensor.getCurrentPeriod()
        expMin, expMax = self.sensor.getExposureRange(fSize, fPeriod)
        return (self.sensor.getCurrentExposure() - expMin) * 100 / (expMax - expMin)
    @exposurePercent.setter
    def exposurePercent(self, value):
        self.__checkState('idle', 'recording')
        fSize = self.sensor.getCurrentGeometry()
        fPeriod = self.sensor.getCurrentPeriod()
        expMin, expMax = self.sensor.getExposureRange(fSize, fPeriod)

        self.__setupExposure((value * (expMax - expMin) / 100) + expMin, self.__exposureMode)
        self.__propChange("exposurePeriod")

    @camProperty(prio=PARAM_PRIO_EXPOSURE)
    def shutterAngle(self):
        fPeriod = self.sensor.getCurrentPeriod()
        return self.sensor.getCurrentExposure() * 360 / fPeriod
    @shutterAngle.setter
    def shutterAngle(self, value):
        self.__checkState('idle', 'recording')
        fPeriod = self.sensor.getCurrentPeriod()

        self.__setupExposure(value * fPeriod / 360, self.__exposureMode)
        self.__propChange("exposurePeriod")

    @camProperty(notify=True)
    def exposureMin(self):
        fSize = self.sensor.getCurrentGeometry()
        fPeriod = self.sensor.getCurrentPeriod()
        expMin, expMax = self.sensor.getExposureRange(fSize, fPeriod)
        return int(expMin * 1000000000)
    
    @camProperty(notify=True)
    def exposureMax(self):
        fSize = self.sensor.getCurrentGeometry()
        fPeriod = self.sensor.getCurrentPeriod()
        expMin, expMax = self.sensor.getExposureRange(fSize, fPeriod)
        return int(expMax * 1000000000)
    
    @camProperty(notify=True, save=True)
    def exposureMode(self):
        """Selects the exposure timing program."""
        return self.__exposureMode
    @exposureMode.setter
    def exposureMode(self, value):
        self.__checkState('idle')
        if value not in self.sensor.getSupportedExposurePrograms():
            raise ValueError("exposureMode value of '%s' is not supported" % (value))
        
        self.__setupExposure(self.__exposurePeriod, value)
        self.__propChange("exposureMode")

    #===============================================================================================
    # API Parameters: Gain Group
    @camProperty(notify=True)
    def currentGain(self):
        return self.sensor.getCurrentGain()
    
    @camProperty()
    def currentISO(self):
        return self.sensor.getCurrentGain() * self.sensor.baseISO

    #===============================================================================================
    # API Parameters: Camera Status Group
    @camProperty(notify=True)
    def state(self):
        return self.__state
    
    @camProperty()
    def dateTime():
        return datetime.datetime.now().isoformat()
    
    @camProperty()
    def externalStorage(self):
        return utils.getStorageDevices()

    #===============================================================================================
    # API Parameters: Recording Group
    @camProperty(notify=True, save=True)
    def recMode(self):
        return self.__recMode
    @recMode.setter
    def recMode(self, value):
        self.__checkState('idle')
        if value not in ('normal', 'segmented', 'burst'):
            raise ValueError("recMode value of '%s' is not supported" % (value))
        self.__recMode = value
        self.__propChange("recMode")

    @camProperty(notify=True, save=True)
    def recMaxFrames(self):
        """Maximum number of frames to use in the recording buffer."""
        currentMaxFrames = self.cameraMaxFrames
        if not self.__recMaxFrames:
            return currentMaxFrames
        elif self.__recMaxFrames > currentMaxFrames:
            return currentMaxFrames
        else:
            return self.__recMaxFrames
    @recMaxFrames.setter
    def recMaxFrames(self, value):
        self.__checkState('idle')
        currentMaxFrames = self.cameraMaxFrames
        if value < currentMaxFrames:
            self.__recMaxFrames = value
        else:
            self.__recMaxFrames = 0 # We use an internal value of zero to mean infinity.
        self.__propChange("recMaxFrames")

    @camProperty(notify=True, save=True)
    def recSegments(self):
        return self.__recSegments
    @recSegments.setter
    def recSegments(self, value):
        if not isinstance(value, int):
            raise TypeError("recSegments must be an integer")
        if value < 1:
            raise ValueError("recSegments must be greater than zero")
        self.__checkState('idle')
        self.__recSegments = value
        self.__propChange("recSegments")
    
    @camProperty(notify=True, save=True)
    def recPreBurst(self):
        return self.__recPreBurst
    @recPreBurst.setter
    def recPreBurst(self, value):
        if not isinstance(value, int):
            raise TypeError("recPreBurst must be an integer")
        if value < 1:
            raise ValueError("recPreBurst must be greater than zero")
        self.__checkState('idle')
        self.__recSegments = value
        self.__propChange("recPreBurst")

    @camProperty(notify=True)
    def cameraMaxFrames(self):
        """Maximum number of frames the camera's memory can save at the current resolution."""
        fSize = self.sensor.getCurrentGeometry()
        return self.getRecordingMaxFrames(fSize)
    
    @camProperty(notify=True, save=True, prio=PARAM_PRIO_RESOLUTION)
    def resolution(self):
        """Dictionary describing the current resolution settings."""
        fSize = self.sensor.getCurrentGeometry()
        return {
            "hRes": fSize.hRes,
            "vRes": fSize.vRes,
            "hOffset": fSize.hOffset,
            "vOffset": fSize.vOffset,
            "vDarkRows": fSize.vDarkRows,
            "bitDepth": fSize.bitDepth,
            "minFrameTime": fSize.minFrameTime
        }
    @resolution.setter
    def resolution(self, value):
        self.__checkState('idle')
        geometry = pychronos.sensors.frameGeometry(**value)
        self.sensor.setResolution(geometry)
        self.setupRecordRegion(geometry, self.REC_REGION_START)
        self.__propChange("resolution")
        # TODO: Should a change of resolution revert exposureMode back to normal?
        self.__exposurePeriod = self.sensor.getCurrentExposure()
        # Changing resolution affects frame timing.
        self.__propChange("minFramePeriod")
        self.__propChange("framePeriod")
        self.__propChange("exposureMin")
        self.__propChange("exposureMax")
        self.__propChange("exposurePeriod")
        # Changing resolution affects recording length.
        self.__propChange("cameraMaxFrames")
        self.__propChange("recMaxFrames")
    
    @camProperty(notify=True)
    def minFramePeriod(self):
        """Minimum frame period at the current resolution settings."""
        fSize = self.sensor.getCurrentGeometry()
        fpMin, fpMax = self.sensor.getPeriodRange(fSize)
        return int(fpMin * 1000000000)

    @camProperty(notify=True, save=True, prio=PARAM_PRIO_FRAME_TIME)
    def framePeriod(self):
        """Time in nanoseconds to record a single frame (or minimum time for frame sync and shutter gating)."""
        return int(self.sensor.getCurrentPeriod() * 1000000000)
    @framePeriod.setter
    def framePeriod(self, value):
        self.__checkState('idle')
        self.sensor.setFramePeriod(value / 1000000000)
        self.__propChange("framePeriod")
        # Changing frame period affects exposure timing.
        self.__propChange("exposureMin")
        self.__propChange("exposureMax")
        self.__propChange("exposurePeriod")

    @camProperty(prio=PARAM_PRIO_FRAME_TIME)
    def frameRate(self):
        """Estimated recording frame rate in frames per second (reciprocal of framePeriod)."""
        return 1 / self.sensor.getCurrentPeriod()
    @frameRate.setter
    def frameRate(self, value):
        self.__checkState('idle')
        self.sensor.setFramePeriod(1 / value)
        self.__propChange("framePeriod")
        # Changing frame period affects exposure timing.
        self.__propChange("exposureMin")
        self.__propChange("exposureMax")
        self.__propChange("exposurePeriod")
    
    #===============================================================================================
    # API Parameters: Color Space Group
    @camProperty(notify=True, save=True)
    def wbMatrix(self):
        """Array of Red, Green, and Blue gain coefficients to achieve white balance."""
        display = regmaps.display()
        return [
            display.whiteBalance[0] / display.WHITE_BALANCE_DIV,
            display.whiteBalance[1] / display.WHITE_BALANCE_DIV,
            display.whiteBalance[2] / display.WHITE_BALANCE_DIV
        ]
    @wbMatrix.setter
    def wbMatrix(self, value):
        display = regmaps.display()
        display.whiteBalance[0] = int(value[0] * display.WHITE_BALANCE_DIV)
        display.whiteBalance[1] = int(value[1] * display.WHITE_BALANCE_DIV)
        display.whiteBalance[2] = int(value[2] * display.WHITE_BALANCE_DIV)
        self.__propChange("wbMatrix")
    
    @camProperty(notify=True, save=True)
    def wbCustom(self):
        """Array of Red, Green, and Blue gain coefficients to achieve white balance."""
        return self.__wbCustom
    @wbCustom.setter
    def wbCustom(self, value):
        self.__wbCustom[0] = value[0]
        self.__wbCustom[1] = value[1]
        self.__wbCustom[2] = value[2]
        self.__propChange("wbCustom")

    @camProperty(notify=True, save=True)
    def colorMatrix(self):
        """Array of 9 floats describing the 3x3 color matrix from image sensor color space in to sRGB, stored in row-scan order."""
        display = regmaps.display()
        return [
            display.colorMatrix[0] / display.COLOR_MATRIX_DIV,
            display.colorMatrix[1] / display.COLOR_MATRIX_DIV,
            display.colorMatrix[2] / display.COLOR_MATRIX_DIV,
            display.colorMatrix[3] / display.COLOR_MATRIX_DIV,
            display.colorMatrix[4] / display.COLOR_MATRIX_DIV,
            display.colorMatrix[5] / display.COLOR_MATRIX_DIV,
            display.colorMatrix[6] / display.COLOR_MATRIX_DIV,
            display.colorMatrix[7] / display.COLOR_MATRIX_DIV,
            display.colorMatrix[8] / display.COLOR_MATRIX_DIV
        ]
    @colorMatrix.setter
    def colorMatrix(self, value):
        display = regmaps.display()
        display.colorMatrix[0] = int(value[0] * display.COLOR_MATRIX_DIV)
        display.colorMatrix[1] = int(value[1] * display.COLOR_MATRIX_DIV)
        display.colorMatrix[2] = int(value[2] * display.COLOR_MATRIX_DIV)
        display.colorMatrix[3] = int(value[3] * display.COLOR_MATRIX_DIV)
        display.colorMatrix[4] = int(value[4] * display.COLOR_MATRIX_DIV)
        display.colorMatrix[5] = int(value[5] * display.COLOR_MATRIX_DIV)
        display.colorMatrix[6] = int(value[6] * display.COLOR_MATRIX_DIV)
        display.colorMatrix[7] = int(value[7] * display.COLOR_MATRIX_DIV)
        display.colorMatrix[8] = int(value[8] * display.COLOR_MATRIX_DIV)
        self.__propChange("colorMatrix")

    #===============================================================================================
    # API Parameters: IO Configuration Group
    @camProperty(notify=True, save=True)
    def ioMapping(self):
        """Configuration for the IO block - this is a dictionary of IO components and what their inputs are configured to"""
        return self.ioInterface.getConfiguration()
    @ioMapping.setter
    def ioMapping(self, value):
        self.ioInterface.setConfiguration(value)

    @camProperty()
    def ioDelayTime(self):
        """Property alias of the ioMapping.delay.delayTime value"""
        return self.ioInterface.delayTime
    @ioDelayTime.setter
    def ioDelayTime(self, value):
        self.ioInterface.delayTime = value


    def __camIoProperty(propname, docstring, readOnly=True, notify=False, save=False, prio=0):
        if readOnly:
            prop = property(fget=lambda self: type(self.ioInterface).__dict__[propname].__get__(self.ioInterface, type(self.ioInterface)),
                            doc=docstring)
        else:
            prop = property(fget=lambda self: type(self.ioInterface).__dict__[propname].__get__(self.ioInterface, type(self.ioInterface)),
                            fset=lambda self, value: type(self.ioInterface).__dict__[propname].__set__(self.ioInterface, value),
                            doc=docstring)
        setattr(prop.fget, 'notifies', notify)
        setattr(prop.fget, 'saveable', save)
        setattr(prop.fget, 'prio', prio)
        return prop

    def __camIoMapping(ioname, docstring, notify=True, save=True, prio=0):
        prop = property(fget=lambda self: self.ioInterface.getSourceConfiguration(ioname),
                        fset=lambda self, config: self.ioInterface.setSourceConfiguration(ioname, config),
                        doc=docstring)
        setattr(prop.fget, 'notifies', notify)
        setattr(prop.fget, 'saveable', save)
        setattr(prop.fget, 'prio', prio)
        return prop

    def __camIoInputConfig(ioname, docstring, notify=True, save=True, prio=0):
        prop = property(fget=lambda self: self.ioInterface.getInputConfiguration(ioname),
                        fset=lambda self, config: self.ioInterface.setInputConfiguration(ioname, config),
                        doc=docstring)
        setattr(prop.fget, 'notifies', notify)
        setattr(prop.fget, 'saveable', save)
        setattr(prop.fget, 'prio', prio)
        return prop
    
    ioStatusSourceIo1 = __camIoProperty('sourceIo1', 'input 1 current value', readOnly=True)
    ioStatusSourceIo2 = __camIoProperty('sourceIo2', 'input 2 current value', readOnly=True)
    ioStatusSourceIo3 = __camIoProperty('sourceIo3', 'input 3 current value', readOnly=True)

    ioMappingIo1         = __camIoMapping('io1',         'output driver 1 configuration')
    ioMappingIo2         = __camIoMapping('io2',         'output driver 2 configuration')
    ioMappingCombOr1     = __camIoMapping('combOr1',     'combinatorial block OR input 1 (out = ((Or1 | Or2 | Or3) ^ XOr) & And)')
    ioMappingCombOr2     = __camIoMapping('combOr2',     'combinatorial block OR input 2 (out = ((Or1 | Or2 | Or3) ^ XOr) & And)')
    ioMappingCombOr3     = __camIoMapping('combOr3',     'combinatorial block OR input 3 (out = ((Or1 | Or2 | Or3) ^ XOr) & And)')
    ioMappingCombXOr     = __camIoMapping('combXOr',     'combinatorial block XOR input (out = ((Or1 | Or2 | Or3) ^ XOr) & And)')
    ioMappingCombAnd     = __camIoMapping('combAnd',     'combinatorial block AND input (out = ((Or1 | Or2 | Or3) ^ XOr) & And)')
    ioMappingDelay       = __camIoMapping('delay',       'delay block configuration')
    ioMappingToggleSet   = __camIoMapping('toggleSet',   'Toggle block Set configuration (out = True on rising edge of input)')
    ioMappingToggleClear = __camIoMapping('toggleClear', 'Toggle block configuration (out = False on rising edge of input)')
    ioMappingToggleFlip  = __camIoMapping('toggleFlip',  'Toggle block configuration (out = not out on rising edge of input)')
    ioMappingShutter     = __camIoMapping('shutter',     'Shutter (signal to timing block) configuration')
    ioMappingStartRec    = __camIoMapping('start',       'Start recording (signal to record sequencer) configuration')
    ioMappingStartRec    = __camIoMapping('stop',        'Stop or end recording (signal to record sequencer) configuration')

    ioInputConfigIo1     = __camIoInputConfig('io1In', 'Input 1 config such as threshhold')
    ioInputConfigIo2     = __camIoInputConfig('io2In', 'Input 2 config such as threshhold')

    @camProperty(notify=False, save=False)
    def ioDetailedStatus(self):
        return self.ioInterface.getIoStatus()
    
    
    
