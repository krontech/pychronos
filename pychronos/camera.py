# Chronos High-Speed Camera class
import fcntl
import os
import numpy
import logging
import datetime
from enum import Enum

import pychronos
import pychronos.regmaps as regmaps
import pychronos.spd as spd
from pychronos.io import io
from pychronos.power import power
from pychronos.error import *
from pychronos.types import *

from . import utils
from . import props
from .props import camProperty as camProperty

# Recording LEDs
REC_LED_FRONT = "/sys/class/gpio/gpio41/value"
REC_LED_BACK = "/sys/class/gpio/gpio25/value"
BACKLIGHT_PIN = "/sys/class/gpio/gpio18/value"

# Wrap a property that is inherited form a nested class.
def propWrapper(membername, propname, propclass):
    setter = None if not propclass.fset else lambda self, value: setattr(getattr(self, membername), propname, value)
    wrapper = property(fget = lambda self: getattr(getattr(self, membername), propname),
                       fset = setter, doc=getattr(propclass, '__doc__'))
    # Duplicate the camera property metadata.
    wrapper.fget.notifies = getattr(propclass.fget, 'notifies', False)
    wrapper.fget.saveable = getattr(propclass.fget, 'saveable', False)
    wrapper.fget.derivedFrom = getattr(propclass.fget, 'derivedFrom', None)
    wrapper.fget.prio = getattr(propclass.fget, 'prio', 0)
    return wrapper

# Duplicate all properties from a member.
def propClassInherit(xclass, membername):
    children = {}
    for propname in dir(xclass):
        xprop = getattr(xclass, propname)
        if (isinstance(xprop, property)):
            children[propname] = propWrapper(membername, propname, xprop)
    return children

class camera:
    BYTES_PER_WORD = 32
    FRAME_ALIGN_WORDS = 64
    
    # Inherit properties from nested classes.
    vars().update(propClassInherit(io, 'io'))
    vars().update(propClassInherit(power, 'power'))

    def __init__(self, sensor, onChange=None):
        self.sensor = sensor
        self.configFile = None
        self.dimmSize = [0, 0]
        self.power = power(onChange=lambda n, v: self.onChange(n, v) if self.onChange else None)
        self.io = io(onChange=lambda n, v: self.onChange(n, v) if self.onChange else None)
        
        # Setup internal defaults.
        self.__state = 'idle'
        self.__recMode = RecModes.normal
        self.__recMaxFrames = 0
        self.__recSegments = 1
        self.__recPreBurst = 1
        self.__recTrigDelay = 0
        self.__exposureMode = ExposureModes.normal
        self.__exposurePeriod = self.sensor.getCurrentExposure()
        self.__tallyMode = TallyModes.auto
        self.__wbTemperature = 5500
        self.__wbCustomColor = self.getWhiteBalance(self.__wbTemperature)
        self.__miscScratchPad = {}
        self.__disableRingBuffer = False
        self.__calSuggested = False

        self.__ioRecord = False

        # Setup the reserved video memory.
        self.MAX_FRAME_WORDS = self.getFrameSizeWords(sensor.getMaxGeometry())
        self.CAL_REGION_START = 0
        self.CAL_REGION_FRAMES = 3
        self.LIVE_REGION_START = (self.CAL_REGION_START + self.MAX_FRAME_WORDS * self.CAL_REGION_FRAMES)
        self.LIVE_REGION_FRAMES = 3
        self.REC_REGION_START = (self.LIVE_REGION_START + self.MAX_FRAME_WORDS * self.LIVE_REGION_FRAMES)
        self.FPN_ADDRESS = self.CAL_REGION_START

        # Probe the SODIMMs
        for slot in range(0, len(self.dimmSize)):
            spdData = spd.spdRead(slot)
            if (spdData):
                self.dimmSize[slot] = spdData.size

        # Probe for the sensor's temperature sensor.
        self.__sensorTemperatureAddr = 0
        for x in range(0x48, 0x50):
            try:
                result = utils.smbusRead(x, 1)
                self.__sensorTemperatureAddr = x
                break
            except Exception as err:
                pass
        
        self.onChange = onChange
        self.description = "Chronos SN:%s" % (self.cameraSerial)
        self.idNumber = 0

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
        if self.__tallyMode == TallyModes.auto:
            with open(REC_LED_FRONT, 'w') as fp:
                fp.write('1' if newState == 'recording' else '0')
            with open(REC_LED_BACK, 'w') as fp:
                fp.write('1' if newState == 'recording' else '0')
    
    def __checkState(self, *args):
        """Internal helper to check if the camera is in a valid state for the operation, or throw exceptions if busy"""
        if not self.__state in args:
            raise CameraError("Camera busy in state '%s'" % (self.__state))


    def setOnChange(self, handler):
        """Install an on-change handler to be called whenever properties are modified.

        To report changes in state or configuration, properties with the `notify` attribute
        set will invoke a callback handler with the name of the property and its new value.

        Args:
            handler (callable): Callback method to invoke whenever a property is changed.
        """
        self.onChange = handler

    def tick(self):
        """Perfom background processing for the camera."""
        self.power.checkPowerSocket()

        seqRegs = regmaps.sequencer()

        if (seqRegs.status & seqRegs.ACTIVE_REC) > 0: ## check if the FPGA is recording
            if (self.__state == 'idle'): ## are we set to 'idle' while FPGA is recording?
                logging.info("State should be RECORDING (state stuck as idle); changing state to recording")
                self.__setState('recording')
                self.__ioRecord = True

        else:
            if (self.__state == 'recording') and self.__ioRecord: ## are we set to 'recording' while FPGA is idle?
                logging.info(">> State should be IDLE (state stuck as recording); changing state to idle")
                self.__setState('idle')
                self.__ioRecord = False



    #===============================================================================================
    # API Methods: Reset Group
    #===============================================================================================
    def abort(self):
        """Terminate anything that the camera might be doing and return to the
        idle state. This would typically be used as part of a reset procedure.
        """
        # If the current state is `recording` then simply stop the recording and
        # wait for the state to become idle.
        if (self.__state == 'recording'):
            self.stopRecording()

        # Otherwise, if the state is not `idle`, then switch to the `reset` state
        # to force any generates to complete. Give the generates up to half a second
        # fo finish.
        if (self.__state != 'idle'):
            self.__setState('reset')
            for x in range(0,5):
                if (self.__state == 'idle'):
                    break
                yield 0.1

    def softReset(self, bitstream=None):
        """Reset the camera and initialize the FPGA and image sensor.

        Args:
            bitstream (str, optional): File path to the FPGA bitstream to load,
                or None to perform only a soft-reset of the FPGA.

        Yields:
            float: The sleep time, in seconds, between steps of the reset procedure.

        Examples:
            This function returns a generator iterator with the sleep time between steps
            of the reset procedure. The caller can perform a complete reset as follows:

            >>> state = camera.softReset()
            >>> for delay in state:
            >>>    time.sleep(delay)
        """
        self.abort()

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

        self.stopRecording()
        
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
        self.wbColor = self.getWhiteBalance()
        self.colorMatrix = self.sensor.getColorMatrix()

        # Reboot any other modules we included.
        self.io.doReset()

        self.__recTrigDelay = 0 ## force the trigger delay to zero on reset

        # Soft reboot completed
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
        fSizeWords = self.getFrameSizeWords(fSize)
        ramSizeWords = (self.dimmSize[0] + self.dimmSize[1]) // self.BYTES_PER_WORD
        maxFrames = ramSizeWords // fSizeWords

        seq = regmaps.sequencer()
        seq.frameSize = fSizeWords
        seq.regionStart = startAddr
        if (frameCount > maxFrames) or (frameCount == 0):
            # Setup the maximum available memory.
            seq.regionStop = (ramSizeWords // fSizeWords) * fSizeWords
            self.__recMaxFrames = 0
        else:
            # Setup the desired number of frames if specified.
            self.__recMaxFrames = frameCount
            seq.regionStop = startAddr + (frameCount * fSizeWords)
        
    
    # Return the size of a frame in memory words.
    def getFrameSizeWords(self, fSize):
        fSizeWords = (fSize.size() + self.BYTES_PER_WORD - 1) // self.BYTES_PER_WORD
        fSizeWords += self.FRAME_ALIGN_WORDS - 1
        fSizeWords //= self.FRAME_ALIGN_WORDS
        fSizeWords *= self.FRAME_ALIGN_WORDS
        return fSizeWords
    
    # Return the length of memory (in frames) minus calibration overhead.
    def getRecordingMaxFrames(self, fSize):
        ramSizeWords = (self.dimmSize[0] + self.dimmSize[1]) // self.BYTES_PER_WORD - self.REC_REGION_START
        return ramSizeWords // self.getFrameSizeWords(fSize)
    
    # Compute the white balance for a given color temperature.
    def getWhiteBalance(self, cTempK=5500):
        if (len(self.sensor.wbPresets) == 0):
            # No white balance presets exists, probably monochrome.
            return [1.0, 1.0, 1.0]
        elif (cTempK in self.sensor.wbPresets):
            # A white balance preset exists for this color temperature.
            return self.sensor.wbPresets[cTempK]
        else:
            # We Must interpolate the white balance matrices.
            cTempList = list(self.sensor.wbPresets.keys())
            cTempList.sort()

            # Find the highest preset below the target.
            cTempLow = cTempList[0]
            cTempHi = cTempList[-1]
            if (cTempLow > cTempK):
                return self.sensor.wbPresets[cTempLow]
            if (cTempHi < cTempK):
                return self.sensor.wbPresets[cTempHi]
            for k in cTempList:
                if (k == cTempK):
                    return self.sensor.wbPresets[cTempK]
                elif (k < cTempK):
                    cTempLow = k
                elif (k > cTempK):
                    cTempHi = k
                    break
            
            # Interpolate!
            wbHigh = self.sensor.wbPresets[cTempHi]
            wbLow = self.sensor.wbPresets[cTempLow]
            return [
                (wbHigh[0] * (cTempK - cTempLow) + wbLow[0] * (cTempHi - cTempK)) / (cTempHi - cTempLow),
                (wbHigh[1] * (cTempK - cTempLow) + wbLow[1] * (cTempHi - cTempK)) / (cTempHi - cTempLow),
                (wbHigh[2] * (cTempK - cTempLow) + wbLow[1] * (cTempHi - cTempK)) / (cTempHi - cTempLow),
            ]
    
    #===============================================================================================
    # API Methods: Recording Group
    #===============================================================================================
    def startCustomRecording(self, program):
        """Programs the recording sequencer and starts recording.

        This variant of startRecording takes a recording program as a list of
        `seqprogram` classes describing the steps for the recording sequencer
        to takes as frames are acquired from the image sensor.

        Args:
            program (:obj:`list` of :obj:`seqprogram`): List of recording sequencer
                commands to executed for this recording.

        Yields:
            float: The sleep time, in seconds, between steps of the recording program.
        
        Example:
            This function returns a generator iterator with the sleep time between the
            steps of the recording procedure. The caller may use this for cooperative
            multithreading, or can complete the calibration sychronously as follows:
        
            >>> state = camera.startRecording()
            >>> for delay in state:
            >>>    time.sleep(delay)
        """
        seq = regmaps.sequencer()
        # Setup the nextStates into a loop, just in case the caller forgot and then
        # load the program into the recording sequencer.
        for i in range(0, len(program)):
            program[i].nextState = (i + 1) % len(program)
            seq.program[i] = program[i]
        
        # Begin recording.
        self.__setState('recording')
        seq.trigDelay = self.__recTrigDelay
        seq.control |= seq.START_REC
        seq.control &= ~seq.START_REC

        # Yield until recording is complete.
        yield 0.1
        while (seq.status & seq.ACTIVE_REC) != 0:
            yield 0.1
        self.__setState('idle')

    def startRecording(self, recMode=None):
        """Program the recording sequencer and start recording.

        Args:
            recMode (RecModes, optional): Override the current `recMode` property when
                starting the recording.

        Yields:
            float: The sleep time, in seconds, between steps of the recording.
        
        Example:
            This function returns a generator iterator with the sleep time between the
            steps of the recording procedure. The caller may use this for cooperative
            multithreading, or can complete the calibration sychronously as follows:
            
            >>> state = camera.startRecording()
            >>> for delay in state:
            >>>     time.sleep(delay)
        """
        if not recMode:
            mode = self.__recMode
        else:
            mode = RecModes[recMode]

        if mode == RecModes.normal:
            # Record into a single segment until the trigger event.
            cmd = regmaps.seqcommand(blockSize=self.recMaxFrames,
                            blkTermFull=False, blkTermRising=True,
                            recTermMemory=False, recTermBlockEnd=True)
            yield from self.startCustomRecording([cmd])
        elif mode == RecModes.segmented:
            # Record into segments 
            cmd = regmaps.seqcommand(blockSize=self.recMaxFrames // self.recSegments,
                            blkTermFull=False, blkTermRising=True,
                            recTermMemory=self.__disableRingBuffer, recTermBlockEnd=False)
            yield from self.startCustomRecording([cmd])
        elif mode == RecModes.burst:
            # When trigger is inactive, save the pre-record into a ring buffer.
            precmd = regmaps.seqcommand(blockSize=self.recPreBurst, blkTermRising=True)
            # While trigger is active, save frames into the remaining memory.
            burstcmd = regmaps.seqcommand(blockSize=self.recMaxFrames - self.recPreBurst - 1,
                            blkTermFalling=True, recTermMemory=False)
            yield from self.startCustomRecording([precmd, burstcmd])
        else:
            raise ValueError("recording mode of '%s' is not supported" % str(mode))

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
        """Begin the white balance procedure.

        Take a white reference sample from the live video stream, and compute the
        white balance coefficients for the current lighting conditions. If successful,
        the results of the white balance calculation will be stored in `wbCustomColor`
        and `wbTemperature` will be set to 0K.

        Args:
            hStart (int, optional): Horizontal position at which the white reference should be taken.
            vStart (int, optional): Veritcal position at which the white reference should be taken.
        
        Yields:
            float: The sleep time, in seconds, between steps of the white balance procedure.
        
        Raises:
            CalibrationError: If frames could not be acquired from the video stream.
            SignalClippingError: Pixels at the target area of the sensor are clipping and cannot
                be used for a reliable measure of the white point.
            LowSignalError: The measured value is too dim to produce an accurate white point.
        
        Example:
            This function returns a generator iterator with the sleep time between the
            steps of the white balance procedure. The caller may use this for cooperative
            multithreading, or can complete the calibration sychronously as follows:

            >>> state = camera.startWhiteBalance()
            >>> for delay in state:
            >>>     time.sleep(delay)
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
        if not seq.liveResult:
            self.__setState('idle')
            raise CalibrationError("Failed to acquire frames during calibration")
        frame = numpy.asarray(seq.liveResult)

        # Apply calibration to the averaged frames.
        display = regmaps.display()
        colGainRegs = pychronos.fpgamap(pychronos.FPGA_COL_GAIN_BASE, fSize.hRes * 2)
        gain = numpy.asarray(colGainRegs.mem16, dtype=numpy.uint16) / (1 << 12)
        fpn = numpy.asarray(pychronos.readframe(display.fpnAddr, fSize.hRes, fSize.vRes))
        
        # Sum up each of the R, G and B channels
        headroom = (1 << fSize.bitDepth) // 32
        minSum = headroom * (hSamples * vSamples) // 2
        rSum = 0
        gSum = 0
        bSum = 0
        for row in range(vStart, vStart+vSamples):
            for col in range(hStart, hStart+hSamples):
                black = fpn[row][col]
                pix = frame[row][col]
                if (pix > black):
                    pix -= black
                    pix *= gain[col]
                else:
                    pix = 0

                if ((pix + headroom) > (1 << fSize.bitDepth)):
                    self.__setState('idle')
                    raise SignalClippingError("Signal clipping, reference image is too bright for white balance")

                if (row & 1):
                    # Odd Rows - Blue/Green pixels
                    if (col & 1):
                        gSum += pix
                    else:
                        bSum += pix * 2
                else:
                    # Even Rows - Green/Blue pixels
                    if (col & 1):
                        rSum += pix * 2
                    else:
                        gSum += pix
        
        # Check for too-low of a signal.
        if ((rSum < minSum) or (gSum < minSum) or (bSum < minSum)):
            self.__setState('idle')
            raise LowSignalError("Low signal, reference image is too dim for white balance")

        # Find the highest channel (probably green)
        maxSum = max(rSum, gSum, bSum)
        whiteBalance = [maxSum / rSum, maxSum / gSum, maxSum / bSum]
        logging.info("Computed White Balance = %s", whiteBalance)

        # Load it into the display block for immediate use.
        displayRegs = regmaps.display()
        self.__wbCustomColor = whiteBalance
        self.wbTemperature = 0 # Select custom white balance.
        self.__setState('idle')
    
    def __applyBlackCal(self, fSize, fAverage):
        display = regmaps.display()
        pychronos.writeframe(display.fpnAddr, fAverage)
        logging.info('loaded black calibration: min=%d, max=%d, deviation=%d',
                     numpy.min(fAverage), numpy.max(fAverage), numpy.std(fAverage))

    def __loadBlackCal(self, calLocation, factoryLocation=None):
        display = regmaps.display()
        fSize = self.sensor.getCurrentGeometry()
        suffix = self.sensor.calFilename("/fpn_%dx%doff%dx%d" % (fSize.hRes, fSize.vRes, fSize.hOffset, fSize.vOffset), '.raw')

        # Load any sensor black calibration data.
        if not self.sensor.loadBlackCal(calLocation, factoryLocation):
            logging.info("Sensor black calibration data not found")
            return False

        # Prefer user calibration over factory calibration, if possible.
        filename = calLocation + suffix
        if (factoryLocation and not os.path.isfile(calLocation + suffix)):
            filename = factoryLocation + suffix
        
        # Load calibration!
        try:
            logging.info("Loading black calibration from %s", filename)
            fpnData = numpy.reshape(numpy.fromfile(filename, dtype=numpy.uint16), (fSize.vRes, fSize.hRes))
            self.__applyBlackCal(fSize, fpnData)
            return True
        except Exception as err:
            logging.info("Clearing black calibration data")
            self.__applyBlackCal(fSize, numpy.zeros((fSize.vRes, fSize.hRes), dtype=numpy.uint16))
            return False
    
    def __startBlackCal(self, numFrames=16, useLiveBuffer=True, saveLocation=None):
        # get the resolution from the display properties
        fSize = self.sensor.getCurrentGeometry()
        seq = regmaps.sequencer()
        display = regmaps.display()

        # Perform any sensor black calibration routines.
        logging.info('starting sensor black calibration')
        yield from self.sensor.startBlackCal(saveLocation)

        fAverage = numpy.zeros((fSize.vRes, fSize.hRes), dtype=numpy.uint16)
        if (useLiveBuffer):
            # Readout anfd average the frames from the live buffer.
            for i in range(0, numFrames):
                # Breakout in case of a soft reset.
                if self.__state == 'reset':
                    self.__setState('idle')
                    return
                logging.debug('waiting for frame %d of %d', i+1, numFrames)
                yield from seq.startLiveReadout(fSize.hRes, fSize.vRes)
                if not seq.liveResult:
                    self.__setState('idle')
                    raise CalibrationError("Failed to acquire frames during calibration")
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

        # Load the FPN data.
        self.__applyBlackCal(fSize, fAverage // numFrames)
        self.__setState('idle')

        # Write the FPN data to file.
        if saveLocation:
            try:
                os.makedirs(saveLocation, exist_ok=True)
                fName = saveLocation + "/fpn_%dx%doff%dx%d" % (fSize.hRes, fSize.vRes, fSize.hOffset, fSize.vOffset)
                fpnFile = self.sensor.calFilename(fName, '.raw')
                logging.info("Saving black calibration to %s", fpnFile)
                numpy.array(fAverage / numFrames, dtype=numpy.uint16).tofile(fpnFile)
            except OSError as e:
                logging.error("Failed to write black calibration: %s", e)

    def __startZeroTimeBlackCal(self):
        """Begin the black calibration procedure using a zero-time exposure.

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
            The sleep time, in seconds, between steps of the calibration procedure.
        
        Examples
        --------
        This function returns a generator iterator with the sleep time between the
        steps of the black calibration procedure. The caller may use this for
        cooperative multithreading, or can complete the calibration sychronously
        as follows:

        state = camera.startZeroTimeBlackCal()
        for delay in state:
            time.sleep(delay)
        """
        # Grab the current frame size and exposure.
        fSize = self.sensor.getCurrentGeometry()
        expPrev = self.sensor.getCurrentExposure()
        expMin, expMax = self.sensor.getExposureRange(fSize)
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
    
    def startCalibration(self, blackCal=False, analogCal=False, zeroTimeBlackCal=False, saveCal=False, factory=False):
        """Begin one or more calibration procedures at the current settings.

        Black calibration takes a sequence of images with the lens cap or shutter
        closed and averages them to find the black level of each pixel on the
        image sensor. This value is then be subtracted during playback to
        correct for image offset defects.

        Analog calibration consists of any automated image sensor calibration
        that can be performed quickly and autonomously without any setup from
        the user (eg: no closing of the aperture or calibration jigs).

        Factory calibration algorithms may require special test equipment or
        setups. Factory calibration also implies that calibration data will
        be saved, and that conflicting user calibration data will be removed.

        Args:
            blackCal (bool, optional): Perform a full black calibration assuming
                the user has closed the aperture or lens cap. (default: false)
            analogCal (bool, optional): Perform autonomous analog calibration of
                the image sensor. (default: false)
            zeroTimeBlackCal (bool, optional): Perform a fast black calibration
                by reducing the exposure time and aperture to their minimum values.
                (default: false)
            saveCal (bool, optional): Whether the results of calibration should be
                saved to the filesystem for later use. (default: false)
            factory (bool, optional): Whether factory calibration algorithms should
                be performed. (default: false)
        
        Yields:
            float : The sleep time, in seconds, between steps of the calibration procedure.
        
        Example:
            This function returns a generator iterator with the sleep time between steps
            of the calibration procedures. The caller may use this for cooperative
            multithreading, or can complete the calibration sychronously as follows:

            >>> state = camera.startCalibration(blackCal=True)
            >>> for delay in state:
            >>>    time.sleep(delay)
        """
        # Clear the calibration suggestion on call.
        self.__calSuggested = False
        self.__propChange("calSuggested")

        # Select paths, if any, depending on saveCal or factory mode.
        if factory:
            calLocation = "/var/camera/cal"
            fpnLocation = "/var/camera/cal/factoryFPN"
        elif saveCal:
            calLocation = "/var/camera/cal"
            fpnLocation = "/var/camera/userFPN"
        else:
            calLocation = None
            fpnLocation = None

        # Perform autonomous sensor calibration first.
        if analogCal:
            logging.info('starting analog calibration')
            self.__setState('analogcal')
            try:
                yield from self.sensor.startAnalogCal(calLocation)
            except Exception as e:
                self.__setState('idle')
                raise e
            self.__setState('idle')

        # Perform at most one black calibration.
        if blackCal:
            self.__setState('blackcal')
            logging.info('starting standard black calibration')
            yield from self.__startBlackCal(saveLocation=fpnLocation)
            self.__setState('idle')
        elif zeroTimeBlackCal:
            self.__setState('blackcal')
            logging.info('starting zero time black calibration')
            yield from self.__startZeroTimeBlackCal()
            self.__setState('idle')

    def loadCalibration(self):
        try:
            self.sensor.loadAnalogCal("/var/camera/cal")
            self.__calSuggested = not self.__loadBlackCal("/var/camera/userFPN", "/var/camera/cal/factoryFPN")
        except Exception as e:
            self.__calSuggested = True
        self.__propChange("calSuggested")
        return not self.__calSuggested

    def clearCalibration(self, factory=False):
        """Removes user calibration data, returning the camera to its factory state.

        When called with no arguments, this removes only the user calibration, allowing
        the camera to return to its factory new state. The caller may also specify the
        removal of factory calibration data, though this is not recommended unless the
        user has made a backup of their calibration data first.

        Args:
            factory (bool, optional): Also remove factory calibration data. (default: false)
        """
        for root, dirs, files in os.walk("/var/camera/userFPN", topdown=False):
            for name in files:
                os.remove(os.path.join(root, name))
                logging.debug("FILE: %s", os.path.join(root, name))
            for name in dirs:
                os.rmdir(os.path.join(root, name))
                logging.debug("DIR: %s", os.path.join(root, name))
        
        if not factory:
            return None
        
        for root, dirs, files in os.walk("/var/camera/cal", topdown=False):
            for name in files:
                os.remove(os.path.join(root, name))
                logging.debug("FILE: %s", os.path.join(root, name))
            for name in dirs:
                os.rmdir(os.path.join(root, name))
                logging.debug("DIR: %s", os.path.join(root, name))

    def exportCalData(self):
        """Generates factory calibration samples and saves them to external storage

        This method iterates through the image sensor's internal calibration modes and generates
        factory calibration sample data to be processed externally. The calibration data will be
        saved to a USB thumb drive, typically mounted at /media/sda1.

        After external processing of the calibration samples is complete, the resulting calibration
        data can be imported to the camera using the `importCalData` method.
        """
        try:
            yield from self.sensor.startFlatFieldExport()
            logging.info("back from \"startFlatFieldExport\" call")
        except Exception as e:
            self.__setState('idle')
            logging.info("had exception in \"startFlatFieldExport\" call")
            raise e
        self.__setState('idle')

    def importCalData(self):
        """Imports calibration data that was generated off-camera.

        This method looks for any calibration data present on a USB thumb drive, typically mounted
        at /media/sda1, and copies the calibration data to the camera's internal filesystem for
        later use.
        
        This method is used during factory calibration to import calibration data that the camera
        is not capable of generating on its own. Typically the camera will be connected to a test
        jig to stimulate the camera, with data being acquired using the `exportCalData` method.
        """
        self.sensor.importColGains()

    #===============================================================================================
    # API Parameters: Configuration Dictionary
    @camProperty()
    def config(self):
        """Return a configuration dictionary of all saveable parameters"""
        logging.debug('Config getter called')
        result = {}
        for name in dir(type(self)):
            try:
                prop = getattr(type(self), name, None)
                if (isinstance(prop, property) and getattr(prop.fget, 'saveable', False)):
                    result[name] = getattr(self, name)
            except AttributeError:
                logging.error('AttributeError while accessing: %s', name)
        return result

    #===============================================================================================
    # API Parameters: Camera Info Group
    @camProperty()
    def cameraApiVersion(self):
        """Version string of the pychronos module"""
        return pychronos.__version__
    
    @camProperty()
    def cameraFpgaVersion(self):
        """Version string of the FPGA bitstream that is currently running"""
        config = regmaps.config()
        return "%d.%d" % (config.version, config.subver)
    
    @camProperty()
    def cameraMemoryGB(self):
        """Amount of video memory attached to the FPGA in GiB"""
        return (self.dimmSize[0] + self.dimmSize[1]) / (1 << 30)
    
    @camProperty()
    def cameraModel(self):
        """Camera model name"""
        boardrev = utils.getBoardRevision()
        if (boardrev == '0000'):
            return "CR14-1.0"
        # Stitch the model and hardware revision together.
        model = "CR" + boardrev[0] + boardrev[1]
        hwmajor = chr(ord(boardrev[2])+1)
        hwminor = boardrev[3]
        return model + '-' + hwmajor + '.' + hwminor
    
    @camProperty()
    def cameraSerial(self):
        """Unique camera serial number"""
        EEPROM_ADDR = 0x54 # From the C++ app
        try:
            serial = utils.smbusRead(EEPROM_ADDR, 12, setup=bytearray([0, 0]))
            serlength = 0
            
            # Truncate up to the first NULL byte.
            for i in range(0, len(serial)):
                serlength = i                
                if not serial[i]:
                    break
            
            return serial[0:serlength].decode("utf-8").strip('\0')
        except:
            return ""
    
    @camProperty(notify=True, save=True)
    def cameraDescription(self):
        """Descriptive string assigned by the user"""
        return self.description
    @cameraDescription.setter
    def cameraDescription(self, value):
        if not isinstance(value, str):
            raise TypeError("cameraDescription must be a string")
        self.description = value
        self.__propChange("cameraDescription")

    @camProperty(notify=True, save=True)
    def cameraIdNumber(self):
        """Unique camera number assigned by the user"""
        return self.idNumber
    @cameraIdNumber.setter
    def cameraIdNumber(self, value):
        try:
            value = int(value)
        except:
            pass
        if not isinstance(value, int):
            raise TypeError("cameraIdNumber must be an integer got %s instead" % (type(value)))
        self.idNumber = value
        self.__propChange("cameraIdNumber")
    
    @camProperty(notify=True, save=True)
    def cameraTallyMode(self):
        """Mode in which the recording LEDs should operate."""
        return self.__tallyMode
    @cameraTallyMode.setter
    def cameraTallyMode(self, value):
        # Update the LEDs and tally state.
        self.__tallyMode = TallyModes[value]
        if (self.__tallyMode == TallyModes.on):
            ledstate = '1'
        elif (self.__tallyMode == TallyModes.off):
            ledstate = '0'
        elif (self.__tallyMode == TallyModes.auto):
            ledstate = '1' if self.__state == 'recording' else '0'
        else:
            raise ValueError("cameraTallyMode value of '%s' is not supported" % (value))

        with open(REC_LED_FRONT, 'w') as fp:
            fp.write(ledstate)
        with open(REC_LED_BACK, 'w') as fp:
            fp.write(ledstate)

        self.__propChange('cameraTallyMode')
    
    #===============================================================================================
    # API Parameters: Sensor Info Group
    @camProperty()
    def sensorName(self):
        """Descriptive name of the image sensor."""
        return self.sensor.name
    
    @camProperty()
    def sensorColorPattern(self):
        """String describing the color filter array pattern of the image sensor.
        
        For example, a typical 2x2 Bayer pattern sensor might have a value of 'GRBG',
        while a monochrome image sensor would have a value of 'mono'.
        """ 
        if self.sensor.cfaPattern:
            return self.sensor.cfaPattern
        else:
            return "mono"

    @camProperty()
    def sensorBitDepth(self):
        """Number of bits per pixel sampled by the image sensor."""
        fSize = self.sensor.getMaxGeometry()
        return fSize.bitDepth
    
    @camProperty()
    def sensorPixelRate(self):
        """Approximate throughput of the image sensor in pixels per second."""
        fSize = self.sensor.getMaxGeometry()
        return (fSize.vRes + fSize.vDarkRows) * fSize.hRes / fSize.minFrameTime
    
    @camProperty()
    def sensorIso(self):
        """ISO number of the image sensor with nominal (0dB) gain applied."""
        return self.sensor.baseIso
    
    @camProperty()
    def sensorMaxGain(self):
        """Maximum gain of the image sensor as a linear muliplier of the `sensorISO`."""
        return self.sensor.maxGain
    
    @camProperty()
    def sensorVMax(self):
        """Maximum vertical resolution, in pixels, of the active area of the image sensor."""
        fSize = self.sensor.getMaxGeometry()
        return fSize.vRes
    
    @camProperty()
    def sensorVMin(self):
        """Minimum vertical resolution, in pixels, of the active area of the image sensor."""
        return self.sensor.vMin

    @camProperty()
    def sensorVIncrement(self):
        """Minimum step size allowed, in pixels, for changes in the vertical resolution of the image sensor."""
        return self.sensor.vIncrement

    @camProperty()
    def sensorHMax(self):
        """Maximum horizontal resolution, in pixels, of the active area of the image sensor."""
        fSize = self.sensor.getMaxGeometry()
        return fSize.hRes
    
    @camProperty()
    def sensorHMin(self):
        """Minimum horizontal resolution, in pixels, of the active area of the image sensor."""
        return self.sensor.hMin

    @camProperty()
    def sensorHIncrement(self):
        """Minimum step size allowed, in pixels, for changes in the horizontal resolution of the image sensor."""
        return self.sensor.hIncrement

    @camProperty()
    def sensorVDark(self):
        """Maximum vertical resolution, in pixels, of the optical black regions of the sensor."""
        fSize = self.sensor.getMaxGeometry()
        return fSize.vDarkRows
    
    @camProperty()
    def sensorTemperature(self):
        """The temperature, in degrees Celcius, measured near the image sensor."""
        if not self.__sensorTemperatureAddr:
            return 0.0

        try:
            tempaddr = self.__sensorTemperatureAddr
            tempdata = utils.smbusRead(tempaddr, 2, setup=bytearray([0]))
            sign = -1.0 if (tempdata[0] & 0x80) else 1.0
            value = (tempdata[0] & 0x7f) + (tempdata[1] / 256)
            if (self.__sensorTemperatureAddr == 0x4c):
                return value * sign * 2 # Some boards have an LM73 sensor instead.
            else:
                return value * sign     # Most boards have an LM75-compatible sensor.
        except:
            return 0.0

    #===============================================================================================
    # API Parameters: Exposure Group
    def __setupExposure(self, expPeriod, expMode):
        """Internal helper to setup the exposure time and mode."""
        if expMode == ExposureModes.normal:
            self.sensor.setExposureProgram(expPeriod)
        elif expMode == ExposureModes.frameTrigger:
            self.sensor.setFrameTriggerProgram(expPeriod)
        elif expMode == ExposureModes.shutterGating:
            self.sensor.setShutterGatingProgram()
        elif expMode == ExposureModes.hdr2slope:
            self.sensor.setHdrExposureProgram(expPeriod, 2)
        elif expMode == ExposureModes.hdr3slope:
            self.sensor.setHdrExposureProgram(expPeriod, 3)
        else:
            raise NotImplementedError()
        
        self.__exposurePeriod = expPeriod
        self.__exposureMode = expMode

    @camProperty(notify=True, save=True, prio=props.PRIO_EXPOSURE)
    def exposurePeriod(self):
        """Minimum period, in nanoseconds, that the image sensor is currently exposing frames for."""
        return int(self.__exposurePeriod * 1000000000)
    @exposurePeriod.setter
    def exposurePeriod(self, value):
        self.__checkState('idle', 'recording')
        self.__setupExposure(value / 1000000000, self.__exposureMode)
        self.__propChange("exposurePeriod")

    @camProperty(prio=props.PRIO_EXPOSURE)
    def exposurePercent(self):
        """The current exposure time rescaled between `exposureMin` and `exposureMax`. This value is 0% when exposure is at minimum, and increases linearly until exposure is at maximum, when it is 100%."""
        fSize = self.sensor.getCurrentGeometry()
        fSize.minFrameTime = self.sensor.getCurrentPeriod()
        expMin, expMax = self.sensor.getExposureRange(fSize)
        return (self.sensor.getCurrentExposure() - expMin) * 100 / (expMax - expMin)
    @exposurePercent.setter
    def exposurePercent(self, value):
        self.__checkState('idle', 'recording')
        fSize = self.sensor.getCurrentGeometry()
        fSize.minFrameTime = self.sensor.getCurrentPeriod()
        expMin, expMax = self.sensor.getExposureRange(fSize)

        self.__setupExposure((value * (expMax - expMin) / 100) + expMin, self.__exposureMode)
        self.__propChange("exposurePeriod")

    @camProperty(prio=props.PRIO_EXPOSURE)
    def exposureNormalized(self):
        """The current exposure time rescaled between `exposureMin` and `exposureMax`. This value is 0 when exposure is at minimum, and increases linearly until exposure is at maximum, when it is 1.0."""
        fSize = self.sensor.getCurrentGeometry()
        fSize.minFrameTime = self.sensor.getCurrentPeriod()
        expMin, expMax = self.sensor.getExposureRange(fSize)
        return (self.sensor.getCurrentExposure() - expMin) * 1 / (expMax - expMin)
    @exposureNormalized.setter
    def exposureNormalized(self, value):
        self.__checkState('idle', 'recording')
        fSize = self.sensor.getCurrentGeometry()
        fSize.minFrameTime = self.sensor.getCurrentPeriod()
        expMin, expMax = self.sensor.getExposureRange(fSize)

        self.__setupExposure((value * (expMax - expMin) / 1) + expMin, self.__exposureMode)
        self.__propChange("exposurePeriod")

    @camProperty(prio=props.PRIO_EXPOSURE)
    def shutterAngle(self):
        """The angle in degrees for which frames are being exposed relative to the frame time."""
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
        """The minimum possible time, in nanoseconds, that the image sensor is capable of exposing
        a frame for at the current `resolution` and `framePeriod`.""" 
        fSize = self.sensor.getCurrentGeometry()
        fSize.minFrameTime = self.sensor.getCurrentPeriod()
        expMin, expMax = self.sensor.getExposureRange(fSize)
        return int(expMin * 1000000000)
    
    @camProperty(notify=True)
    def exposureMax(self):
        """The maximum possible time, in nanoseconds, that the image sensor is capable of exposing
        a frame for at the current `resolution` and `framePeriod`.""" 
        fSize = self.sensor.getCurrentGeometry()
        fSize.minFrameTime = self.sensor.getCurrentPeriod()
        expMin, expMax = self.sensor.getExposureRange(fSize)
        return int(expMax * 1000000000)
    
    @camProperty(notify=True, save=True)
    def exposureMode(self):
        """Mode in which frame timing and exposure should operate."""
        return self.__exposureMode
    @exposureMode.setter
    def exposureMode(self, value):
        self.__checkState('idle')
        # Not all sensors support all modes.
        if value not in self.sensor.getSupportedExposurePrograms():
            raise ValueError("exposureMode value of '%s' is not supported" % (value))
        
        self.__setupExposure(self.__exposurePeriod, ExposureModes[value])
        self.__propChange("exposureMode")

    #===============================================================================================
    # API Parameters: Gain Group
    @camProperty(notify=True, save=True)
    def currentGain(self):
        """The current gain of the image sensor as a linear multiplier of `sensorIso`."""
        return self.sensor.getCurrentGain()
    @currentGain.setter
    def currentGain(self, value):
        self.__checkState('idle')
        self.sensor.setGain(value)
        self.__propChange("currentGain")
    
    @camProperty()
    def currentIso(self):
        """The ISO number of the image sensor at the current current gain."""
        return self.sensor.getCurrentGain() * self.sensor.baseIso
    @currentIso.setter
    def currentIso(self, value):
        self.__checkState('idle')
        self.sensor.setGain(value / self.sensor.baseIso)
        self.__propChange("currentGain")

    @camProperty(notify=True, save=True)
    def digitalGain(self):
        """Digital image gain applied during video processing."""
        disp = pychronos.regmaps.display()
        return float(disp.gain / 4096)
    @digitalGain.setter
    def digitalGain(self, value):
        disp = pychronos.regmaps.display()
        gainval = int(value * 4096)
        if (gainval > 0xffff):
             disp.gain = 0xffff
        elif (gainval < 0):
             disp.gain = 0
        else:
             disp.gain = gainval
        self.__propChange("digitalGain")

    #===============================================================================================
    # API Parameters: Camera Status Group
    @camProperty(notify=True)
    def state(self):
        """The current operating state of the camera.
        
        Args:
            idle: The camera is powered up and operating, but not doing anything.
            reset: The camera is in the process of resetting the FPGA and image sensor.
            blackCal: The camera is currently calibrating using a dark reference image.
            analogCal: The camera is currently performing analog calibration of the image sensor.
            recording: The camera is running a recording program to save images into video memory. 
        """
        return self.__state
    
    @camProperty()
    def dateTime(self):
        """The current date and time in ISO-8601 format."""
        return datetime.datetime.now().isoformat()

    @camProperty(notify=True, save=False)
    def backlightEnabled(self):
        """True if the LCD on the back of the camera is lit. Can be set to False to dim the screen and save a small amount of power."""
        with open(BACKLIGHT_PIN, 'r') as fp:
            return fp.read(1) != '0'        
    @backlightEnabled.setter
    def backlightEnabled(self, value):
        with open(BACKLIGHT_PIN, 'w') as fp:
            fp.write('1' if bool(value) else '0')
        self.__propChange("backlightEnabled")

    @camProperty()
    def externalStorage(self):
        """The currently attached external storage partitions and their status. The sizes
        of the reported storage devices are in units of kB.
        
        Examples:
            >>> print(json.dumps(camera.externalStorage, indent=3))
            {
                \"mmcblk1p1\": {
                    \"mount\": \"/media/mmcblk1p1\",
                    \"device\": \"/dev/mmcblk1p1\",
                    \"fstype\": \"vfat\",
                    \"description\": \"MMC/SD Card Partition 1\"
                }
            }
        """
        return utils.getStorageDevices()

    #===============================================================================================
    # API Parameters: Camera Network Group
    @camProperty()
    def networkHostname(self):
        """Hostname to be used for dhcp requests and to be displayed on the command line.
        """
        return utils.getHostname()
    @networkHostname.setter
    def networkHostname(self, name):
        utils.setHostname(name)
    
    #===============================================================================================
    # API Parameters: Recording Group
    @camProperty(notify=True, save=True)
    def recMode(self):
        """Mode in which the recording sequencer stores frames into video memory."""
        return self.__recMode
    @recMode.setter
    def recMode(self, value):
        self.__checkState('idle')
        self.__recMode = RecModes[value]
        self.__propChange("recMode")

        ## whenever the record mode is changed, write those settings into the fpga so that an IO block 'start' works
        if self.__recMode == RecModes.normal:
            # Record into a single segment until the trigger event.
            cmd = regmaps.seqcommand(blockSize=self.recMaxFrames,
                            blkTermFull=False, blkTermRising=True,
                            recTermMemory=False, recTermBlockEnd=True)
            program = [cmd]
        elif self.__recMode == RecModes.segmented:
            # Record into segments 
            cmd = regmaps.seqcommand(blockSize=self.recMaxFrames // self.recSegments,
                            blkTermFull=False, blkTermRising=True,
                            recTermMemory=self.__disableRingBuffer, recTermBlockEnd=False)
            program = [cmd]
        elif self.__recMode == RecModes.burst:
            # When trigger is inactive, save the pre-record into a ring buffer.
            precmd = regmaps.seqcommand(blockSize=self.recPreBurst, blkTermRising=True)
            # While trigger is active, save frames into the remaining memory.
            burstcmd = regmaps.seqcommand(blockSize=self.recMaxFrames - self.recPreBurst - 1,
                            blkTermFalling=True, recTermMemory=False)
            program = [precmd, burstcmd]
        else:
            raise ValueError("recording mode of '%s' is not supported" % str(mode))

        seq = regmaps.sequencer()
        # Setup the nextStates into a loop, just in case the caller forgot and then
        # load the program into the recording sequencer.
        for i in range(0, len(program)):
            program[i].nextState = (i + 1) % len(program)
            seq.program[i] = program[i]

        seq.trigDelay = self.__recTrigDelay ## make sure the trigger delay gets written




    @camProperty(notify=True, save=True)
    def recMaxFrames(self):
        """Limit on the maximum number of frames for the recording sequencer to use."""
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
        self.setupRecordRegion(self.sensor.getCurrentGeometry(), self.REC_REGION_START, value)
        self.__propChange("recMaxFrames")

    @camProperty(notify=True, save=True)
    def recSegments(self):
        """The number of segments used by the recording sequencer when in 'segmented' recording mode."""
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
        """The number of frames leading up to the trigger rising edge to save when in 'burst' recording mode."""
        return self.__recPreBurst
    @recPreBurst.setter
    def recPreBurst(self, value):
        if not isinstance(value, int):
            raise TypeError("recPreBurst must be an integer")
        if value < 1:
            raise ValueError("recPreBurst must be greater than zero")
        self.__checkState('idle')
        self.__recPreBurst = value
        self.__propChange("recPreBurst")
    
    @camProperty(save=True)
    def disableRingBuffer(self):
    	"""When true, the camera will stop recording once the RAM buffer is full instead of looping over.
    	
    	By default, the camera will enable the ring buffer, so once the maximum record length has been reached, the camera will overwrite the oldest footage in the recording in normal recording mode, or overwrite the oldest segment in sedgmented recording mode."""
    	return self.__disableRingBuffer
    @disableRingBuffer.setter
    def disableRingBuffer(self, value):
    	self.__disableRingBuffer = value
    	self.__propChange("disableRingBuffer")

    @camProperty(notify=True, save=True)
    def recTrigDelay(self):
        """The number of frames to delay the trigger rising edge by in 'normal' and 'segmented' recording modes."""
        return self.__recTrigDelay
    @recTrigDelay.setter
    def recTrigDelay(self, value):
        if not isinstance(value, int):
            raise TypeError("recTrigDelay must be an integer")
        if value < 0:
            raise ValueError("recTrigDelay must be positive")
        self.__checkState('idle')
        self.__recTrigDelay = value
        self.__propChange("recTrigDelay")

    @camProperty(notify=True)
    def cameraMaxFrames(self):
        """The maximum number of frames the camera's memory can save at the current resolution."""
        fSize = self.sensor.getCurrentGeometry()
        return self.getRecordingMaxFrames(fSize)
    
    @camProperty(notify=True, save=True, prio=props.PRIO_RESOLUTION)
    def resolution(self):
        """Resolution geometry at which the image sensor should capture frames.
        
        The optional `hOffset` and `vOffset` parameters allow the user to select where on
        the sensor to position the frame when operating at a cropped resolution. If not
        provided when setting, the camera will attempt to centre the cropped image on the
        image sensor.

        When setting resolution, the `minFrameTime` may be optionally provided to allow
        the image sensor to better tune itself for the desired frame period. When omitted,
        it is assumed that the sensor will tune itself for its maximum framerate.

        Args:
            hRes (int): Horizontal resolution of the catpured image, in pixels.
            vRes (int): Vertical resolution of the captured image, in pixels.
            hOffset (int, optional): Horizontal offset, in pixels, from the top left of the
                full frame at which the first pixel will be read out.
            vOffset (int, optional): Vertical offset, in pixels, from the top left of the
                full frame at which the first pixel will be read out.
            vDark (int, optional): The number of vertical dark rows to read out.
            minFrameTime (float, optional): The minimum frame time, in seconds, that the
                image sensor is capable of recording frames when at this resolution
                configuration.
        """
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

        # Set defaults if not provided by value
        fMax = self.sensor.getMaxGeometry()
        if not "hOffset" in value:
            geometry.hOffset = (fMax.hRes - geometry.hRes) / 2
        if not "vOffset" in value:
            geometry.vOffset = (fMax.vRes - geometry.vRes) / 2
        if not "vDarkRows" in value:
            geometry.vDarkRows = 0
        if not "bitDepth" in value:
            geometry.bitDepth = fMax.bitDepth

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
    def calSuggested(self):
        """True when the calibration of the camera needs updating."""
        return self.__calSuggested

    @camProperty(notify=True)
    def minFramePeriod(self):
        """The minimum frame period, in nanoseconds, at the current resolution settings."""
        fSize = self.sensor.getCurrentGeometry()
        fpMin, fpMax = self.sensor.getPeriodRange(fSize)
        return int(fpMin * 1000000000)

    @camProperty(notify=True, save=True, prio=props.PRIO_FRAME_TIME)
    def framePeriod(self):
        """The time, in nanoseconds, to record a single frame."""
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

    @camProperty(prio=props.PRIO_FRAME_TIME)
    def frameRate(self):
        """The estimated estimated recording rate in frames per second (reciprocal of `framePeriod`)."""
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
    @camProperty(notify=True)
    def wbColor(self):
        """The Red, Green and Blue gain coefficients to achieve white balance."""
        display = regmaps.display()
        return [
            display.whiteBalance[0] / display.WHITE_BALANCE_DIV,
            display.whiteBalance[1] / display.WHITE_BALANCE_DIV,
            display.whiteBalance[2] / display.WHITE_BALANCE_DIV
        ]
    @wbColor.setter
    def wbColor(self, value):
        display = regmaps.display()
        display.whiteBalance[0] = int(value[0] * display.WHITE_BALANCE_DIV)
        display.whiteBalance[1] = int(value[1] * display.WHITE_BALANCE_DIV)
        display.whiteBalance[2] = int(value[2] * display.WHITE_BALANCE_DIV)
        self.__propChange("wbColor")
    
    @camProperty(notify=True, save=True)
    def wbCustomColor(self):
        """The Red, Green and Blue gain coefficients last computed by `startWhiteBalance()`."""
        return self.__wbCustomColor
    @wbCustomColor.setter
    def wbCustomColor(self, value):
        self.__wbCustomColor[0] = value[0]
        self.__wbCustomColor[1] = value[1]
        self.__wbCustomColor[2] = value[2]
        
        # A color temperature of zero means: use wbCustom.
        if (self.__wbTemperature == 0):
            self.wbColor = self.__wbCustomColor
        
        self.__propChange("wbCustomColor")
    
    @camProperty(notify=True, save=True)
    def wbTemperature(self):
        """Color temperature, in degrees Kelvin, to use for white balance."""
        return self.__wbTemperature
    @wbTemperature.setter
    def wbTemperature(self, value):
        if not isinstance(value, (int, float)):
            raise TypeError("wbTemperature must be an integer")
        tempk = int(value)
        if tempk < 0:
            raise ValueError("wbTemperature must be greater than absolute zero")
        
        # A color temperature of zero means: use wbCustomColor.
        if (tempk == 0):
            self.wbColor = self.__wbCustomColor
        else:
            self.wbColor = self.getWhiteBalance(tempk)
        
        self.__wbTemperature = tempk
        self.__propChange("wbTemperature")

    @camProperty(notify=True, save=True)
    def colorMatrix(self):
        """The matrix coefficients for a 3x3 color matrix converting the image sensor color space into sRGB. The values are stored in row-scan order."""
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
    # API Parameters: Misc
    @camProperty(notify=True, save=True)
    def miscScratchPad(self):
        """A dictionary of arbitrary values that can be stored in the camera."""
        if self.__miscScratchPad == {}:
            return {"empty":True}
        else:
            return self.__miscScratchPad
    @miscScratchPad.setter
    def miscScratchPad(self, value):
        for key,value in value.items():
            if value is None or value == 'null':
                if key in self.__miscScratchPad:
                    del self.__miscScratchPad[key]
            else:
                self.__miscScratchPad[key] = value

        self.__propChange('miscScratchPad')
    
