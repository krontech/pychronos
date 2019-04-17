# Chronos High-Speed Camera class
import fcntl
import time
import os
import numpy
import logging

import pychronos
import pychronos.regmaps as regmaps
import pychronos.spd as spd

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

    def __init__(self, sensor):
        self.sensor = sensor
        self.dimms = [0, 0]

        # Probe the SODIMMs
        for slot in range(0, len(self.dimms)):
            spdData = spd.spdRead(slot)
            if (spdData):
                self.dimms[slot] = spdData.size

    def reset(self, bitstream=None):
        """Reset the camera and initialize the FPGA and image sensor.

        Parameters
        ----------
        bitstream : str, optional
            File path to the FPGA bitstream to load, or None to perform
            only a soft-reset of the FPGA.
        """

        # Setup the FPGA if a bitstream was provided.
        if (bitstream):
            os.system("cam-loader %s" % (bitstream))
            config = regmaps.config()
            config.sysReset = 1
            time.sleep(0.2)
            logging.info("Loaded FPGA Version %s.%s", config.version, config.subver)
        else:
            config = regmaps.config()
            config.sysReset = 1
            time.sleep(0.2)
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

        displayRegs = regmaps.display()
        displayRegs.control = displayRegs.COLOR_MODE
        i = 0
        for row in self.sensor.getColorMatrix():
            for val in row:
                displayRegs.colorMatrix[i] = val * displayRegs.COLOR_MATRIX_DIV
                i += 1

        # Load a default calibration
        colGainRegs = pychronos.fpgamap(pychronos.FPGA_COL_GAIN_BASE, 0x1000)
        colCurveRegs = pychronos.fpgamap(pychronos.FPGA_COL_CURVE_BASE, 0x1000)
        for x in range(0, self.geometry.hRes):
            colGainRegs.mem16[x] = (1 << 12)
            colCurveRegs.mem16[0] = 0

        # Reboot the sensor and enter live display mode
        self.sensor.reset()
        self.sensor.setResolution(self.geometry)

    def setupMemory(self):
        configRegs = regmaps.config()

        # We require at least one DIMM to be present.
        if ((self.dimms[0] + self.dimms[1]) == 0):
            raise RuntimeError("Memory configuration failed, no DIMMs detected")

        if (self.dimms[1] > self.dimms[0]):
            # Swap DIMMs to put the largest one first.
            configRegs.mmuConfig = configRegs.MMU_INVERT_CS
        elif (self.dimms[0] < (16 << 30)):
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
            ramSizeWords = self.getMemorySize() // self.BYTES_PER_WORD
            seq.regionStop = (ramSizeWords // fSizeWords) * fSizeWords

    def getMemorySize(self):
        return self.dimms[0] + self.dimms[1]
    
    # Return the length of memory (in frames) minus calibration overhead.
    def getRecordingMaxFrames(self, fSize):
        ramSizeWords = self.getMemorySize() // self.BYTES_PER_WORD - self.REC_REGION_START
        fSizeWords = (fSize.size() + self.BYTES_PER_WORD - 1) // self.BYTES_PER_WORD
        fSizeWords //= self.FRAME_ALIGN_WORDS
        fSizeWords *= self.FRAME_ALIGN_WORDS
        return ramSizeWords // fSizeWords
    
    def setRecordingConfig(self, fSize, fPeriod=None, expPeriod=None):
        if (not self.sensor.isValidResolution(fSize)):
            raise ValueError("Unsupported resolution setting")
        
        # Default to maximum framerate if not provided.
        if (fPeriod is None):
            minPeriod, maxPeriod = self.sensor.getPeriodRange(fSize)
            fPeriod = minPeriod
        
        self.sensor.setResolution(fSize, fPeriod)
        self.setupRecordRegion(fSize, self.REC_REGION_START)
        if (expPeriod is not None):
            self.sensor.setExposurePeriod(expPeriod)

        ## TODO: Attempt to load calibration files, if present.

    # Read the serial number - or make it an attribute?
    def getSerialNumber(self):
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

    def startBlackCal(self, numFrames=16, useLiveBuffer=True):
        """Begin the black calibration proceedure at the current settings.

        Black calibration takes a sequence of images with the lens cap or shutter
        closed and averages them to find the black level of the image sensor. This
        value can then be subtracted during playback to correct for image offset
        defects.

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

        state = camera.startBlackCal()
        for delay in state:
            time.sleep(delay)
        """
        # get the resolution from the display properties
        # TODO: We actually want to get it from the sequencer.
        display = regmaps.display()
        xres = display.hRes
        yres = display.vRes

        logging.debug('Starting Black Calibration')

        seq = regmaps.sequencer()
        fAverage = numpy.zeros((yres, xres))
        if (useLiveBuffer):
            # Readout and average the frames from the live buffer.
            for i in range(0, numFrames):
                logging.debug('waiting for frame')
                yield from seq.startLiveReadout(xres, yres)
                fAverage += numpy.asarray(seq.liveResult)
        else:
            # Take a recording and read the results from the live buffer.
            program = [regmaps.seqcommand(blockSize=numFrames+1, recTermBlkEnd=True, recTermBlkFull=True)]
            logging.debug('making recording')
            yield from seq.startRecording([program])
            addr = seq.regionStart
            for i in range(0, numFrames):
                fAverage += numpy.asarray(pychronos.readframe(addr, xres, yres))
                addr += seq.frameSize
                yield 0

        fAverage /= numFrames
        
        # Readout the column gain and linearity calibration.
        colGainRegs = pychronos.fpgamap(pychronos.FPGA_COL_GAIN_BASE, display.hRes * 2)
        colCurveRegs = pychronos.fpgamap(pychronos.FPGA_COL_CURVE_BASE, display.hRes * 2)
        colOffsetRegs = pychronos.fpgamap(pychronos.FPGA_COL_OFFSET_BASE, display.hRes * 2)
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
        for x in range(0, xres):
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

        print("---------------------------------------------")
        print("fpn details: min = %d, max = %d" % (numpy.min(fAverage), numpy.max(fAverage)))
        print("fpn standard deviation: %d" % (numpy.std(fAverage)))
        print("fpn standard deviation horiz: %s" % (numpy.std(fAverage, axis=1)))
        print("fpn standard deviation vert:  %s" % (numpy.std(fAverage, axis=0)))
        print("---------------------------------------------")

    def startZeroTimeBlackCal(self):
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

        # Reconfigure for the minum exposure supported by the sensor.
        self.sensor.setExposurePeriod(expMin)

        # Do a fast black cal from the live display buffer.
        # TODO: We might get better quality out of the zero-time cal by
        # testing a bunch of exposure durations and finding the actual
        # zero-time intercept.
        yield (3 / 60) # ensure the exposure time has taken effect.
        yield from self.startBlackCal(numFrames=2, useLiveBuffer=True)

        # Restore the previous exposure settings.
        self.sensor.setExposurePeriod(expPrev)

    def startRecording(self, program):
        """Program the recording sequencer and start recording.

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
        seq.control |= seq.START_REC
        seq.control &= ~seq.START_REC

        # Yield until recording is complete.
        yield 0.1
        while (seq.status & seq.ACTIVE_REC) != 0:
            yield 0.1
    
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
        displayRegs.whiteBalance[0] = whiteBalance[0] * displayRegs.WHITE_BALANCE_DIV
        displayRegs.whiteBalance[1] = whiteBalance[1] * displayRegs.WHITE_BALANCE_DIV
        displayRegs.whiteBalance[2] = whiteBalance[2] * displayRegs.WHITE_BALANCE_DIV
