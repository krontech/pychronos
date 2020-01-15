from abc import ABC, abstractmethod

class frameGeometry:
    """Container to express the geometry of images recorded from an image sensor.

    Attributes:
        hRes (int): Horizontal resolution, in pixels, of the recorded image.
        vRes (int): Vertical resolution, in pixels, of the recorded image.
        hOffset (int): Horizontal offset from the top left of the sensor to the first recorded pixel.
        vOffset (int): Vertical offset from the top left of the sensor to the first recorded pixel.
        vDarkRows (int): Number of optical dark rows to be recorded and appended to the top of the image.
        bitDepth (int): Number of bits recorded per pixel from the image sensor.
        minFrameTime (float): Minimum time recording time per frame at this resolution settings.
    """
    def __init__(self, hRes, vRes, hOffset=0, vOffset=0, vDarkRows=0, bitDepth=12, minFrameTime=0.0):
        self.hRes = int(hRes)
        self.vRes = int(vRes)
        self.hOffset = int(hOffset)
        self.vOffset = int(vOffset)
        self.vDarkRows = int(vDarkRows)
        self.bitDepth = int(bitDepth)
        self.minFrameTime = float(minFrameTime)
    
    def __repr__(self):
        rstr = "frameGeometry(hRes=%s, vRes=%s" % (self.hRes, self.vRes)
        rstr += ", hOffset=%s, vOffset=%s" % (self.hOffset, self.vOffset)
        rstr += ", vDarkRows=%s, bitDepth=%s" % (self.vDarkRows, self.bitDepth)
        if (self.minFrameTime):
            rstr += ", minFrameTime=%s" % (self.minFrameTime)
        return rstr + ")"

    def pixels(self):
        """Compute the number of pixels in a recorded image.
        
        Returns:
            int: Number of pixels in the image.
        """
        return self.hRes * (self.vRes + self.vDarkRows)
    
    def size(self):
        """Compute the size in bytes of a recorded image.
        
        Returns:
            int: Size of bytes of a recorded image.
        """
        return self.pixels() * self.bitDepth // 8

# Abstract Sensor class
class api(ABC):
    """Abstract class defining the API to an image sensor."""
    def __init__(self):
        super().__init__()
    
    #--------------------------------------------
    # Sensor Configuration and Control API
    #--------------------------------------------
    @abstractmethod
    def reset(self, fSize=None):
        """Perform a reset of the image sensor and bring it into normal operation.
        
        This function may be called either to initialzie the image sensor, or to restart it
        if the sensor is already running. After a reset, the expected state of the image
        sensor is to be operating at full-frame resolution, the maximum framerate, nominal
        (0dB) gain and a 180-degree shutter angle.

        This function returns no value, but may throw an exception if the initialization
        procedure failed.

        Args:
            fSize (:obj:`frameGeometry`, optional): The initial video geometry to set after
                initializing the image sensor, or use the maximum active video by default.
        """
        pass

    #--------------------------------------------
    # Sensor Descriptive Properties
    #--------------------------------------------
    @property
    @abstractmethod
    def name(self):
        """str: The name of the image sensor."""
        pass
    
    @property
    def cfaPattern(self):
        """str: String describing the color filter array pattern, or `None` for monochrome sensors."""
        return None
    
    @property
    def baseIso(self):
        """int: The ISO number of the image sensor at normal (0dB) gain."""
        return 100

    @property
    def maxGain(self):
        """int: The maximum gain supported by the sensor as a multiplier of baseIso."""
        return 1
    
    @property
    def hMin(self):
        """int: The minimum horizontal resolution, in pixels, of the image sensor"""
        return 1
    
    @property
    def hIncrement(self):
        """int: The minimum step size, in pixels, for changes in horizontal resolution"""
        return 1

    @property
    def vMin(self):
        """int: The minimum vertical resolution, in pixels, of the image sensor"""
        return 1
    
    @property
    def vIncrement(self):
        """int: The minimum step size, in pixels, for changes in vertical resolution"""
        return 1
    
    @property
    def wbPresets(self):
        """Dict[List[float]]: Dictionary of white balance points, indexed by their color temperature."""
    
    #--------------------------------------------
    # Frame Geometry Configuration Functions
    #--------------------------------------------
    @abstractmethod
    def getMaxGeometry(self):
        """Return the maximum frame geometry supported by the image sensor
        
        Returns:
            :obj:`frameGeometry`
        """
        pass
    
    @abstractmethod
    def getCurrentGeometry(self):
        """Return the current frame size of the image sensor
        
        Returns:
            :obj:`frameGeometry`
        """
        pass
    
    def isValidResolution(self, fSize):
        """Test if the provided geometry is supported by the iamge sensor.
        
        Args:
            fSize (:obj:`frameGeometry`): The resolution to test for support.

        Returns:
            bool: `True` if `fSize` is a supported resolution, and `False` otherwise.
        """
        # Default implementation only checks agianst the maximums. Sensors
        # should normally override this with some more accurate sanity checks.
        limits = self.getMaxGeometry()
        if ((size.hRes + size.hOffset) > limits.hRes):
            return False
        if ((size.vRes + size.vOffset) > limits.vRes):
            return False
        if (size.vDarkRows > limits.vDarkRows):
            return False
        if (size.bitDepth != limits.bitDepth):
            return False
        return True

    @abstractmethod
    def setResolution(self, fSize):
        """Configure the resolution and frame geometry of the image sensor.
        
        Args:
            fSize (:obj:`frameGeometry`): Frame size and geometry to configure.
        
        Raises:
            ValueError: If `fSize` is not a valid resolution for the image sensor.
        """
        pass

    #--------------------------------------------
    # Frame Timing Configuration Functions
    #--------------------------------------------
    @abstractmethod
    def getPeriodRange(self, fSize):
        """Return a tuple with the minimum and maximum frame periods at a given frame size
        
        Args:
            fSize (:obj:`frameGeometry`): The frame size for which the frame period limits
                are being requested.

        Returns:
            (float, float): A tuple of (min, max) frame periods in seconds, or zero to
                indicate that the frame period is not limited.

        Raises:
            ValueError: If `fSize` is not a valid resolution for the image sensor.
        """
        pass

    @abstractmethod
    def getCurrentPeriod(self):
        """Return the current frame period of the image sensor.
        
        Returns:
            float: Current frame period in seconds
        """
        pass
    
    @abstractmethod
    def setFramePeriod(self, fPeriod):
        """Configure the frame minimum period of the image sensor
        
        Args:
            fPeriod (float): The frame period to configure on the image sensor.
        
        Raises:
            ValueError: If `fPeriod` is not a valid frame period at the current resolution. 
        """
        pass

    @abstractmethod
    def getExposureRange(self, fSize):
        """Return a tuple with the minimum and maximum exposure at a given frame size
        
        Args:
            fSize (:obj:`frameGeometry`): The frame size at which the exposure range is being requested.

        Returns:
            (float, float) : A tuple of (min, max) exposure periods in seconds, or zero to
                indicate that te exposure period is not limited.
        
        Raises:
            ValueError: If `fSize` is not a valid resolution for the image sensor.
        """
        pass
    
    @abstractmethod
    def getCurrentExposure(self):
        """Return the current image sensor exposure time.
        
        Returns:
            float: The current exposure time in seconds.
        """
        pass
    
    @abstractmethod
    def setExposureProgram(self, expPeriod):
        """Configure the sensor to operate in normal exposure mode.

        When in normal exposure mode, the image sensor is free running
        and will capture frames continuously  with the desired exposure
        period. This function may be called to enter the standard
        exposure program or to update the exposure time.

        This function is mandatory for all image sensors.

        Args:
            expPeriod (float): The exposure time of each frame, in seconds.
        """
        pass

    #--------------------------------------------
    # Advanced Exposure and Timing Functions
    #--------------------------------------------
    def getSupportedExposurePrograms():
        """Return a tuple of the supported exposure programs."""
        return ("normal")

    def setShutterGatingProgram(self):
        """Configure the sensor to operate in shutter gating mode.

        When in shutter gating mode, the image sensor is inactive until
        the trigger signal is asserted. Once asserted the exposure duration
        is controlled by the active period of the trigger signal. Frame
        readout begins on the falling edge of the trigger signal, after
        which the sensor becomes idle again until the next rising edge
        of the trigger.

        Raises:
            NotImplementedError: If shutter gating is not supported by the image sensor.
        """
        raise NotImplementedError()

    def setFrameTriggerProgram(self, expPeriod, numFrames=1):
        """Configure the sensor to operate in frame trigger mode.

        When in frame trigger mode, the image sensor is inactive until a
        rising edge of the frame trigger is detected, after which the
        image sensor will capture a fixed number of frames and then return
        to an idle state until the next rising edge is detected.
        
        Args:
            expPeriod (float): The exposure time of each frame, in seconds.
            numFrames (int, optional): The number of frames to acquire after each risng edge (default: 1)
        
        Raises:
            ValueError: If `expPeriod` is not a valid exposure time at the current resolution.
            NotImplementedError: If frame trigger mode is not supported by the image sensor.
        """
        raise NotImplementedError()
    
    def setHdrExposureProgram(self, expPeriod, numIntegration=2):
        """Configure the sensor to operate in high dynamic range mode

        When in high dynamic range mode, the image sensor combines multiple
        integration periods to achieve a non-linear response to incoming
        sensitivities. 
        
        Args:
            expPeriod (float): The total exposure time of each frame, in seconds.
            numIntegrations (int, optional): The number of integration periods to apply for HDR mode. (default: 2)

        Raises:
            ValueError: If `expPeriod` is not a valid exposure time at the current resolution.
            ValueError: If `numIntegration` is not supported by the image sensor.
            NotImplementedError: If high dynamic range mode is not supported by the image sensor.
        """
        raise NotImplementedError()

    #--------------------------------------------
    # Sensor Analog Calibration Functions
    #--------------------------------------------
    def getColorMatrix(self, cTempK=5500):
        """Return the color correction matrix for this image sensor.

        A color matrix is required to convert the image sensor's bayer filter data into
        sRGB color space. If the image sensor is characterized under multiple lighting
        conditions, the matrix which best matches the provided color temperature should
        be returned.
        
        Args:
            cTempK (int, optional): The color temperature (degrees Kelvin) of the CIE
                D-series illuminant under which the color matrix will be used (default 5500K).

        Returns:
            List[float]: The matrix coefficients for the 3x3 matrix converting the camera
                color space into sRGB. The matrix coefficients are stored in row-scan order.
        """
        ## Return an identity matrix if not implemented.
        return [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    
    def getWhiteBalance(self, cTempK=5500):
        """Return the white balance matrix for this image sensor.

        A white balance is required to equalize the sensitivity of each channel of the
        bayer filter data to achieve white. If the image sensor is characterized under
        multiple lighting conditions, the white balance which best matches the provided
        color temperature should be returned.
        
        Args:
            cTempK (int, optional): The color temperature (degrees Kelvin) of the CIE
                D-series illuminant under which the white balance will be used (default 5500K).

        Returns:
            List[float]: An array containing the gains for the Red, Green and Blue channels
                to achieve white balance at the desired lighting temperature.
        """
        ## Return a do-nothing white balance if not implemented.
        return [1.0, 1.0, 1.0]
    
    @abstractmethod
    def startAnalogCal(self, saveLocation=None):
        """Perform the automatic analog sensor calibration at the current settings.

        Any analog calibration that the sensor can perform without requiring any
        extern user setup, such as covering the lens cap or attaching calibration
        jigs, should be performed by this call.

        Args:
            saveLocation (string, optional): Filesystem path to the directory where
                calibration data should should be stored.

        Yields:
            float: The sleep time, in seconds, between steps of the calibration procedure.
        
        Examples:
            This function returns a generator iterator with the sleep time between the
            steps of the analog calibration procedure. The caller may use this for
            cooperative multithreading, or can complete the calibration sychronously
            as follows:

            >>> for delay in sensor.startAnalogCal():
            >>>    time.sleep(delay)
        """
        pass
    
    def startBlackCal(self, saveLocation=None):
        """Perform sensor black calibration routines at the current settings.

        Any calibration steps that the sensor can perform while assuming a black
        reference image, should be performed by this call.

        Args:
            saveLocation (string, optional): Filesystem path to the directory where
                calibration data should should be stored.

        Yields:
            float: The sleep time, in seconds, between steps of the calibration procedure.
        
        Examples:
            This function returns a generator iterator with the sleep time between the
            steps of the analog calibration procedure. The caller may use this for
            cooperative multithreading, or can complete the calibration sychronously
            as follows:

            >>> for delay in sensor.startBlackCal():
            >>>    time.sleep(delay)
        """
        yield from ()

    def calFilename(self, prefix, extension=""):
        """Generate the filename for sensor calibration data at the current settings.

        Calibration data may depend on any number of internal sensor settings that
        are not directly exposed via the sensor API. The caller may use this function
        to generate calibration filenames that are unique to the current operating
        conditions.

        Args:
            prefix (string): Filename prefix to begin calibration data with.
            extension (string, optional): File extension to end the calibration data with.

        Returns:
            string: A filename, beginning with prefix, that is unique to the current
                settings which affect calibration.
        """
        return prefix + extension
    
    def loadAnalogCal(self, calLocation):
        """Load stored analog calibration data from a file, if supported
        
        Args:
            calLocation (string): Filesystem path to the directory where
                calibration data is stored.
        
        Returns:
            bool: `True` if the calibration data was successfully loaded, or
                `False` if calibration data was expected but not found.
        """
        return False
    
    def loadBlackCal(self, calLocation, factoryLocation):
        """Loads stored black calibration data from a file, if supported

        Args:
            calLocation (string): Filesystem path to the directory where
                user calibration data is stored.
            factoryLocation (string, optional): Filesystem path to a directory
                where factory calibration can be loaded if no user calibration
                data is present
        
        Returns:
            bool: `True` if the calibration data was successfully loaded, or
                `False` if calibration data was expected but not found.
        """
        return False
    
    @abstractmethod
    def setGain(self, gain):
        """Configure the analog gain of the image sensor"""
        pass
    
    @abstractmethod
    def getCurrentGain(self, gain):
        """Return the current analog gain of the image sensor"""
        pass 

    @abstractmethod
    def exportCalData(self, saveLocation=None):
        """Export flat-fields while in ADC test mode for PC based calibration

        Args:
            saveLocation (string): Filesystem path to a directory that can
                be transferred to an external PC for processing.

        Returns:
            bool: `True` if the flat-fields were successfully captured and
                saved. `False` if the capture or saving failed.
        """
        pass

    @abstractmethod
    def importColGains(self, sourceLocation='/media/sda1', calLocation='/var/camera/cal'):
        """Import column gain calibration data (.bin files) that were generated off-camera
        
        Args:
            sourceLocation (string, optional): Filesystem path to the location of the .bin
                calibration files that were generated off-camera. Defaults to a usb thumb
                drive on /media/sda1.

            calLocation (string, optional): Filesystem path to the directory where
                user calibration data is stored. Defaults to /var/camera/cal.

        Returns:
            bool: `True` if all calibration data files were copied for each level of
                analog gain and wavetable. `False` if one or more files were not
                copied.
        """
        pass