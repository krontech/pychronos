from abc import ABC, abstractmethod

class frameGeometry:
    def __init__(self, hRes, vRes, hOffset=0, vOffset=0, vDarkRows=0, bitDepth=12):
        self.hRes = hRes
        self.vRes = vRes
        self.hOffset = hOffset
        self.vOffset = vOffset
        self.vDarkRows = vDarkRows
        self.bitDepth = bitDepth
    
    def __repr__(self):
        args = (self.hRes, self.vRes, self.hOffset, self.vOffset, self.vDarkRows, self.bitDepth)
        return "frameGeometry(hRes=%s, vRes=%s, hOffset=%s, vOffset=%s, vDarkRows=%s, bitDepth=%s)" % args
    
    def pixels(self):
        return self.hRes * (self.vRes + self.vDarkRows)
    
    def size(self):
        return self.pixels() * self.bitDepth // 8

# Abstract Sensor class
class api(ABC):
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
        proceedure failed.

        Parameters
        ----------
        fSize : frameGeometry (optional)
            The initial video geometry to set after initializing the image sensor, or
            use the default video geometry by default.
        """
        pass

    #--------------------------------------------
    # Sensor Descriptive Properties
    #--------------------------------------------
    @property
    @abstractmethod
    def name(self):
        """The name of the image sensor"""
        pass
    
    @property
    def cfaPattern(self):
        """The color filter array pattern or None for monochrome sensors"""
        return None

    #--------------------------------------------
    # Frame Geometry Configuration Functions
    #--------------------------------------------
    @abstractmethod
    def getMaxGeometry(self):
        """Return the maximum frame geometry supported by the image sensor
        
        Returns
        -------
        frameGeometry
        """
        pass
    
    @abstractmethod
    def getCurrentGeometry(self):
        """Return the current frame size of the image sensor
        
        Returns
        -------
        frameGeometry
        """
        pass
    
    def isValidResolution(self, size):
        """Test if the provided geometry is supported by the iamge sensor"""
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
    def setResolution(self, size, fPeriod=None):
        """Configure the resolution and frame geometry of the image sensor"""
        pass

    #--------------------------------------------
    # Frame Timing Configuration Functions
    #--------------------------------------------
    @abstractmethod
    def getPeriodRange(self, fSize):
        """Return a tuple with the minimum and maximum frame periods at a given frame size
        
        Parameters
        ----------
        fSize : `frameGeometry`
            The frame size for which the frame period limits are being requested.

        Returns
        -------
        (float, float) : A tuple of (min, max) frame periods in seconds, or zero to
            indicate that the frame period is not limited.
        """
        pass

    @abstractmethod
    def getCurrentPeriod(self):
        """Return the current frame period of the image sensor
        
        Returns
        -------
        float : Current frame period in seconds
        """
        pass
    
    @abstractmethod
    def setFramePeriod(self, fPeriod):
        """Configure the frame period of the image sensor
        
        Parameters
        ----------
        fPeriod : `float`
            The frame period to configure on the image sensor.
        """
        pass

    @abstractmethod
    def getExposureRange(self, fSize, fPeriod):
        """Return a tuple with the minimum and maximum exposure at a given frame size
        
        Parameters
        ----------
        fSize : `frameGeometry`
            The frame size at which the exposure range is being requested.
        fPeriod : `float`
            The frame period for which the exposure range is being requested.

        Returns
        -------
        (float, float) : A tuple of (min, max) exposure periods in seconds, or zero to
            indicate that te exposure period is not limited.
        """
        pass
    
    @abstractmethod
    def getCurrentExposure(self):
        """Return the current image sensor exposure time
        
        Returns
        -------
        float : The current exposure time in seconds.
        """
        pass
    
    @abstractmethod
    def setExposurePeriod(self, expPeriod):
        """Configure the exposure time for the image sensor"""
        pass
    
    #--------------------------------------------
    # Sensor Analog Calibration Functions
    #--------------------------------------------
    def getColorMatrix(self, cTempK=5500):
        """Return the color correction matrix for this image sensor.

        A color matrix is required to convert the image sensor's bayer filter data into
        sRGB color space. If the image sensor is characterized under multiple lighting
        conditions, the matrix which best matches the provided color temperature should
        be returned.
        
        Parameters
        ----------
        cTempK: int, optional
            The color temperature (degrees Kelvin) of the CIE D-series illuminant under
            which the color matrix will be used (default 5500K).

        Returns
        -------
        [[float]] : A 3x3 matrix of floats converting the camera color space to sRGB.
        """
        ## Return an identity matrix if not implemented.
        return [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    
    @abstractmethod
    def startAnalogCal(self):
        """Perform the automatic analog sensor calibration at the current settings.

        Any analog calibration that the sensor can perform without requiring any
        extern user setup, such as covering the lens cap or attaching calibration
        jigs, should be performed by this call.

        Yields
        ------
        float :
            The leep time, in seconds, between steps of the calibration proceedure.
        
        Examples
        --------
        This function returns a generator iterator with the sleep time between the
        steps of the analog calibration proceedure. The caller may use this for
        cooperative multithreading, or can complete the calibration sychronously
        as follows:

        for delay in sensor.startAnalogCal():
            time.sleep(delay)
        """
        pass
    
    def loadAnalogCal(self):
        """Load stored analog calibration data from a file, if supported"""
        raise NotImplementedError
    
    def saveAnalogCal(self):
        """Write analog calibration data from a file, if supported"""
        raise NotImplementedError
    
    @abstractmethod
    def setGain(self, gain):
        """Configure the analog gain of the image sensor"""
        pass
