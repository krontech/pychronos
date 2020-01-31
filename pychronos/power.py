import os
import socket
import select
import logging
import re

from . import props
from .props import camProperty as camProperty

def within(x, min, max):
    if x > max:
        return max
    if x < min:
        return min
    return x

class pDataCache:
    def __init__(self):
        # Battery data fields from the PMIC.
        self.battCapacityPercent = 0
        self.battSOHPercent = 0
        self.battVoltage = 0
        self.battCurrent = 0
        self.battHiResCap = 0
        self.battHiResSOC = 0
        self.battVoltageCam = 0
        self.battCurrentCam = 0
        self.mbTemperature = 0
        self.flags = 0
        self.fanPWM = 0

        # Modes used for power and fan status.
        self.powerMode = 0
        self.fanOverride = -1

class power:
    # Power flag bitmasks.
    FLAG_BATTERY_PRESENT   = 1 << 0
    FLAG_ADAPTOR_PRESENT   = 1 << 1
    FLAG_CHARGING          = 1 << 2
    FLAG_AUTO_POWERON      = 1 << 3
    FLAG_OVER_TEMP         = 1 << 4
    FLAG_SHIPPING_MODE     = 1 << 5
    FLAG_SHUTDOWN_REQUEST  = 1 << 6

    def __init__(self, onChange=None, path="/tmp/pcUtil.socket"):
        self.cache = pDataCache()
        self.onChange = onChange
        
        # Battery critical tracking.
        self._batteryCritical = False
        self._batteryCriticalThreshold = 5
        self._batteryCriticalHystersis = 5

        if (os.path.exists(path)):
            self.gclient = socket.socket(socket.AF_UNIX)
            self.gclient.setblocking(0)
            self.gclient.connect(path)
        else:
            self.gclient = None
        
        # Issue initial requests for the power mode and fan control.
        self.sendMessage("GET_POWERUP_MODE")
        self.sendMessage("GET_FAN_MODE")

    def sendMessage(self, string):
        if (self.gclient):
            self.gclient.send(string.encode('utf-8'))

    def __propChange(self, name):
        """Quick and dirty wrapper to throw an on-change event by name"""
        try:
            logging.info("%s changed", name)
            self.onChange(name, getattr(self, name))
        except Exception as e:
            logging.debug("onChange handler failed: %s", e)
            pass
    
    parsePowerRegex = re.compile(b'([a-zA-Z]+) ([0-9]+)')
    def parsePower(self, powerControllerMessages: bytes):
        """Parse a string containing battery data"""
        changedFlags = self.cache.flags
        for key, value in self.parsePowerRegex.findall(powerControllerMessages):
            name = key.decode('utf-8')
            try:
                intval = int(value)
                setattr(self.cache, name, intval)
            except ValueError:
                setattr(self.cache, name, str(value))

        # Recompute battery charge estimate
        if not self.cache.flags & self.FLAG_BATTERY_PRESENT:
            self.cache.battVoltageCam = 0
            self.cache.battCapacityPercent = 0
        elif self.cache.flags & self.FLAG_CHARGING: 
            self.cache.battCapacityPercent = within((self.cache.battVoltageCam/1000.0 - 10.75) / (12.4 - 10.75) * 80, 0.0, 80.0) + \
                20 - 20*within((self.cache.battCurrentCam/1000.0 - 0.1) / (1.28 - 0.1), 0.0, 1.0)
        else:
            self.cache.battCapacityPercent = within((self.cache.battVoltageCam/1000.0 - 9.75) / (11.8 - 9.75) * 100, 0.0, 100.0)

        # Determine if the battery is critical.
        changedCritical = self._batteryCritical
        if (self.cache.flags & self.FLAG_ADAPTOR_PRESENT):
            self._batteryCritical = False
        elif (self.cache.battCapacityPercent < self._batteryCriticalThreshold):
            self._batteryCritical = True
        elif (self.cache.battCapacityPercent > (self._batteryCriticalThreshold + self._batteryCriticalHystersis)):
            self._batteryCritical = False
        if (changedCritical != self._batteryCritical):
            self.__propChange("batteryCritical")
        
        # Check for flags that have changed.
        changedFlags ^= self.cache.flags
        if (changedFlags & self.FLAG_ADAPTOR_PRESENT):
            self.__propChange("externalPower")
        if (changedFlags & self.FLAG_BATTERY_PRESENT):
            self.__propChange("batteryPresent")

    def checkPowerSocket(self):
        if not self.gclient:
            return False

        inputs = [self.gclient]
        outputs = []
        readable, writeable, exceptional = select.select(inputs, outputs, inputs, 0)
        if (readable):
            try:
                ret = self.gclient.recv(1000)
                self.parsePower(ret)
                return True
            except KeyboardInterrupt as k:
                logging.info("Shutting down.")
                self.gclient.close()
                return False
        return False

    def setPowerMode(self, mode):
        """This sets the automatic power-on and automatic save and power-off, when the power supply is connected or disconnected"""
        self.cache.powerMode = mode
        self.sendMessage("SET_POWERUP_MODE_" + str(mode))
    
    #===============================================================================================
    # API Parameters: Power Management Group
    #===============================================================================================
    @camProperty(notify=True)
    def shippingMode(self):
        """bool: True when the camera is configured for shipping mode"""
        return bool(self.cache.flags & self.FLAG_SHIPPING_MODE)
    @shippingMode.setter
    def shippingMode(self, value):
        if (value):
            self.sendMessage("SET_SHIPPING_MODE_ENABLED")
        else:
            self.sendMessage("SET_SHIPPING_MODE_DISABLED")

    @camProperty(notify=True)
    def externalPower(self):
        """bool: True when the AC adaptor is present, and False when on battery power."""
        return bool(self.cache.flags & self.FLAG_ADAPTOR_PRESENT)

    @camProperty(notify=True)
    def batteryPresent(self):
        """bool: True when the battery is installed, and False when the camera is only running on adaptor power"""
        return bool(self.cache.flags & self.FLAG_BATTERY_PRESENT)
    
    @camProperty(notify=True)
    def batteryCritical(self):
        """bool: True when the battery voltate is critically low and a powerdown is imminent"""
        return self._batteryCritical

    @camProperty()
    def batteryChargePercent(self):
        """float: Estimated battery charge, with 0% being depleted and 100% being fully charged."""
        return self.cache.battCapacityPercent
    
    @camProperty(derivedFrom="batteryChargePercent")
    def batteryChargeNormalized(self):
        """float: Estimated battery charge, with 0.0 being depleted and 1.0 being fully charged."""
        return self.cache.battCapacityPercent / 100
    
    @camProperty()
    def batteryVoltage(self):
        """float: The voltage that is currently being output from the removable battery. A healthy and fully charged battery outputs between 12v and 12.5v. This value is graphed on the battery screen on the Chronos."""
        return self.cache.battVoltageCam / 1000
    
    @camProperty(notify=True)
    def fanOverride(self):
        """float: Fan speed in the range of 0=off to 1.0=full, or -1 for automatic fan control."""
        return -1.0 if (self.cache.fanOverride < 0) else (self.cache.fanOverride / 255.0)
    
    @fanOverride.setter
    def fanOverride(self, value):
        if (value < 0):
            self.cache.fanOverride = -1
            self.sendMessage('SET_FAN_AUTO')
        elif (value >= 1.0):
            self.cache.fanOverride = 255
            self.sendMessage('SET_FAN_255')
        else:
            self.cache.fanOverride = int(value * 255)
            self.sendMessage('SET_FAN_%d' % (self.cache.fanOverride))
        self.__propChange("fanOverride")

    @camProperty(notify=True)
    def powerOnWhenMainsConnected(self):
        """bool: Set to `True` to have the camera turn itself on when it is plugged into mains power."""
        return bool(self.cache.powerMode & 1)
        
    @powerOnWhenMainsConnected.setter
    def powerOnWhenMainsConnected(self, val):
        if (val):
            self.setPowerMode(self.cache.powerMode | 1)
        else:
            self.setPowerMode(self.cache.powerMode & ~1)
        self.__propChange("powerOnWhenMainsConnected")
    
    @camProperty(notify=True)
    def powerOffWhenMainsLost(self):
        """bool: Set to `True` to have the camera turn itself off when it is unplugged from mains power."""
        return bool(self.cache.powerMode & 2)
        
    @powerOffWhenMainsLost.setter
    def powerOffWhenMainsLost(self, val):
        if (val):
            self.setPowerMode(self.cache.powerMode | 2)
        else:
            self.setPowerMode(self.cache.powerMode & ~2)
        self.__propChange("powerOffWhenMainsLost")
