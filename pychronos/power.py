import os, time
import socket
import sys
import select

# Power information bit masks
POWER_BATTERY_PRESENT   = 1 << 0
POWER_ADAPTOR_PRESENT   = 1 << 1
POWER_CHARGING          = 1 << 2
POWER_AUTO_POWERON      = 1 << 3
POWER_OVER_TEMP         = 1 << 4
POWER_SHIPPING_MODE     = 1 << 5
POWER_SHUTDOWN_REQUEST  = 1 << 6

def within(x, min, max):
    if x > max:
        return max
    if x < min:
        return min
    return x

class power:
    def __init__(self, path="/tmp/pcUtil.socket"):
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
        self.acAdaptorPresent = False
        self.lastAcAdaptorPresent = False

        if (os.path.exists(path)):
            self.gclient = socket.socket(socket.AF_UNIX)
            self.gclient.setblocking(0)
            self.gclient.connect(path)
        else:
            self.gclient = None
	
    def parsePower(self, str):
        """Parse a string containing battery data"""
        self.lastAcAdaptorPresent = self.acAdaptorPresent
        elements = str.split()
        if elements[0].decode('utf8') != "battCapacityPercent":
            return
        size = len(elements)
        for i in range(0, size//2):
            element = elements[2*i].decode('utf8')
            value = int(elements[2*i + 1].decode('utf8'))
            if element == "battCapacityPercent":
                self.battCapacityPercent = value
            elif element == "battSOHPercent":
                self.battSOHPercent = value
            elif element == "battVoltage":
                self.battVoltage = value
            elif element == "battCurrent":
                self.battCurrent = value
            elif element == "battHiResCap":
                self.battHiResCap = value
            elif element == "battHiResSOC":
                self.battHiResSOC = value
            elif element == "battVoltageCam":
                self.battVoltageCam = value
            elif element == "battCurrentCam":
                self.battCurrentCam = value
            elif element == "mbTemperature":
                self.mbTemperature = value
            elif element == "flags":
                self.flags = value
            elif element == "fanPWM":
                self.fanPWM = value
        if self.flags & POWER_CHARGING:	
            self.battCapacityPercent = within((self.battVoltageCam/1000.0 - 10.75) / (12.4 - 10.75) * 80, 0.0, 80.0) + \
                20 - 20*within((self.battCurrentCam/1000.0 - 0.1) / (1.28 - 0.1), 0.0, 1.0) 
        else:
            self.battCapacityPercent = within((self.battVoltageCam/1000.0 - 9.75) / (11.8 - 9.75) * 100, 0.0, 100.0)
        if not self.flags & POWER_ADAPTOR_PRESENT:
            self.battVoltageCam = 0
            self.battCapacityPercent = 0
        self.acAdaptorPresent = bool(self.flags & POWER_ADAPTOR_PRESENT)

    def checkPowerSocket(self):
        if not self.gclient:
            return False

        inputs = [self.gclient]
        outputs = []
        readable, writeable, exceptional = select.select(inputs, outputs, inputs, 0)
        if (readable):
            try:
                ret = gclient.recv(1000)
                self.parsePower(self, ret)
                return True
            except KeyboardInterrupt as k:
                logging.info("Shutting down.")
                gclient.close()
                return False
        return False

    def setShippingMode(self, en):
        """This sets whether shipping mode is enabled"""
        if en:
            self.gclient.send("SET_SHIPPING_MODE_ENABLED".encode('utf-8'))
        else:
            self.gclient.send("SET_SHIPPING_MODE_DISABLED".encode('utf-8'))

    def setPowerMode(self, powerOn, powerOff):
        """This sets the automatic power-on and automatic save and power-off, when the power supply is connected or disconnected"""
        num = powerOn + 2*powerOff
        modeStr = "SET_POWERUP_MODE_" + str(num)
        self.gclient.send(modeStr.encode('utf-8'))				
