import os
import socket
import select
import re

def within(x, min, max):
    if x > max:
        return max
    if x < min:
        return min
    return x

class power:
    # Power flag bitmasks.
    FLAG_BATTERY_PRESENT   = 1 << 0
    FLAG_ADAPTOR_PRESENT   = 1 << 1
    FLAG_CHARGING          = 1 << 2
    FLAG_AUTO_POWERON      = 1 << 3
    FLAG_OVER_TEMP         = 1 << 4
    FLAG_SHIPPING_MODE     = 1 << 5
    FLAG_SHUTDOWN_REQUEST  = 1 << 6

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
    
    parsePowerRegex = re.compile(b'([a-zA-Z]+) ([0-9]+)')
    def parsePower(self, powerControllerMessages: bytes):
        """Parse a string containing battery data"""
        for key, value in self.parsePowerRegex.findall(powerControllerMessages):
            setattr(self, key.decode('utf8'), int(value))
        
        if self.flags & self.FLAG_CHARGING: 
            self.battCapacityPercent = within((self.battVoltageCam/1000.0 - 10.75) / (12.4 - 10.75) * 80, 0.0, 80.0) + \
                20 - 20*within((self.battCurrentCam/1000.0 - 0.1) / (1.28 - 0.1), 0.0, 1.0) 
        else:
            self.battCapacityPercent = within((self.battVoltageCam/1000.0 - 9.75) / (11.8 - 9.75) * 100, 0.0, 100.0)
        if not self.flags & self.FLAG_ADAPTOR_PRESENT:
            self.battVoltageCam = 0
            self.battCapacityPercent = 0
        
        self.lastAcAdaptorPresent = self.acAdaptorPresent
        self.acAdaptorPresent = bool(self.flags & self.FLAG_ADAPTOR_PRESENT)

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
