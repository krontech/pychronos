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

AUTOPOWER_ON 			= 1 << 0
AUTOPOWER_OFF			= 1 << 1

gclient = None

def within(x, min, max):
	if x > max:
		return max
	if x < min:
		return min
	return x

class powerClass:

	battCapacityPercent = 0
	battSOHPercent = 0
	battVoltage = 0
	battCurrent = 0
	battHiResCap = 0
	battHiResSOC = 0
	battVoltageCam = 0
	battCurrentCam = 0
	mbTemperature = 0
	flags = 0
	fanPWM = 0
	acAdaptorPresent = False
	lastAcAdaptorPresent = False
	
	bufferSize = 1000;
	PATH = "/var/run/bmsFifo";
	pipe = None
	client = None

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

	def openPowerSocket(self):
		"""This opens the socket to the power controller daemon"""
		if os.path.exists("/tmp/pcUtil.socket"):
			gclient = socket.socket(socket.AF_UNIX)
			gclient.setblocking(0)
			gclient.connect("/tmp/pcUtil.socket")

	def nonBlockPowerSocket(self):
		inputs = [gclient]
		outputs = []
		for i in range(1):
			readable, writeable, exceptional = select.select(inputs, outputs, inputs, 0)
			if(readable):
				try:
					ret = gclient.recv(1000)
					self.parsePower(self, ret)
					return True
				except KeyboardInterrupt as k:
					print("Shutting down.")
					gclient.close()
					break
		return False

	def setShippingMode(self, en):

		"""This sets whether shipping mode is enabled"""
		if en:
			self.sendToPcUtil(self, "SET_SHIPPING_MODE_ENABLED")
		else:
			self.sendToPcUtil(self, "SET_SHIPPING_MODE_DISABLED")

	def setPowerMode(self, powerOn, powerOff):
		"""This sets the automatic power-on and automatic save and power-off, when the power supply is connected or disconnected"""
		num = powerOn + 2 * powerOff
		if (num == 0): modeStr = "SET_POWERUP_MODE_0"
		elif (num == 1): modeStr = "SET_POWERUP_MODE_1"
		elif (num == 2): modeStr = "SET_POWERUP_MODE_2"
		elif (num == 3): modeStr = "SET_POWERUP_MODE_3"
		self.sendToPcUtil(self, modeStr)

	def sendToPcUtil(self, msg):
		logging.info("Send to pcUtil:", msg)
		gclient.send(msg.encode('utf-8'))				

if os.path.exists("/tmp/pcUtil.socket"):
	gclient = socket.socket(socket.AF_UNIX) #, socket.SOCK_STREAM)
	gclient.setblocking(0)
	gclient.connect("/tmp/pcUtil.socket")

