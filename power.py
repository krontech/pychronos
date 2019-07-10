import os, time
import socket
import sys
import select

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
	
	bufferSize = 1000;
	PATH = "/var/run/bmsFifo";
	pipe = None
	client = None

	def parsePower(self, str):
		"""Parse a string containing battery data"""
		elements = str.split()
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
		# Now calculate capacity percent from voltage
		if self.flags & 4:	# If battery is charging
			self.battCapacityPercent = 				within((self.battVoltageCam/1000.0 - 10.75) / (12.4 - 10.75) * 80, 0.0, 80.0) + \
							20 - 20*within((self.battCurrentCam/1000.0 - 0.1) / (1.28 - 0.1), 0.0, 1.0) 
		else:   #discharging
			self.battCapacityPercent = within((self.battVoltageCam/1000.0 - 9.75) / (11.8 - 9.75) * 100, 0.0, 100.0)
		#charge 10.75 - 12.4 is 0 - 80%
		if not self.flags & 1:
			self.battVoltageCam = 0
			self.battCapacityPercent = 0
		self.acAdaptorPresent = self.flags & 2

		#print("Voltage:", self.battVoltageCam)
		#print("Charge:", self.battCapacityPercent, "%")
		#print("Flags", self.flags)

	def readPowerFifo(self):	
		try:
			input = os.read(pipe,bufferSize)
		except OSError as err:
			if err.errno == 11:
				print(".", end='')
				return
			else:
				raise err
		if input:
			parsePower(input)
		else:
			pass

	def openPowerFifo(self):
		self.pipe = os.open(self.PATH, os.O_RDONLY | os.O_NONBLOCK);
		 
	def openPowerSocket(self):
		"""This opens the socket to the power controller daemon"""
		if os.path.exists("/tmp/pcUtil.socket"):
			gclient = socket.socket(socket.AF_UNIX)
			gclient.setblocking(0)
			gclient.connect("/tmp/pcUtil.socket")
			print("CLIENT:", gclient)
			#self.receivePowerSocket(self)

	def queryPowerSocket(self):
		"""This polls the power controller daemon for battery data"""
		gclient.send("GET_BATTERY_DATA".encode('utf-8'))
		ret = gclient.recv(1000)
		self.parsePower(self, ret)

	def receivePowerSocket(self):
		ret = gclient.recv(1000) #.encode('utf-8')
		print (ret, ".")
		self.parsePower(self, ret)
		
	def nonBlockPowerSocket(self):
		inputs = [gclient]
		outputs = []
		for i in range(1):
			readable, writeable, exceptional = select.select(inputs, outputs, inputs, 0)

			if(readable):
				try:
					ret = gclient.recv(1000)
					self.parsePower(self, ret)

				except KeyboardInterrupt as k:
					print("Shutting down.")
					gclient.close()
					break

	def setShippingMode(self, en):
		"""This sets whether shipping mode is enabled"""
		print("shipping mode:", eb)
		if en:
			gclient.send("SET_SHIPPING_MODE_ENABLED".encode('utf-8'))
		else:
			gclient.send("SET_SHIPPING_MODE_DISABLED".encode('utf-8'))


	def setPowerMode(self, powerOn, powerOff):
		"""This sets the automatic power-on and automatic save and power-off, when the power supply is connected or disconnected"""
		print(powerOn, powerOff)
		num = powerOn + 2*powerOff
		modeStr = "SET_POWERUP_MODE_" + str(num)
		print(modeStr)
		gclient.send(modeStr.encode('utf-8'))				


print("Connecting...")
if os.path.exists("/tmp/pcUtil.socket"):
	gclient = socket.socket(socket.AF_UNIX) #, socket.SOCK_STREAM)
	gclient.setblocking(0)
	gclient.connect("/tmp/pcUtil.socket")

