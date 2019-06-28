import os
import socket
import sys

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
			self.client = socket.socket(socket.AF_UNIX)
			self.client.connect("/tmp/pcUtil.socket")

	def queryPowerSocket(self):
		"""This polls the power controller daemon for battery data"""
		self.client.send("GET_BATTERY_DATA".encode('utf-8'))
		ret = self.client.recv(1000)
		self.parsePower(self, ret)
