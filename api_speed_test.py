#!/usr/bin/python3
# -*- coding: future_fstrings -*-
"""Speed test PyQt5's D-Bus implementation."""

from twisted.web import server, resource
from twisted.internet import reactor, defer
from twisted.internet.defer import inlineCallbacks

from txdbus import client

import logging
import time



try:
	from aimCamera import aimCameraResource
except ImportError:
	aimCameraResource = None
	

def asleep(secs):
	"""
	@brief Do a reactor-safe sleep call. Call with yield to block until done.
	@param secs Time, in seconds
	@retval Deferred whose callback will fire after time has expired
	"""
	d = defer.Deferred()
	reactor.callLater(secs, d.callback, None)
	return d

TEST_ITERATIONS = 100

class Root(resource.Resource):
	"""
	Root resource; serves JavaScript
	"""
	def getChild(self, name, request):
		if name == '':
			return self
		return resource.Resource.getChild(self, name, request)


class Test():
	isLeaf = True
	def __init__(self, controlApi):
		self.controlApi = controlApi
		super().__init__()
	
	def run(self, request=None):
		t1 = time.perf_counter()
		print('test 1: simple calls to control api')
		for x in range(TEST_ITERATIONS):
			self.controlApi.callRemote('get', ['batteryVoltage'])
			print('.', end='', flush=True)
		print(f"""
Time: {time.perf_counter()-t1}s total, {(time.perf_counter()-t1)/TEST_ITERATIONS*1000}ms per call.
""")
		#Either of these produce too many errors.
		#reactor.stop()
		#sys.exit()

@inlineCallbacks
def main():

	root.putChild(b'', root)

	logging.info('Adding dbus signals')
	system = yield client.connect(reactor, 'system')
	controlApi = yield system.getRemoteObject('ca.krontech.chronos.control', '/ca/krontech/chronos/control')
	#videoApi   = yield system.getRemoteObject('ca.krontech.chronos.video',   '/ca/krontech/chronos/video')
	
	test = Test(controlApi)
	test.run()

root = Root()
reactor.callWhenRunning( main )
reactor.run()




"""
Results:

PyQt5.DBus; Unladen, no intercall time:
test 1: Time: 0.6735163000048487s total, 6.7353785000159405ms per call.
test 2: Time: 1.0611664000025485s total, 10.611879499992938ms per call.
test 3: Time: 1.0689314999981434s total, 10.68952249996073ms per call.

PyQt5.DBus; Unladen, 1ms intercall time:
test 1: Time: 0.6696369499986758s total, 6.696585999961826ms per call.
test 2: Time: 1.2051974499991047s total, 12.052213999995729ms per call.
test 3: Time: 1.1969613500041305s total, 11.969804000036675ms per call.


PyQt5.DBus; Laden, 1ms intercall time:
test 1: Time: 1.315425900000264s total, 13.154962500047986ms per call.
test 2: Time: 2.5850381999989622s total, 25.85057850003068ms per call.
test 3: Time: 2.7320590499948594s total, 27.320799000008265ms per call.

Twisted; Unladen:
test 1: Time: 0.4529215499997008s total, 4.5315949999985605ms per call.

Twisted; Laden:
test 1: Time: 1.6233974999995553s total, 16.234189500000866ms per call.

Notes:
1 frame is 16ms.
Test is getting battery voltage, since I think that's a quintisentially cheap operation.
Burdening tool was yes.
"""