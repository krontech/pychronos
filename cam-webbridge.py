#!/usr/bin/python3
import sys

from twisted.web import server, resource
from twisted.web.static import File
from twisted.internet import reactor, defer, utils
from twisted.python import log
from twisted.internet.defer import inlineCallbacks

from txdbus import client, error

import json
import logging

import cgi



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

def allowCrossOrigin(request, methods='GET, POST, OPTION', contentType='application/json'):
    # Append headers to allow cross-origin requests.
    request.setHeader('Access-Control-Allow-Origin', '*')
    request.setHeader('Access-Control-Allow-Methods', methods)
    if ('POST' in methods):
        request.setHeader('Access-Control-Allow-Headers', 'Content-Type')
    request.setHeader('Content-Type', contentType)
    request.setHeader('Access-Control-Max-Age', 2520)

eventList = {'test':'',
             'control/notify':'/com/krontech/chronos/control/notify',
             'video/segment':'/com/krontech/chronos/video/segment',
             'video/eof':'/com/krontech/chronos/video/eof',
             'video/sof':'/com/krontech/chronos/video/sof'}


class Root(resource.Resource):
    """
    Root resource; serves JavaScript
    """
    def getChild(self, name, request):
        if name == '':
            return self
        return resource.Resource.getChild(self, name, request)

    def render_GET(self, request):
        returnString = """
        <html>
            <head>
                <script language="JavaScript">
                        function updateEventDetails(event) {
                            eventName = document.getElementById("event-name");
                            eventData = document.getElementById("event-data");
                            eventName.innerHTML = event.type;
                            eventData.innerHTML = event.data;
                        }

                        eventSource = new EventSource("/subscribe");
        """
        for name in eventList.keys():
            returnString += '\n                        eventSource.addEventListener("{name}", updateEventDetails, false);'.format(name=name)
        returnString += """
                        eventSource.addEventListener("test", updateEventDetails, false);
                        eventSource.onmessage = updateEventDetails;
                    </script>
            </head>
            <body>
                <h3> Event name: </h3>
                <p id="event-name"></p>
                <h3> Event data: </h3>
                <p id="event-data"></p>
            </body>
        </html>
        """
        return bytes(returnString, 'utf8')


class GetMethod(resource.Resource):
    """
    Implements a callable method with arguments on dbus
    """
    isLeaf = True
    def __init__(self, parent, bus, methodName='get'):
        self.parent = parent
        self.bus = bus
        self.arguments = True

        self.parent.putChild(bytes(methodName, 'utf8'), self)

    def allowCrossOrigin(self, request):
        # Append headers to allow cross-origin requests.
        request.setHeader('Access-Control-Allow-Origin', '*')
        request.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTION')
        request.setHeader('Access-Control-Allow-Headers', 'Content-Type')
        request.setHeader('Access-Control-Max-Age', 2520)

    def render_OPTIONS(self, request):
        allowCrossOrigin(request)
        request.setHeader('Content-Type', 'application/json')
        request.write('')
        request.finish()
        return server.NOT_DONE_YET

    def render_GET(self, request):
        names = []
        logging.info('GET arguments: %s', request.args)
        names = request.args.get(b'names')
        if not names:
            request.setResponseCode(400)
            return b'"names" field required'
        # this lets multiple copies of 'names' concatenate together then breaks
        # them appart to individual items
        names = names[0].decode('utf8').split(',')

        allowCrossOrigin(request)
        request.setHeader('Content-Type', 'application/json')
        reactor.callLater(0.0, self.startDBusGetRequest, request, names)
        return server.NOT_DONE_YET
        
    def render_POST(self, request):
        contentType = request.getHeader("Content-Type")
        self.allowCrossOrigin(request)
        if contentType == "application/json":
            # Expect a JSON object for the POST data.
            names = request.content.getvalue().decode("utf8")
            request.setHeader('Content-Type', 'application/json')
            reactor.callLater(0.0, self.startDBusGetRequest, request, json.loads(names))
            return server.NOT_DONE_YET
        # Expect JSON data passed in URL-encoded form.
        rawData = request.args.get(b'names', None)
        if not rawData:
            request.setResponseCode(400)
            return b'"names" field required'
        names = ''
        for line in rawData:
            names += line.decode('utf8')
        names = json.loads(names)

        request.setHeader('Content-Type', 'application/json')
        reactor.callLater(0.0, self.startDBusGetRequest, request, names)
        return server.NOT_DONE_YET

    @inlineCallbacks
    def startDBusGetRequest(self, request, names):
        logging.debug('Get Request: %s (%s)', names, type(names))
        reply = yield self.bus.callRemote('get', names)

        returnData = json.dumps(reply)
        
        request.write(bytes('{0}\n'.format(returnData), 'utf8'))
        request.finish()

        logging.info('Get Method requested with names: %s with response: %s', names, returnData)



class Method(resource.Resource):
    """
    Implements a callable method with arguments on dbus
    """
    isLeaf = True
    def __init__(self, parent, bus, methodName, arguments=False):
        self.parent = parent
        self.bus = bus
        self.methodName = methodName
        self.arguments = arguments

        self.parent.putChild(bytes(methodName, 'utf8'), self)

    def allowCrossOrigin(self, request):
        # Append headers to allow cross-origin requests.
        request.setHeader('Access-Control-Allow-Origin', '*')
        request.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTION')
        request.setHeader('Access-Control-Allow-Headers', 'Content-Type')
        request.setHeader('Access-Control-Max-Age', 2520)

    def render_OPTIONS(self, request):
        allowCrossOrigin(request)
        request.setHeader('Content-Type', 'application/json')
        request.write('')
        request.finish()
        return server.NOT_DONE_YET

    def render_GET(self, request):
        data = {}
        if self.arguments:
            logging.info('GET arguments: %s', request.args)
            for key in request.args:
                try:
                    data[key.decode('utf8')] = int(request.args[key][0])
                except:
                    if   request.args[key][0] == b'true':  data[key.decode('utf8')] = True
                    elif request.args[key][0] == b'false': data[key.decode('utf8')] = False
                    else:  data[key.decode('utf8')] = request.args[key][0].decode('utf8')

        allowCrossOrigin(request)
        request.setHeader('Content-Type', 'application/json')
        if self.arguments:
            reactor.callLater(0.0, self.startDbusRequestWData, request, data)
        else:
            reactor.callLater(0.0, self.startDbusRequest, request)
        return server.NOT_DONE_YET
        
    def render_POST(self, request):
        contentType = request.getHeader("Content-Type")
        self.allowCrossOrigin(request)
        if contentType == "application/json":
            # Expect a JSON object for the POST data.
            data = request.content.getvalue().decode("utf8")
            request.setHeader('Content-Type', 'application/json')
            if self.arguments:
                reactor.callLater(0.0, self.startDbusRequestWData, request, json.loads(data))
            else:
                reactor.callLater(0.0, self.startDbusRequest, request)
            return server.NOT_DONE_YET
        if self.arguments:
            # Expect JSON data passed in URL-encoded form.
            rawData = request.args.get(b'data', None)
            if not rawData:
                request.setResponseCode(400)
                return b'"data" field required'
            data = ''
            for line in rawData:
                data += line.decode('utf8')
            data = json.loads(data)

            request.setHeader('Content-Type', 'application/json')
            reactor.callLater(0.0, self.startDbusRequestWData, request, data)
            return server.NOT_DONE_YET
        else:
            reactor.callLater(0.0, self.startDbusRequest, request)
            return server.NOT_DONE_YET

    @inlineCallbacks
    def startDbusRequestWData(self, request, data):
        logging.debug('request: %s, (%s) %s', self.methodName, type(data), data)
        reply = yield self.bus.callRemote(self.methodName, data)

        returnData = json.dumps(reply)
        
        request.write(bytes('{0}\n'.format(returnData), 'utf8'))
        request.finish()

        logging.info('method "%s" requested with data: %s with response: %s', self.methodName, data, returnData)

    @inlineCallbacks
    def startDbusRequest(self, request):
        logging.debug('request: %s', self.methodName)
        reply = yield self.bus.callRemote(self.methodName)

        returnData = json.dumps(reply)
        
        request.write(bytes('{0}\n'.format(returnData), 'utf8'))
        request.finish()

        logging.info('method "%s" requested with response: %s', self.methodName, returnData)
        
class Subscribe(resource.Resource):
    """
    Implements the subscribe resource
    """
    isLeaf = True

    def __init__(self):
        self.subscribers = set()

    def render_GET(self, request):
        allowCrossOrigin(request, methods='GET, OPTION')
        request.setHeader('Content-Type', 'text/event-stream; charset=utf-8')
        request.setResponseCode(200)
        self.subscribers.add(request)
        d = request.notifyFinish()
        d.addBoth(self.removeSubscriber)
        logging.info("Adding subscriber...")
        request.write(b"")
        return server.NOT_DONE_YET

    def publishToAll(self, event, data):
        """
        Publish an event to all recipients.
        """
        for subscriber in self.subscribers:
            if event:
                subscriber.write(bytes('event: {event}\n'.format(event=event), 'utf8'))
            subscriber.write(bytes('data: {data}\n'.format(data=data), 'utf8'))
            # NOTE: the last CRLF is required to dispatch the event at the client
            subscriber.write(b"\n")

    def removeSubscriber(self, subscriber):
        if subscriber in self.subscribers:
            logging.info("Removing subscriber..")
            self.subscribers.remove(subscriber)


class Publish(resource.Resource):
    """
    Implements the publish resource
    """
    isLeaf = True

    def __init__(self, subscriber):
        self.subscriber = subscriber

    def render_POST(self, request):
        eventlines = request.args.get(b'event', [b''])
        event = ''
        for line in eventlines:
            event += line.decode('utf8')
        rawData  = request.args.get(b'data', None)
        if not rawData:
            request.setResponseCode(400)
            return b"The parameter 'data' must be set\n"
        data = ''
        for line in rawData:
            logging.info('line length: %d', len(line))
            logging.info(' utf8 length: %d', len(line.decode('utf8')))
            data += line.decode('utf8')
        self.subscriber.publishToAll(event, data)
        
        return bytes('event: {event}\ndata: {data}\n'.format(event=event, data=data), 'utf8')


class dbusPublisher:
    def __init__(self, subscriber, controlApi, videoApi, ringApi):
        self.subscriber = subscriber
        self.controlApi = controlApi
        self.videoApi = videoApi
        self.ringApi = ringApi

        self.controlApi.notifyOnSignal('notify', self.publishNotify)
        self.videoApi.notifyOnSignal('segment', self.publishSegment)
        self.videoApi.notifyOnSignal('sof', self.publishSOF)
        self.videoApi.notifyOnSignal('eof', self.publishEOF)

    def publishNotify(self, signal):
        event = 'control/notify'
        data = json.dumps(signal)
        logging.info('%s: %s', event, data)
        self.subscriber.publishToAll(event, data)

    def publishSegment(self, signal):
        event = 'video/segment'
        data = json.dumps(signal)
        logging.info('%s: %s', event, data)
        self.subscriber.publishToAll(event, data)

    def publishSOF(self, signal):
        event = 'video/segment'
        data = json.dumps(signal)
        logging.info('%s: %s', event, data)
        self.subscriber.publishToAll(event, data)

    def publishEOF(self, signal):
        event = 'video/segment'
        data = json.dumps(signal)
        logging.info('%s: %s', event, data)
        self.subscriber.publishToAll(event, data)


class previewImage(resource.Resource):
    isLeaf = True
    def __init__(self, bus):
        self.bus = bus
        super().__init__()

    def render_GET(self, request):
        allowCrossOrigin(request, methods='GET, OPTION')
        request.setHeader('Content-Type', 'image/jpeg')
        reactor.callLater(0.0, self.requestPlaybackImage, request)
        return server.NOT_DONE_YET

    @inlineCallbacks
    def requestPlaybackImage(self, request):
        reply = yield self.bus.callRemote('livedisplay', {'hres':0,'vres':0})
        yield asleep(2/60)

        image = open('/tmp/cam-screencap.jpg', 'br').read()
        request.write(image)
        request.finish()

class playbackImage(resource.Resource):
    isLeaf = True
    def __init__(self, bus):
        self.bus = bus
        super().__init__()
    
    def render_GET(self, request):
        allowCrossOrigin(request, methods='GET, OPTION')
        request.setHeader('Content-Type', 'image/jpeg')
        reactor.callLater(0.0, self.requestPlaybackImage, request)
        return server.NOT_DONE_YET

    @inlineCallbacks
    def requestPlaybackImage(self, request):
        frameNum = int(request.args.get(b'frameNum', (0))[0])
        reply = yield self.bus.callRemote('playback', {"framerate":0, "position":frameNum})
        yield asleep(2/60)

        image = open('/tmp/cam-screencap.jpg', 'br').read()
        request.write(image)
        request.finish()


class forceReboot(resource.Resource):
    isLeaf = True

    def render_OPTIONS(self, request):
        allowCrossOrigin(request, methods='GET, POST, OPTION')
        request.setHeader('Content-Type', 'application/json')
        request.finish()
        return server.NOT_DONE_YET

    def render_GET(self, request):
        allowCrossOrigin(request, methods='GET, POST, OPTION')
        request.setHeader('Content-Type', 'application/json')
        utils.getProcessOutput('/sbin/reboot', [])
        return b'{"done":true}'

    def render_POST(self, request):
        allowCrossOrigin(request, methods='GET, POST, OPTION')
        request.setHeader('Content-Type', 'application/json')
        utils.getProcessOutput('/sbin/reboot', [])
        return b'{"done":true}'


class waitForTouch(resource.Resource):
    isLeaf = True
    def render_GET(self, request):
        allowCrossOrigin(request, methods='GET, OPTION')
        reactor.callLater(0.0, self.runCommand, request)
        return server.NOT_DONE_YET

    @inlineCallbacks
    def runCommand(self, request):
        message = cgi.escape(request.args.get(b'message', (b"Tap"))[0].decode('utf8'))
        logging.info('started wait for touch')
        yield utils.getProcessOutput('python3', ['/root/api/uiScripts/tap-to-exit-button.py', message], env={"QT_QPA_PLATFORM":"linuxfb:fb=/dev/fb0"})
        logging.info('touch happened')
        request.write(b'{"Touched":true}')
        request.finish()

class waitForTouchThenBlackcal(resource.Resource):
    isLeaf = True
    def __init__(self, controlApi):
        self.controlApi = controlApi
        super().__init__()
    
    def render_GET(self, request):
        allowCrossOrigin(request, methods='GET, OPTION')
        reactor.callLater(0.0, self.runCommand, request)
        return server.NOT_DONE_YET
        
    @inlineCallbacks
    def runCommand(self, request):
        message = cgi.escape(request.args.get(b'message', (b"Touch to cal"))[0].decode('utf8'))
        logging.info('started wait for touch')
        yield utils.getProcessOutput('python3', ['/root/api/uiScripts/tap-to-exit-button.py', message], env={"QT_QPA_PLATFORM":"linuxfb:fb=/dev/fb0"})
        logging.info('touch happened; calibrating')

        reply = yield self.controlApi.callRemote('calibrate', {"blackCal":True, "analogCal":True})
        returnData = json.dumps(reply)
        request.write(bytes('{0}\n'.format(returnData), 'utf8'))
        request.finish()
        
@inlineCallbacks
def main():
    subscribe = Subscribe()
    webPublisher = Publish(subscribe)
    root.putChild(b'subscribe', subscribe)
    root.putChild(b'publish', webPublisher)
    root.putChild(b'', root)
    site = server.Site(root)

    logging.info('Adding dbus signals')
    system = yield client.connect(reactor, 'system')
    controlApi = yield system.getRemoteObject('com.krontech.chronos.control', '/com/krontech/chronos/control')
    videoApi   = yield system.getRemoteObject('com.krontech.chronos.video',   '/com/krontech/chronos/video')
    ringApi    = None

    dbusSignalPublisher = dbusPublisher(subscribe, controlApi, videoApi, ringApi)

    logging.info('Adding methods')
    control = resource.Resource()
    root.putChild(b'control', control)
    GetMethod(control, controlApi, 'get')
    Method(control, controlApi, 'set',                    arguments=True)
    
    Method(control, controlApi, 'startAutoWhiteBalance',  arguments=False)
    Method(control, controlApi, 'revertAutoWhiteBalance', arguments=False)
    Method(control, controlApi, 'startAutoFocus',         arguments=True)
    Method(control, controlApi, 'startCalibration',       arguments=True)
    Method(control, controlApi, 'startRecording',         arguments=False)
    Method(control, controlApi, 'stopRecording',          arguments=False)
    Method(control, controlApi, 'flushRecording',         arguments=False)
    Method(control, controlApi, 'startFilesave',          arguments=False)
    Method(control, controlApi, 'softTrigger',            arguments=False)
    Method(control, controlApi, 'revertToDefaults',       arguments=False)
    Method(control, controlApi, 'softReset',              arguments=False)
    Method(control, controlApi, 'testResolution',         arguments=True)

    video = resource.Resource()
    root.putChild(b'video', video)
    Method(video, videoApi, 'status',      arguments=False)
    Method(video, videoApi, 'flush',       arguments=False)
    Method(video, videoApi, 'playback',    arguments=True)
    Method(video, videoApi, 'configure',   arguments=True)
    Method(video, videoApi, 'livedisplay', arguments=True)
    Method(video, videoApi, 'recordfile',  arguments=True)
    Method(video, videoApi, 'stop',        arguments=False)
    Method(video, videoApi, 'overlay',     arguments=True)

    root.putChild(b'screenCap.jpg', previewImage(videoApi))
    root.putChild(b'liveImage.jpg', previewImage(videoApi))
    root.putChild(b'playbackImage.jpg', playbackImage(videoApi))

    root.putChild(b'waitForTouch', waitForTouch())
    root.putChild(b'waitForTouchThenBlackcal', waitForTouchThenBlackcal(controlApi))

    if aimCameraResource:
        root.putChild(b'aimCamera', aimCameraResource())

    root.putChild(b'timingControl.html', File('timingControl.html'))

    root.putChild(b'forceReboot', forceReboot())

    reactor.listenTCP(12000, site)
    yield asleep(0.1)
    
    logging.info("All Systems Go")
    
if __name__ == "__main__":
    root = Root()
    logging.basicConfig(level=logging.DEBUG, format='%(levelname)s [%(funcName)s] %(message)s')
    reactor.callWhenRunning( main )

    log.startLogging(sys.stdout)
    reactor.run()
