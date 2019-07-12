#!/usr/bin/python3

import os, sys, pdb
import json
import argparse
from functools import lru_cache

import inspect
import logging
import traceback
import time

import dbus
import dbus.service
import dbus.mainloop.glib
from gi.repository import GLib

from pychronos import camera
from pychronos.error import *
from pychronos.sensors import lux1310, frameGeometry
import pychronos.regmaps as regmaps

interface = 'ca.krontech.chronos.control'
dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
bus = dbus.SystemBus()

#-----------------------------------------------------------------
# Some constants that ought to go into a board-specific dict.
FPGA_BITSTREAM = "/var/camera/FPGA.bit"
GPIO_ENCA = "/sys/class/gpio/gpio20/value"
GPIO_ENCB = "/sys/class/gpio/gpio26/value"
GPIO_ENCSW = "/sys/class/gpio/gpio27/value"
REC_LED_FRONT = "/sys/class/gpio/gpio41/value"
REC_LED_BACK = "/sys/class/gpio/gpio25/value"
#-----------------------------------------------------------------

class controlApi(dbus.service.Object):
    ## This feels like a duplication of the Python exceptions.
    ERROR_NOT_IMPLEMENTED_YET = 9999
    VALUE_ERROR               = 1
    
    def __init__(self, bus, path, mainloop, camera, configFile=None):
        # FIXME: This seems hacky, just calling the class method directly.
        # Shouldn't we be using a super() call somehow?
        dbus.service.Object.__init__(self, bus, path)
        self.bus = bus
        self.mainloop = mainloop
        self.configFile = configFile
        self.calLocation = "/var/camera/cal"
    
        self.camera = camera
        self.video = bus.get_object('ca.krontech.chronos.video', '/ca/krontech/chronos/video')
        self.io = regmaps.ioInterface()
        self.display = regmaps.display()

        # Install a callback to catch parameter and state changes.
        self.camera.setOnChange(self.onChangeHandler)
        self.changeset = None
        self.changecfg = False

        # Install a callback to catch video signals.
        self.video.connect_to_signal('sof', self.videoSofSignal)
        self.video.connect_to_signal('eof', self.videoEofSignal)
        self.video.connect_to_signal('segment', self.videoSegmentSignal)
        self.video.connect_to_signal('update', self.videoUpdateSignal)

        self.callLater(0.5, self.softReset)

    ## Internal helper to iterate over a generator from the GLib mainloop.
    def stepGenerator(self, generator):
        try:
            delay = next(generator)
            GLib.timeout_add(int(delay * 1000), self.stepGenerator, generator)
        except StopIteration:
            pass
        except CameraError as error:
            logging.error(error)
            self.notify({"state": self.camera.state, "error": str(error)})
        except Exception as error:
            logging.error(error)
            logging.debug(traceback.format_exc())

        # Always return false to remove the Glib source for the last step.
        return False
    
    ## Internal helper to run a generator. This will make the first call to the
    ## generator, and throw an exception if an error occurs.
    def runGenerator(self, generator):
        try:
            delay = next(generator)
            GLib.timeout_add(int(delay * 1000), self.stepGenerator, generator)
        except CameraError as error:
            logging.error(error)
            self.notify({"state": self.camera.state, "error": str(error)})
        except StopIteration:
            pass

    ## Internal helper to call something in the future. Should function identically
    ## to Twisted's reactor.callLater function, except that the yield asleep thing
    ## is just replaced with a yield of the desired sleep time.
    def callLater(self, timeout, callback, *args, **kwargs):
        msec = int(timeout * 1000)
        if (inspect.isgeneratorfunction(callback)):
            ## The callback will yeild. Use stepGenerator instead to handle it.
            GLib.timeout_add(msec, self.stepGenerator, callback(*args, **kwargs))
        else:
            # Just a plain old function, call directly but force it to return False
            GLib.timeout_add(msec, lambda *args, **kwargs: callback(*args, **kwargs) and False, *args, **kwargs)
    
    ## Internal helper to convert types into D-Bus compatible versions. For
    ## now this only applies to dictionaries, where the key/value pairs are
    ## converted into an 'a{sv}' signature.
    def dbusifyTypes(self, src, variant_level=0):
        # For everything other than dicts - do nothing.
        if not isinstance(src, dict):
            return src
        
        # For dicts, convert all values into variants.
        result = dbus.types.Dictionary(variant_level=variant_level, signature='sv')
        for key in src:
            value = src[key]
            if isinstance(value, int):
                result[key] = dbus.types.Int32(value, variant_level=1)
            elif isinstance(value, float):
                result[key] = dbus.types.Double(value, variant_level=1)
            elif isinstance(value, (dict, list)):
                result[key] = self.dbusifyTypes(value, variant_level=1)
            else:
                result[key] = dbus.types.String(value, variant_level=1)
        return result
    
    def dbusReplyHandler(self, args):
        logging.debug("D-Bus reply: %s", args)
    
    def dbusErrorHandler(self, err):
        logging.error("D-Bus error: %s", err)
    
    #===============================================================================================
    #Method('notify', arguments='', returns='a{sv}'),
    def onChangeHandler(self, pName, pValue):
        logging.debug("Parameter %s -> %s", pName, pValue)
        if not self.changeset:
            self.changeset = {pName: self.dbusifyTypes(pValue)}
            GLib.timeout_add(100, self.notifyChanges)
        else:
            self.changeset[pName] = self.dbusifyTypes(pValue)
        
        # Check if this is a saved property.
        prop = getattr(type(self.camera), pName, None)
        if prop is not None and isinstance(prop, property) and getattr(prop.fget, 'saveable', False):
            self.changecfg = True

    def notifyChanges(self):
        # Generate the DBus notify signal.
        self.notify(self.changeset)
        self.changeset = None
        
        # Save configuration changes to disk.
        if (self.changecfg and self.configFile):
            camType = type(self.camera)
            savableConfig = {
                key: value
                for key, value in self.camera.config.items()
                if isinstance(getattr(camType, key), property)
                and getattr(getattr(camType, key).fget, 'saveable', False) #Only save values marked as savable.
                and not getattr(getattr(camType, key).fget, 'derivedFrom', False) #Don't save any derived values.
            }
            with open(self.configFile, 'w') as outFile:
                json.dump(savableConfig, outFile, sort_keys=True, indent=4, separators=(',', ': '))
            self.changecfg = False
        
        return False
    
    @dbus.service.signal(interface, signature='a{sv}')
    def notify(self, args):
        return self.changeset
    
    #===============================================================================================
    # Video Proxy Signal Handling

    def videoSofSignal(self, args):
        logging.debug("Received SOF -> %s", args)
    
    def videoEofSignal(self, args):
        logging.debug("Received EOF -> %s", args)
    
    def videoSegmentSignal(self, args):
        logging.debug("Received segment -> %s", args)

    def videoUpdateSignal(self, args):
        # Pass parameter updates along as though they came from the control API.
        for key, value in args.items():
            self.onChangeHandler(key, value)
    
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}', async_callbacks=('onReply', 'onError'))
    def startFilesave(self, args, onReply=None, onError=None):
        """TBD: A proxy for the `filesave` method in the Video API."""
        # Filename suffixes for the formats that need one.
        suffixes = {
            'h264': '.mp4',
            'x264': '.mp4',
            'byr2': '.raw',
            'y16':  '.raw',
            'y12b': '.raw'
        }
        saveFormat = args['format']

        # Remove some arguments for sanitiziation.
        devName = args.pop('device')
        filename = args.pop('filename', time.strftime('vid_%F_%H-%M-%S')) + suffixes.get(saveFormat)

        # Assemble the full pathname for recorded file.
        extStorate = self.camera.externalStorage
        if devName not in extStorage:
            raise ValueError('Invalid storage device given for recording')
        storage = extStorate[devName]
        filepath = os.path.abspath(os.path.join(storage['mount'], filename))
        if not filepath.startswith(storage['mount']):
            raise ValueError('Invalid filename given for recording')
        # TODO: Check for available space or any other sanity checking.
        # TODO: Start a saveDoneTimer to monitor available storage space?
        # TODO: Maybe launch the saveDoneTimer from the SOF/EOF signal.

        # Make the real call to save the file and pass through the remaining arguments.
        args['filename'] = filepath
        video.recordfile(args, reply_handler=onReply, error_handler=onError)
    
    #===============================================================================================
    #Method('get', arguments='as', returns='a{sv}')
    #Method('set', arguments='a{sv}', returns='a{sv}')
    @dbus.service.method(interface, in_signature='as', out_signature='a{sv}')
    def get(self, attrs): #Use "attrs", not "args", because too close to "*args".
        """Retrieve named values from the control API and the video API."""
        
        try:
            controlAttrs = [
                name
                for name in attrs 
                if name in self.ownKeys()
            ]
            videoAttrs = [
                str(name)
                for name in attrs 
                if name not in controlAttrs
            ]
            
            data = self.video.get(videoAttrs, timeout=150) if videoAttrs else {}
            for name in controlAttrs:
                data[name] = self.dbusifyTypes(getattr(self.camera, name))
            return data
        
        except dbus.exceptions.DBusException as e:
            if e.get_dbus_name() != 'ca.krontech.chronos.UnknownAttribute':
                raise e
            else:
                #Video didn't have the attr, we don't have the attr, return a nice error message saying _which_ attr is causing issues and why.
                raise dbus.exceptions.DBusException(
                    'ca.krontech.chronos.UnknownAttribute',
                    "'%s' is not a known attribute. Known attributes are: %s" % (
                        e.get_dbus_message().split()[0], #attribute name
                        sorted(set(self.availableKeys()))
                    )
                )
    
    def paramsort(self, name):
        """Internal helper function to sort a parameter by priority"""
        try:
            camprop = getattr(type(self.camera), name)
            return camprop.fget.prio
        except:
            return 0
    
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def set(self, newValues):
        """Set named values in the control API and the video API."""
        
        keys = sorted(newValues.keys(), key=self.paramsort, reverse=True)
        failedAttributes = {}
        videoAttributes = {}
        startCal = self.camera.sensor.calFilename("test", ".bin")
        resChange = False
        for name in keys:
            # If the property exists in the camera class, set it.
            value = newValues[name]
            logging.debug("Setting %s -> %s", name, value)
            camprop = getattr(type(self.camera), name, None)
            if isinstance(camprop, property):
                try:
                    setattr(self.camera, name, value)
                    logging.debug("Set %s -> %s", name, value)
                    if (name == 'resolution'):
                        resChange = True
                except Exception as e:
                    logging.debug("Setting %s failed: %s", name, e)
                    logging.debug(traceback.format_exc())
                    failedAttributes[name] = str(e)
            # Otherwise, try setting the property in the video interface.
            elif name in self.availableKeys():
                videoAttributes[name] = value
            else:
                failedAttributes[name] = "Unknown property '%s' could not be set." % (name)
        
        # If the calibration file name has changed, a new calibration will
        # be required. Check if stored calibration data exists, and load it.
        # Otherwise, we should load some sensible defaults and perform a zero
        # time black cal.
        if resChange or (startCal != self.camera.sensor.calFilename("test", ".bin")):
            if not self.camera.loadCalibration():
                logging.warning('Sensor configuration chage requires new calibration')
                try:
                    self.runGenerator(self.camera.startCalibration(analogCal=True, zeroTimeBlackCal=True, saveCal=False))
                finally:
                    pass

        # For any keys that don't exist - try setting them in the video API.
        # TODO: We should catch the async response and pass errors back to the
        # caller, but for now we just assume that everything succeeded.
        if videoAttributes:
            self.video.set(self.dbusifyTypes(videoAttributes),
                reply_handler=self.dbusReplyHandler,
                error_handler=self.dbusErrorHandler,
                timeout=150)
        
        #HACK: Manually poke the video pipeline back into live display after changing
        # the display resolution. This should eventually go away by making the video
        # system more autonomous with regards to the live display res.
        if ('resolution' in newValues):
            res = self.camera.resolution
            logging.info('Notifying cam-pipeline to reconfigure display')
            self.video.livedisplay({
                'hres':dbus.types.Int32(res['hRes'], variant_level=1),
                'vres':dbus.types.Int32(res['vRes'], variant_level=1)
            }, reply_handler=self.dbusReplyHandler, error_handler=self.dbusErrorHandler, timeout=150)
        
        # Return the settings as they've been applied.
        result = self.get(keys)
        if failedAttributes:
            result['error'] = self.dbusifyTypes(failedAttributes)
        return result
    
    #===============================================================================================
    #Method('availableKeys', arguments='', returns='as')
    #Method('availableCalls', arguments='', returns='as')
    @lru_cache(maxsize=1)
    def ownKeys(self):
        return {
            elem: {
                'get': getattr(type(self.camera), elem).fget is not None,
                'set': getattr(type(self.camera), elem).fset is not None,
                'notifies': getattr(getattr(type(self.camera), elem).fget, 'notifies', False), #set with camProperty
                'derivedFrom': getattr(getattr(type(self.camera), elem).fget, 'derivedFrom', False), #set with camProperty
            }
            for elem in dir(self.camera)
            if elem[0] != '_'
            and isinstance(getattr(type(self.camera), elem, None), property) #is a getter, maybe a setter
        }
    
    #===============================================================================================
    #Method('availableKeys', arguments='', returns='as')
    #Method('availableCalls', arguments='', returns='as')
    @lru_cache(maxsize=1)
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def availableKeys(self):
        """Get a list of the properties we can get/set/subscribe.
            
            For a list of functions, see org.freedesktop.DBus.Properties.GetAll."""
        
        #Video API doesn't know which keys it owns; hack it in here for now. (DDR 2019-05-06)
        try:
            videoKeys = self.video.availableKeys(timeout=150)
        except dbus.exceptions.DBusException:
            logging.error('could not load video available keys')
            videoKeys = {
                'videoState': {'get': True, 'set': False, 'notifies': True},
                'overlayEnable': {'get': True, 'set': True, 'notifies': True},
                'overlayFormat': {'get': True, 'set': True, 'notifies': True},
                'focusPeakingColor': {'get': True, 'set': True, 'notifies': True},
                'focusPeakingLevel': {'get': True, 'set': True, 'notifies': True},
                'zebraLevel': {'get': True, 'set': True, 'notifies': True},
                'playbackRate': {'get': True, 'set': True, 'notifies': True},
                'playbackPosition': {'get': True, 'set': True, 'notifies': False},
                'playbackStart': {'get': True, 'set': True, 'notifies': True},
                'playbackLength': {'get': True, 'set': True, 'notifies': True},
                'totalFrames': {'get': True, 'set': False, 'notifies': True},
                'totalSegments': {'get': True, 'set': False, 'notifies': True},
            }
        
        keys = videoKeys
        keys.update(self.ownKeys())
        return self.dbusifyTypes(keys)
    
    
    #===============================================================================================
    #Method('status', arguments='', returns='a{sv}')
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def status(self):
        """Get the current :meth:`~pychronos.camera.state` of the camera.
        
            Return value::
            
                {
                    'state': pychronos.camera.state
                }
        """
        return {'state':self.camera.state}

    #===============================================================================================
    #Method('softReset', arguments='', returns='a{sv}'),
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def softReset(self):
        """Perform a soft reset and initialization of the FPGA and image sensor."""
        try:
            self.runGenerator(self.runSoftReset())
            return {
                "state": self.camera.state
            }
        except CameraError as e:
            return {
                "state": self.camera.state,
                "error": str(e)
            }
    
    def loadConfig(self):
        if not self.configFile:
            return
        
        try:
            logging.info('checking for config file: %s', self.configFile)
            with open(self.configFile, 'r') as inFile:
                self.set(json.load(inFile))
        except FileNotFoundError:
            logging.info('config file not found')

    def runSoftReset(self):
        # Wrapper method to do a complete soft reset of both the camera class,
        # reloading of parameters, and tickling of the video system.
        yield from self.camera.softReset()

        # Reload the configuration.
        self.loadConfig()

        # Reload, or generate initial calibration data.
        if not self.camera.loadCalibration():
            yield from self.camera.startCalibration(analogCal=True, zeroTimeBlackCal=True, saveCal=False)

        # Re-configure the video system back into live display.
        res = self.camera.resolution
        logging.info('Notifying cam-pipeline to reconfigure display')
        self.video.livedisplay({
            'hres':dbus.types.Int32(res['hRes'], variant_level=1),
            'vres':dbus.types.Int32(res['vRes'], variant_level=1)
        }, reply_handler=self.dbusReplyHandler, error_handler=self.dbusErrorHandler, timeout=150)

    #===============================================================================================
    #Method('startCalibration', arguments='a{sv}', returns='a{sv}'),
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def startCalibration(self, args):
        """ Perform full calibration operations.
            
            Takes a dict of calibration names set to True to run. For example::
            
                {
                    'blackCal': True,
                    'analogCal': True,
                    'zeroTimeBlackCal': False,
                }
            
            This performs a full black calibration and analog calibration, but
            does not run the zero time black calibration routine. If a calibration
            is not named, it is treated as if it were False.
        """
        try:
            self.runGenerator(self.camera.startCalibration(**args))
            return {
                "state": self.camera.state
            }
        except CameraError as e:
            return {
                "state": self.camera.state,
                "error": str(e)
            }
    
    #===============================================================================================
    #Method('startAutoWhiteBalance', arguments='a{sv}', returns='a{sv}'),
    #Method('revertAutoWhiteBalance', arguments='a{sv}', regutns='a{sv}'),
    #Method('startRecording', arguments='', regutns='a{sv}'),
    #Method('stopRecording', arguments='', regutns='a{sv}'),
    #Method('flushRecording', arguments='', regutns='a{sv}'),
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def startAutoWhiteBalance(self, args):
        """Take a reference image from the live display and compute the white balance."""
        logging.info('starting white balance')
        try:
            self.runGenerator(self.camera.startWhiteBalance(**args))
            return {
                "state": self.camera.state
            }
        except CameraError as e:
            return {
                "state": self.camera.state,
                "error": str(e)
            }

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def revertAutoWhiteBalance(self, args):
        """This copies the contents of `wbCustom` into `wbMatrix`."""
        self.camera.wbMatrix = self.camera.wbCustom
        return {
            "state": self.camera.state
        }
    
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def startRecording(self):
        """Begin recording video data to memory."""
        try:
            self.runGenerator(self.camera.startRecording())
            return {
                "state": self.camera.state
            }
        except CameraError as e:
            return {
                "state": self.camera.state,
                "error": str(e)
            }
    
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def stopRecording(self):
        """Terimnate recording of video data to memory."""
        try:
            self.camera.stopRecording()
            return {
                "state": self.camera.state
            }
        except CameraError as e:
            return {
                "state": self.camera.state,
                "error": str(e)
            }
    
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}', async_callbacks=('onReply', 'onError'))
    def flushRecording(self, onReply=None, onError=None):
        """Flush recoreded video data from memory."""
        self.video.flush(reply_handler=onReply, error_handler=onError, timeout=150)

    #===============================================================================================
    #Method('getResolutionTimingLimits', arguments='a{sv}', returns='a{sv}'),
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def getResolutionTimingLimits(self, args):
        """Return the timing limits (framerate) at a resolution.
            
            Returns an error if the resolution is invalid.
            
            Example shell invocation::
            
                $ call --system \\
                    --dest ca.krontech.chronos.control \\
                    --object-path /ca/krontech/chronos/control \\
                    --method ca.krontech.chronos.control.getResolutionTimingLimits \\
                    "{'hRes': <1280>, 'vRes': <1020>}"
                ({'minFramePeriod': <931277>, 'exposureMin': <1000>, 'cameraMax
                Frames': <17542>, 'exposureMax': <925722>},)
            
            .. note::
                Maximum framerate, in fps, is ``1e9 / minFramePeriod``.
        """
        fSize = frameGeometry(**args)
        if (self.camera.sensor.isValidResolution(fSize)):
            fpMin, fpMax = self.camera.sensor.getPeriodRange(fSize)
            expMin, expMax = self.camera.sensor.getExposureRange(fSize, fpMin)
            return {
                "cameraMaxFrames": self.camera.getRecordingMaxFrames(fSize),
                "minFramePeriod": int(fpMin * 1000000000),
                "exposureMin": int(expMin * 1000000000),
                "exposureMax": int(expMax * 1000000000)
            }
        else:
            return {
                "error": "Invalid Resolution"
            }

class powerTimer:
    def __init__(self, camera):
        self.camera = camera

    def __call__(self, *args):
        self.camera.power.checkPowerSocket()
        if self.camera.power.lastAcAdaptorPresent != self.camera.power.acAdaptorPresent:
            self.camera.externalPowerChanged()
        return True

# Run the control API
if __name__ == "__main__":
    # Enable logging.
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s [%(funcName)s] %(message)s')

    # Do argument parsing
    parser = argparse.ArgumentParser(description="Chronos control daemon")
    parser.add_argument('--config', metavar='FILE', action='store',
                        default='/var/camera/apiConfig.json',
                        help="Configuration file path")
    parser.add_argument('--debug', default=False, action='store_true',
                        help="Enable debug logging")
    parser.add_argument('--pdb', default=False, action='store_true',
                        help="Drop into a python debug console on exception")
    args = parser.parse_args()

    if not args.debug:
        logging.getLogger().setLevel(logging.INFO)
    else:
        logging.getLogger().setLevel(logging.DEBUG)

    # Install exception handlers for interactive debug on exception.
    if args.pdb:
        def excepthook(t,v,tb):
            pdb.traceback.print_exception(t, v, tb)
            pdb.post_mortem(t=tb)
        sys.excepthook = excepthook
        dbg, brk = pdb.set_trace, pdb.set_trace #convenience debugging

        #Fix system not echoing keystrokes in debugger, after restart.
        try:
            os.system('stty sane')
        except Exception:
            pass

    # Use the GLib mainloop.    
    mainloop = GLib.MainLoop()

    # Setup resources.
    cam  = camera(lux1310())

    # Install a timer for battery data monitoring
    timer = powerTimer(cam)
    GLib.timeout_add(1000, timer)
   
    name = dbus.service.BusName('ca.krontech.chronos.control', bus=bus)
    obj  = controlApi(bus, '/ca/krontech/chronos/control', mainloop, cam, configFile=args.config)

    # Run the mainloop.
    logging.info("Running control service...")
    mainloop.run()
