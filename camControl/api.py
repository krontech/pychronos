#!/usr/bin/python3
import os, sys, pdb
import json
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
from pychronos.sensors import frameGeometry
import pychronos.regmaps as regmaps

from pychronos.utils import getBoardRevision

# The D-Bus interface we are exporting.
interface = 'ca.krontech.chronos.control'

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

        # Try creating the calibration directory if it doesn't exist.
        try:
            os.makedirs(self.calLocation, exist_ok=True)
        except OSError as e:
            logging.info("Unable to create calibration directory at %s: %s", self.calLocation, e)

        # Install a callback to catch parameter and state changes.
        self.camera.setOnChange(self.onChangeHandler)
        self.changeset = None
        self.changecfg = False

        # Args to pass on down to the reboot logic.
        self.rebootMode = {}

        # Install a callback to catch video signals.
        self.video.connect_to_signal('sof', self.videoSofSignal)
        self.video.connect_to_signal('eof', self.videoEofSignal)
        self.video.connect_to_signal('segment', self.videoSegmentSignal)
        self.video.connect_to_signal('update', self.videoUpdateSignal)

        self.callLater(0.5, self.softReset)

    ## Internal helper to iterate over a generator from the GLib mainloop.
    def stepGenerator(self, generator, name):
        try:
            delay = next(generator)
            GLib.timeout_add(int(delay * 1000), self.stepGenerator, generator, name)
        except StopIteration:
            if (name):
                self.complete({"state": self.camera.state, "method": name})
        except Exception as error:
            logging.error(error)
            if (name):
                self.complete({
                    "state": self.camera.state,
                    "method": name,
                    "error": type(error).__name__,
                    "message": str(error)
                })
            if not isinstance(error, CameraError):
                logging.debug(traceback.format_exc())

        # Always return false to remove the Glib source for the last step.
        return False
    
    ## Internal helper to run a generator. This will make the first call to the
    ## generator, and throw an exception if an error occurs.
    def runGenerator(self, generator, name=None):
        try:
            delay = next(generator)
            GLib.timeout_add(int(delay * 1000), self.stepGenerator, generator, name)
        except StopIteration:
            if (name):
                self.complete({"state": self.camera.state, "method": name})

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
    
    ## Internal helper to merge multiple dictionaries together into D-Bus
    ## compatible types.
    def dbusifyMerge(self, *args, variant_level=0):
        result = dbus.types.Dictionary(variant_level=variant_level, signature='sv')
        for src in args:
            for key in src:
                value = src[key]
                if isinstance(value, bool) or isinstance(value, dbus.types.Boolean):
                    result[key] = dbus.types.Boolean(value, variant_level=1)
                elif isinstance(value, int):
                    result[key] = dbus.types.Int32(value, variant_level=1)
                elif isinstance(value, float):
                    result[key] = dbus.types.Double(value, variant_level=1)
                elif isinstance(value, (dict, list)):
                    result[key] = self.dbusifyTypes(value, variant_level=1)
                else:
                    result[key] = dbus.types.String(value, variant_level=1)

        return result

    ## Internal helper to convert types into D-Bus compatible versions. For
    ## now this only applies to dictionaries, where the key/value pairs are
    ## converted into an 'a{sv}' signature.
    def dbusifyTypes(self, src, variant_level=0):
        if isinstance(src, dict):
            # For dicts, convert all values into variants.
            return self.dbusifyMerge(src, variant_level=variant_level)
        else:
            # For everything other than dicts - do nothing.
            return src
    
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
   
    @dbus.service.signal(interface, signature='a{sv}')
    def complete(self, args):
        return args
 
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

        filename = args.pop('filename', time.strftime('vid_%F_%H-%M-%S'))
        filename += suffixes.get(saveFormat, '')

        # Assemble the full pathname for recorded file.
        extStorage = self.camera.externalStorage
        if devName not in extStorage:
            raise ValueError('Invalid storage device given for recording')
        storage = extStorage[devName]
    #    print("### filename =", filename)
    #    print("### storage =", storage)
    #    print("### mount =",storage['mount'])
    #    print("### join =", os.path.join(storage['mount'], filename))
        filepath = os.path.abspath(os.path.join(storage['mount'], filename))
        if not filepath.startswith(storage['mount']):
            raise ValueError('Invalid filename given for recording')        
    #    print ("### filepath =", filepath)
        # TODO: Check for available space or any other sanity checking.
        # TODO: Start a saveDoneTimer to monitor available storage space?
        # TODO: Maybe launch the saveDoneTimer from the SOF/EOF signal.
        # TODO: Look for sneaky path tricks, like /../..

        # Make the real call to save the file and pass through the remaining arguments.
        args['filename'] = filepath
        self.video.recordfile(args, reply_handler=onReply, error_handler=onError)
    
    #===============================================================================================
    #Method('get', arguments='as', returns='a{sv}')
    #Method('set', arguments='a{sv}', returns='a{sv}')
    def onGetReply(self, a, b, onReply=None):
        """Internal helper to handle an async reply when proxying a get/set to the video API."""
        if onReply:
            onReply(self.dbusifyMerge(a, b))
    
    def onGetError(self, results, keys, err, onReply):
        """Internal helper to handle an async error when proxying a get/set to the video API."""
        x = {}
        if 'error' in results:
            x.update(results['error'])
        for name in keys:
            x[name] = str(err)
        
        if onReply:
            onReply(self.dbusifyMerge(results, {'error': x}))

    @dbus.service.method(interface, in_signature='as', out_signature='a{sv}', async_callbacks=('onReply', 'onError'))
    def get(self, attrs, onReply=None, onError=None): #Use "attrs", not "args", because too close to "*args".
        """Retrieve named values from the control API and the video API."""
        
        # Get any parameters we can handle locally.
        results = dbus.types.Dictionary(signature='sv')
        notfound = dbus.types.Array(signature='s')
        for name in attrs:
            try:
                results[name] = self.dbusifyTypes(getattr(self.camera, name))
            except AttributeError as e:
                notfound.append(str(name))
        
        if notfound:
            # If there were not-found parameters, check the video API.
            self.video.get(notfound, timeout=150,
                reply_handler=lambda vresults: self.onGetReply(results, vresults, onReply),
                error_handler=lambda err: self.onGetError(results, notfound, err, onReply))
        elif onReply:
            # Otherwise, we can reply immediately.
            onReply(results)
    
    def paramsort(self, name):
        """Internal helper function to sort a parameter by priority"""
        try:
            camprop = getattr(type(self.camera), name)
            return camprop.fget.prio
        except:
            return 0
    
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}',  async_callbacks=('onReply', 'onError'))
    def set(self, newValues, onReply=None, onError=None):
        """Set named values in the control API and the video API."""
        
        keys = sorted(newValues.keys(), key=self.paramsort, reverse=True)
        failedAttributes = {}
        videoAttributes = {}
        localKeys = []
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
                    localKeys.append(name)
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
        if resChange or (startCal != self.camera.sensor.calFilename("test", ".bin")):
            self.camera.loadCalibration()
        
        # Get the results of the local attributes.
        results = {}
        for name in localKeys:
            try:
                results[name] = self.dbusifyTypes(getattr(self.camera, name))
            except Exception as e:
                failedAttributes[name] = str(e)
        if failedAttributes:
            results['error'] = self.dbusifyTypes(failedAttributes)
        
        if videoAttributes:
            # If there were keys that didn't exist, try setting them in the video API.
            self.video.set(self.dbusifyTypes(videoAttributes), timeout=150,
                reply_handler=lambda vresults: self.onGetReply(results, vresults, onReply),
                error_handler=lambda err: self.onGetError(results, videoAttributes.keys(), err, onReply))
        elif onReply:
            # Otherwise, we can return immediately.
            onReply(results)
    
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
        
        try:
            videoKeys = self.video.describe(timeout=150)
        except dbus.exceptions.DBusException:
            logging.error('could not load video available keys')
            videoKeys = {}
        
        return self.dbusifyMerge(self.ownKeys(), videoKeys)
    
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
    #Method('reboot', arguments='a{sv}', returns='a{sv}')
    #Method('softReset', arguments='', returns='a{sv}')
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def reboot(self, args):
        """ Restart the control API and/or the camera.
            
            Takes a dict of reboot options, for example:
            
                {
                    'settings': True,
                    'power': False,
                    'reload': True,
                }
            
            When 'settings' is true, the user settings are removed during the reboot,
            which should return the camera to its factory default state.

            When 'power' is true, the camera will perform a full power cycle.

            When 'reload' is true (the default), the control API will restart itself.
        """
        try:
            self.runGenerator(self.camera.abort())
            self.rebootMode = args
            self.mainloop.quit()
            return {
                "state": self.camera.state
            }
        except CameraError as e:
            return {
                "state": self.camera.state,
                "error": type(e).__name__,
                "message": str(e)
            }

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
                "error": type(e).__name__,
                "message": str(e)
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
            self.runGenerator(self.camera.startCalibration(**args), "startCalibration")
            return {
                "state": self.camera.state
            }
        except CameraError as e:
            return {
                "state": self.camera.state,
                "error": type(e).__name__,
                "message": str(e)
            }
    
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def clearCalibration(self, args):
        """ Remove calibration data, returning to factory calibration. """
        try:
            self.camera.clearCalibration(**args)
            return {
                "state": self.camera.state
            }
        except CameraError as e:
            return {
                "state": self.camera.state,
                "error": type(e).__name__,
                "message": str(e)
            }
    
    #===============================================================================================
    #Method('exportCalData', arguments='a{sv}', returns='a{sv}'),
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def exportCalData(self, args):
        """Export flat-field frames in numpy format to USB thumb drive."""
        try:
            self.runGenerator(self.camera.exportCalData(**args))
            return {
                "state": self.camera.state
            }
        except CameraError as e:
            return {
                "state": self.camera.state,
                "error": type(e).__name__,
                "message": str(e)
            }

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def importCalData(self, args):
        """ Import calibration data that was generated off-camera. """
        try:
            self.camera.importCalData(**args)
            return {
                "state": self.camera.state
            }
        except CameraError as e:
            return {
                "state": self.camera.state,
                "error": type(e).__name__,
                "message": str(e)
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
            self.runGenerator(self.camera.startWhiteBalance(**args), "startWhiteBalance")
            return {
                "state": self.camera.state
            }
        except CameraError as e:
            return {
                "state": self.camera.state,
                "error": type(e).__name__,
                "message": str(e)
            }

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def revertAutoWhiteBalance(self, args):
        """This copies the contents of `wbCustomColor` into `wbColor`."""
        self.camera.wbColor = self.camera.wbCustomColor
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
                "error": type(e).__name__,
                "message": str(e)
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
                "error": type(e).__name__,
                "message": str(e)
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
            expMin, expMax = self.camera.sensor.getExposureRange(fSize)
            return {
                "cameraMaxFrames": self.camera.getRecordingMaxFrames(fSize),
                "minFramePeriod": int(fpMin * 1000000000),
                "exposureMin": int(expMin * 1000000000),
                "exposureMax": int(expMax * 1000000000)
            }
        else:
            return {
                "state": self.camera.state,
                "error": "ValueError",
                "message": "Invalid Resolution"
            }
