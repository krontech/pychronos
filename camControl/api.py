#!/usr/bin/python3
import os, sys, pdb
import json
from functools import lru_cache
from enum import Enum

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

import camControl.gdocstring as gdocstring

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
                elif isinstance(value, Enum):
                    result[key] = dbus.types.String(value.name, variant_level=1)
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
        elif isinstance(src, Enum):
            # For enumerated types, convert values into strings.
            return src.name
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
            configdata = self.camera.config
            with open(self.configFile, 'w') as outFile:
                json.dump(self.dbusifyTypes(configdata), outFile, sort_keys=True, indent=4, separators=(',', ': '))
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
        """Saves a region of recorded video to external storage.

        Upon calling this method, the video system will switch to the `filesave` state and begin
        encoding video data to the output `device`. During this procedure, the `playbackStart`,
        `playbackPosition` and `playbackLength` parameters will be updated to track the progress
        of the filesave.

        When the filesave is completed, the video system will exit the `filesave` state, and
        revert back to whichever state it was in when the `startFilesave` method was called.

        Args:
            format (string) : Enumerate the output video format.
            device (string) : Name of the external storage device where video should be saved.
            filename (string, optional): Name to give to the video file (or directory for TIFF
                and DNG formats). When omitted, a filename is generated using the current date
                and time.
            start (int, optional) : The frame number in recorded video where the saved video
                begin (default: 0).
            length (int, optional) : The number of frames of video that should be saved (default:
                all frames).
            framerate (int, optional) : For formats with a media container (such as MPEG-4), this
                determines the framerate of the encoded media file (default: 60 frames per second).
            bitrate (int, optional) : For compressed formats, this sets the desired bitrate of the
                encoded file in bits per second (0.25 bits per pixel per second).
        """
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

        # Assemble the complete filename
        filename = args.pop('filename', '')
        if not filename:
            filename = time.strftime('vid_%F_%H-%M-%S')
        filename += suffixes.get(saveFormat, '')

        # Assemble the full pathname for recorded file.
        extStorage = self.camera.externalStorage
        if devName not in extStorage:
            raise ValueError('Invalid storage device given for recording')
        storage = extStorage[devName]
        filepath = os.path.abspath(os.path.join(storage['mount'], filename))
        if not filepath.startswith(storage['mount']):
            raise ValueError('Invalid filename given for recording')
        # TODO: Check for available space or any other sanity checking.
        # TODO: Start a saveDoneTimer to monitor available storage space?
        # TODO: Maybe launch the saveDoneTimer from the SOF/EOF signal.

        # Make the real call to save the file and pass through the remaining arguments.
        args['filename'] = filepath
        self.video.recordfile(args, reply_handler=onReply, error_handler=onError)
    
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}', async_callbacks=('onReply', 'onError'))
    def startPlayback(self, args, onReply=None, onError=None):
        """Switches the video system into playback mode, or sets the playback position and rate.

        When in playback mode, the camera will replay the captured video on the LCD, HDMI port
        and its RTSP stream. The user may configure the starting frame number and the rate at
        which video is replayed.

        The actual video stream replayed by the camera is fixed at either 30 or 60fps, the camera
        will either skip or duplicate frames to achieve the requested framerate. For example,
        setting the `framerate` to 120fps will typically play every 2nd frame at 60fps.

        The `framerate` can be either positive for forward playback, or negative to rewind backwards
        through video. A value of zero will effectively pause the video on the current frame.

        Args:
            position (int) : The starting frame number from which video should play.
            framerate (int) : The rate, in frames per second, at which video should advance
                through the playback memory.
            loopcount (int, optional) : The number of frames, after which the video system
                should return back to `position` and continue playback. This allows the user
                to select a subset of the video to play.
        """
        self.video.playback(args, reply_handler=onReply, error_handler=onError)
    
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
        """Retrieves parameter values from the API.
        
        The resulting dictionary will contain an element for each parameter that was successfully
        read from the API. If any parameters could not be read, they will be included in an `error`
        dictionary giving the reasons that they could not be retrieved.

        Args:
           *names (string): list of parameter names to rerieve from the API.
        """
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
        """Sets parameter values in the API.
        
        The resulting dictionary will contain an element for each paramer that was successfully
        set in the API. If any parameters could not be set, they will be included in an `error`
        dictionary given the reason that they could not be set. Typically this is either because
        the value given was not valid for the parameter, or the parameter did not exist.

        Args:
           **values (dict): A dictionary naming each of the parameters to update, and the
                to which they should be set.
        """
        
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
        results = {}
        for name in dir(self.camera):
            prop = getattr(type(self.camera), name, None)
            if (isinstance(prop, property)):
                # By API convention, all dicts are of type a{sv}
                value = getattr(self.camera, name)
                if isinstance(value, dict):
                    signature = 'a{sv}'
                elif isinstance(value, Enum):
                    signature = 's'
                else:
                    signature = dbus.lowlevel.Message.guess_signature(value)

                results[name] = {
                   'get': prop.fget is not None,
                   'set': prop.fset is not None,
                   'notifies': getattr(prop.fget, 'notifies', False),
                   'doc': prop.__doc__,
                   'type': signature,
                }

                # For Enumerated types, try to document the values too.
                if isinstance(value, Enum):
                    enumvals = gdocstring.parse(type(value)).get('attributes', {})
                    results[name]['enum'] = { k: v.get('doc', "") for (k, v) in enumvals.items() }

        return results
    
    @lru_cache(maxsize=1)
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def availableKeys(self):
        """Gets a list of the parameters available in the API.

        This method returns a dictionary with an entry for each parameter that can be
        accessed via the API. Each entry will describe the `type` of the parameter as 
        a D-Bus signature, a `doc` string that describes the function of the parameter,
        as well `get`, `set`, and `notify` flags that indicate whether the parameter can
        is read-only, read-write or generates `notify` events when its value changes.

        Returns:
            keys (dict): A dictionary describing each parameter in the API.
        """
        try:
            videoKeys = self.video.describe(timeout=150)
        except dbus.exceptions.DBusException:
            logging.error('could not load video available keys')
            videoKeys = {}
        
        return self.dbusifyMerge(self.ownKeys(), videoKeys)
    
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def availableCalls(self):
        """Gets a list of the methds that can be called via the API.
        
        This method returns a dictionary with an entry for each method that can be called
        via the API. Each entry will include a `brief` string that summarizes the purpose
        of the API method. Optionally, the entries may also contain a `description` with
        a more extensive detail, as well as `args` and `returns` dictionaries that list
        the parameters that the method accepts, and any values that the method returns.

        Returns:
            calls (dict): A dictionary describing each method that is callable by the API.
        """
        results = {}
        for name in dir(type(self)):
            call = getattr(type(self), name, None)
            if not getattr(call, '_dbus_is_method', False):
                continue
            if getattr(call, '_dbus_interface') != interface:
                continue
            
            # Check the API class for direct documentation, and failing
            # that, see if there exists a method in the camera class with
            # the same name that might provide something.
            logging.debug("Parsing docstrings for %s", name)
            if call.__doc__:
                results[name] = gdocstring.parse(call.__doc__)
            else:
                realcall = getattr(type(self.camera), name, None)
                if realcall:
                    results[name] = gdocstring.parse(realcall.__doc__)
            
        return self.dbusifyTypes(results)

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
        """ Restarts the control API and/or the camera.

        This method allows the user to restart their camera software, and optionally perform
        a full power cycle and/or return to factory default settings at the same time.

        Args:
            settings (boolean, optional): When true, the user and API settings are removed
                during the reboot, returning the camera to its factory default state.
            power (boolean, optional): When true, the camera will perform a full power cycle.
            reload (boolean, optional): When true, the control API and user interfaces will
                restart themeselves (default: true).
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
        try:
            self.runGenerator(self.camera.exportCalData(**args), "exportCalData")
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
    #Method('startRecording', arguments='a{sv}', regutns='a{sv}'),
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
    
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def startRecording(self, args):
        try:
            self.runGenerator(self.camera.startRecording(**args))
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
        """Flushes recoreded video data from memory.
        
        Normally when recording video, the camera will overwrite video data only as needed
        to make room for new data from the the image sensor. This method discards all video
        data from the video memory so that the user can start fresh on their next recording.
        """
        self.video.flush(reply_handler=onReply, error_handler=onError, timeout=150)

    #===============================================================================================
    #Method('getResolutionTimingLimits', arguments='a{sv}', returns='a{sv}'),
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def getResolutionTimingLimits(self, args):
        """Tests the camera ability to support a desired resolution and framerate.

        This method checks the sensor's ability to operate at the desired resolution parameters
        and, if successful, reports on some of the parameters that would apply if that resolution
        was configured. Otherwise, this method will generate an error to indicate that the
        resolution setting is not supported by the image sensor.

        Args:
            hRes (int): Horizontal image resolution, in pixels.
            vRes (int): Vertical image resolution, in pixels.
            hOffset (int, optional): Horizontal offset of the image from the right edge of the
                image sensor. (default: center the image horizontally)
            vOffset (int, optional): Vertical offset of the image from the top edge of the image
                sensor. (default: center the image vertically)
            bitDepth (int, optional): Desired pixel bit depth to use for image readout.
                (default: image sensor maximum)
            minFrameTime (float, optional): Minimum time period, in seconds between frames, that
                the imager sensor will operate at. (default: image sensor minimum)
        
        Raises:
            ValueError: Resolution settings are not valid for this image sensor.
        
        Returns:
            cameraMaxFrames (int): The maximum number of frames that the camera can save at this
                resolution and framerate setting.
            minFramePeriod (int): The minimum frame period, in nanoseconds between frames, that
                the image sensor can operate at.
            exposureMin (int): The minimum exposure period in nanoseconds that the image sensor
                can exposure a frame for.
            exposureMax (int): The maximmum exposure period in nanoseconds, that the image sensor
                can expose a frame for if framePeriod was set equal to minFramePeriod.
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
