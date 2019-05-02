#!/usr/bin/python3

import os, sys, pdb
import json
import argparse
from functools import lru_cache
from pathlib import Path

import inspect
import logging
import traceback
import dbus
import dbus.service
import dbus.mainloop.glib
from gi.repository import GLib

import pychronos
from pychronos import camera, CameraError
from pychronos.sensors import lux1310, frameGeometry
import pychronos.regmaps as regmaps

interface = 'com.krontech.chronos.control'
dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
bus = dbus.SystemBus()
video = bus.get_object('com.krontech.chronos.video',
                       '/com/krontech/chronos/video')

#-----------------------------------------------------------------
# Some constants that ought to go into a board-specific dict.
FPGA_BITSTREAM = "/var/camera/FPGA.bit"
GPIO_ENCA = "/sys/class/gpio/gpio20/value"
GPIO_ENCB = "/sys/class/gpio/gpio26/value"
GPIO_ENCSW = "/sys/class/gpio/gpio27/value"
REC_LED_FRONT = "/sys/class/gpio/gpio41/value"
REC_LED_BACK = "/sys/class/gpio/gpio25/value"
#-----------------------------------------------------------------

class store():
    """A persistant key/value store which survives reboots.
        
        Example:
            Store.set('foo', 5)
            assert Store.get('foo') == 5 #passes
            #«reboot camera»
            assert Store.get('foo') == 5 #passes
        """
    
    _basepath = Path(os.path.expanduser('~/.config/Krontech/dbus control api'))
    try:
        _basepath.mkdir(parents=True)
    except FileExistsError:
        pass
    
    @staticmethod
    def get(name: str, *args):
        """get(name[, default]): Retrieve a set value.
            
            If a default is supplied, use it if the stored
            value cannot be loaded. (ie, is corrupt or dne)"""
        
        try:
            with (store._basepath/name).open(mode='r') as file:
                return json.load(file)
        except Exception as e:
            if args:
                return args[0]
            else:
                #This merges the errors "we can't find the data" and "we can't find all the data".
                raise ValueError("No valid saved data named '%s' found and no default provided." % name)
    
    @staticmethod
    def set(name: str, value: any):
        """set(name, value): Persist a value to disk.
            
            Can be got() later."""
        
        with (store._basepath/name).open(mode='w') as file:
            json.dump(
                value,
                file,
                ensure_ascii = False,
                check_circular = False,
                indent = 4,
            )
    
    @staticmethod
    def all():
        result = {}
        for name in os.listdir(str(store._basepath)):
            result[name] = store.get(name)
        return result

class controlApi(dbus.service.Object):
    ## This feels like a duplication of the Python exceptions.
    ERROR_NOT_IMPLEMENTED_YET = 9999
    VALUE_ERROR               = 1
    
    def __init__(self, bus, path, mainloop, camera):
        # FIXME: This seems hacky, just calling the class method directly.
        # Shouldn't we be using a super() call somehow?
        dbus.service.Object.__init__(self, bus, path)
        self.bus = bus
        self.mainloop = mainloop
    
        self.camera = camera
        self.io = regmaps.ioInterface()
        self.display = regmaps.display()

        # Install a callback to catch parameter and state changes.
        self.camera.setOnChange(self.onChangeHandler)
        self.changeset = None

        self.callLater(0.5, self.softReset)
        
    ## Internal helper to iterate over a generator from the GLib mainloop.
    def stepGenerator(self, generator):
        try:
            delay = next(generator)
            GLib.timeout_add(int(delay * 1000), self.stepGenerator, generator)
        except StopIteration:
            pass
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
        result = dbus.types.Dictionary(variant_level=variant_level)
        for key in src:
            value = src[key]
            if isinstance(value, int):
                result[key] = dbus.types.Int32(value, variant_level=1)
            elif isinstance(value, float):
                result[key] = dbus.types.Double(value, variant_level=1)
            elif isinstance(value, dict):
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

    def notifyChanges(self):
        self.notify(self.changeset)
        self.changeset = None
        return False
    
    @dbus.service.signal(interface, signature='a{sv}')
    def notify(self, args):
        return self.changeset
    
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
                if name in self.availableKeys()
            ]
            videoAttrs = [
                name
                for name in attrs 
                if name not in controlAttrs
            ]
            
            data = video.get(videoAttrs) if videoAttrs else {}
            for name in controlAttrs:
                data[name] = self.dbusifyTypes(getattr(self.camera, name))
            return data
        
        except dbus.exceptions.DBusException as e:
            if e.get_dbus_name() != 'com.krontech.chronos.UnknownAttribute':
                raise e
            else:
                #Video didn't have the attr, we don't have the attr, return a nice error message saying _which_ attr is causing issues and why.
                raise dbus.exceptions.DBusException(
                    'com.krontech.chronos.UnknownAttribute',
                    "'%s' is not a known attribute. Known attributes are: %s" % (
                        e.get_dbus_message().split()[0], #attribute name
                        sorted(set(self.availableKeys()) | set(video.availableKeys())) #all attribute names
                    )
                )
    
    def paramsorter(self, name):
        """Internal helper function to sort a parameter by priority"""
        try:
            camprop = getattr(type(self.camera), name)
            return camprop.fget.prio
        except:
            return 0
    
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def set(self, newValues):
        """Set named values in the control API and the video API."""
        
        keys = sorted(newValues.keys(), key=self.paramsorter, reverse=True)
        videoAttributes = {}
        for name in keys:
            # If the property exists in the camera class, set it.
            value = newValues[name]
            logging.debug("Setting %s -> %s", name, value)
            camprop = getattr(type(self.camera), name)
            if camprop.fset is not None:
                setattr(self.camera, name, value)
                if (getattr(camprop.fget, 'savable', False)):
                    store.set(name, value)
        
            # Otherwise, try setting the property in the video interface.
            videoAttributes[name] = value
        
        # For any keys that don't exist - try setting them in the video API.
        if videoAttributes:
            video.set(self.dbusifyTypes(videoAttributes),
                    reply_handler=self.dbusReplyHandler,
                    error_handler=self.dbusErrorHandler)
        
        #HACK: Manually poke the video pipeline back into live display after changing
        # the display resolution. This should eventually go away by making the video
        # system more autonomous with regards to the live display res.
        if ('resolution' in newValues):
            res = self.camera.resolution
            logging.info('Notifying cam-pipeline to reconfigure display')
            video.livedisplay({
                'hres':dbus.types.Int32(res['hRes'], variant_level=1),
                'vres':dbus.types.Int32(res['vRes'], variant_level=1)
            }, reply_handler=self.dbusReplyHandler, error_handler=self.dbusErrorHandler)
        
        return self.get(keys) # return the settings as they've been applied
    
    
    #===============================================================================================
    #Method('availableKeys', arguments='', returns='as')
    #Method('availableCalls', arguments='', returns='as')
    @lru_cache(maxsize=1)
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def availableKeys(self):
        """Get a list of the properties we can get/set/subscribe.
            
            For a list of functions, see org.freedesktop.DBus.Properties.GetAll."""
        
        #Return a map, vs a list with a name key, because everything else is a{sv}.
        #dbg() #x = getattr(type(self.camera), 'sensorHMax')
        return self.dbusifyTypes({
            elem: {
                'get': getattr(type(self.camera), elem).fget is not None,
                'set': getattr(type(self.camera), elem).fset is not None,
                'notifies': getattr(getattr(type(self.camera), elem).fget, 'notifies', False), #set with camProperty
            }
            for elem in dir(self.camera)
            if elem[0] != '_'
            and isinstance(getattr(type(self.camera), elem, None), property) #is a getter, maybe a setter
        })
    
    @lru_cache(maxsize=1)
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def availableCalls(self):
        """Get a list of the properties we can get/set/subscribe.
            
            For a list of functions, see org.freedesktop.DBus.Properties.GetAll."""
        
        return self.dbusifyTypes({
            elem: {
                'constant': False,
                'action': 'set', #or 'get'.
            }
            for elem in dir(self)
            if elem[0] != '_'
            and callable(getattr(type(self), elem, None))
        })
    
    #===============================================================================================
    #Method('status', arguments='', returns='a{sv}')
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def status(self):
        return {'state':self.camera.state}

    #===============================================================================================
    #Method('softReset', arguments='', returns='a{sv}'),
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def softReset(self):
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
    
    def runSoftReset(self):
        # Wrapper method to do a complete soft reset of both the camera class,
        # reloading of parameters, and tickling of the video system.
        yield from self.camera.softReset()

        # Reload the configuration.
        self.set(store.all())

        # Re-run initial calibration.
        yield from self.camera.startCalibration(analogCal=True, zeroTimeBlackCal=True)

        # Re-configure the video system back into live display.
        res = self.camera.resolution
        logging.info('Notifying cam-pipeline to reconfigure display')
        video.livedisplay({
            'hres':dbus.types.Int32(res['hRes'], variant_level=1),
            'vres':dbus.types.Int32(res['vRes'], variant_level=1)
        }, reply_handler=self.dbusReplyHandler, error_handler=self.dbusErrorHandler)

    #===============================================================================================
    #Method('startCalibration', arguments='a{sv}', returns='a{sv}'),
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def startCalibration(self, args):
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
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def startAutoWhiteBalance(self, args):
        logging.info('starting white balance')
        try:
            self.runGenerator(self.camera.startWhiteBalance(args))
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
        self.camera.wbMatrix = self.camera.wbCustom
        return {
            "state": self.camera.state
        }
    
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def startRecording(self):
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

    #===============================================================================================
    #Method('testResolution', arguments='a{sv}', returns='a{sv}'),
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def testResolution(self, args):
        if (self.camera.sensor.isValidResolution(args)):
            fpMin, fpMax = self.camera.sensor.getPeriodRange(args)
            expMin, expMax = self.camera.sensor.getExposureRange(args, fpMin)
            return {
                "cameraMaxFrames": self.camera.getRecordingMaxFrames(args),
                "minFramePeriod": int(fpMin * 1000000000),
                "exposureMin": int(expMin * 1000000000),
                "exposureMax": int(expMax * 1000000000)
            }
        else:
            return {
                "error": "Invalid Resolution"
            }

# Run the control API
if __name__ == "__main__":
    # Enable logging.
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s [%(funcName)s] %(message)s')

    # Do argument parsing
    parser = argparse.ArgumentParser(description="Chronos control daemon")
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
   
    name = dbus.service.BusName('com.krontech.chronos.control', bus=bus)
    obj  = controlApi(bus, '/com/krontech/chronos/control', mainloop, cam)

    # Run the mainloop.
    logging.info("Running control service...")
    mainloop.run()
