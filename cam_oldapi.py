#!/usr/bin/python3
API_VERISON_STRING = '0.1'

import inspect
import logging
import dbus
import dbus.service
import dbus.mainloop.glib
from gi.repository import GLib

import pychronos
from pychronos import camera
from pychronos.sensors import lux1310, frameGeometry
import pychronos.regmaps as regmaps

interface = 'com.krontech.chronos.control'

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

    def __init__(self, bus, path, mainloop, camera):
        # FIXME: This seems hacky, just calling the class method directly.
        # Shouldn't we be using a super() call somehow?
        dbus.service.Object.__init__(self, bus, path)
        self.bus = bus
        self.mainloop = mainloop
    
        self.camera = camera
        self.io = regmaps.ioInterface()
        self.display = regmaps.display()

        self.currentState = 'idle'
        self.description = "Chronos SN:%s" % (self.camera.getSerialNumber())
        self.idNumber = None

        self.callLater(0.5, self.doReset, {'reset':True, 'sensor':True})
    
    ## Internal helper to iterate over a generator from the GLib mainloop.
    ## TODO: Do we need a callback for exception handling?
    def stepGenerator(self, generator, onError=None):
        try:
            delay = next(generator)
            GLib.timeout_add(int(delay * 1000), self.stepGenerator, generator)
        except StopIteration:
            pass
        except Exception as error:
            if (onError):
                onError(error)
        # Always return false to remove the Glib source for the last step.
        return False
    
    ## Internal helper to run a generator. This ought to be the preferred way to
    ## invoke a generator from within GLib's mainloop.
    def runGenerator(self, generator, onError=None):
        GLib.idle_add(self.stepGenerator, generator, onError)

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


    def pokeCamPipelineToRestart(self, geometry, zebra=False, peaking=True):
        logging.info('Notifying cam-pipeline to reconfigure display')
        video = self.bus.get_object('com.krontech.chronos.video', '/com/krontech/chronos/video')
        video.livedisplay({'hres':dbus.types.Int32(geometry.hRes, variant_level=1),
                           'vres':dbus.types.Int32(geometry.vRes, variant_level=1),
                           'zebra':dbus.types.Boolean(zebra, variant_level=1),
                           'peaking':dbus.types.Boolean(peaking, variant_level=1)})
            
    #===============================================================================================
    #Method('status',                   arguments='',      returns='a{sv}'),
    #Signal('statusHasChanged',         arguments=''),

    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def status(self, lastState=None, error=None):
        data = {'state':self.currentState}
        if lastState:
            data['lastState'] = lastState
        if error:
            data['error'] = error
        return data
    
    @dbus.service.signal(interface, signature='a{sv}')
    def statusHasChanged(self, args):
        return args
    
    def emitStateChanged(self, reason=None, details=None):
        data = {'state':self.currentState}
        if reason:
            data['reason'] = reason
        if details:
            data['details'] = details
        self.statusHasChanged(data)
    
    #===============================================================================================
    #Method('getCameraData',            arguments='',      returns='a{sv}'),
    #Method('getSensorData',            arguments='',      returns='a{sv}'),
    #Method('setDescription',           arguments='a{sv}', returns='a{sv}'),
    
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def getCameraData(self):
        data = {
            'model':'Chronos1.4',
            'apiVersion':API_VERISON_STRING,
            'serial':self.camera.getSerialNumber().strip(),
            'description':self.description,
        }
        if (self.idNumber is not None):
            data['idNumber'] = self.idNumber
        return data

    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def getSensorData(self):
        geometry = self.camera.sensor.getMaxGeometry()
        cfaPattern = self.camera.sensor.cfaPattern
        data = {
            'name': self.camera.sensor.name,
            'pixelRate':1.4*10**9,
            'hMax': geometry.hRes,
            'vMax': geometry.vRes,
            'hMin': 320,
            'vMin': 2,
            'hIncrement': 16,
            'vIncrement': 2,
            'pixelFormat': 'Y12'
        }
        # If a color sensor, set the pixel format accordingly.
        if (cfaPattern):
            cfaString = "BYR2-"
            for color in cfaPattern:
                cfaString += color
            data["pixelFormat"] = cfaString
        
        return data

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def setDescription(self, args):
        if args["description"]:
            self.description = str(args["description"])
        if args["idNumber"] is not None:
            self.idNumber = int(args["idNumber"])
        return self.status()
    
    #===============================================================================================
    #Method('reinitSystem',             arguments='a{sv}', returns='a{sv}'),
    
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def reinitSystem(self, args):
        self.callLater(0.0, self.doReset, args)
        self.currentState = 'reinitializing'
        return self.status()

    def doReset(self, args):
        recal = False
        self.emitStateChanged()
        reinitAll = args.get('all', False)
        if args.get('fpga') or reinitAll:
            reinitAll = True
            recal = True
            self.camera.reset(FPGA_BITSTREAM)

        if args.get('reset'):
            recal = True
            self.camera.reset()
            
        if args.get('sensor') or reinitAll:
            recal = True
            self.camera.sensor.reset()

        if recal:
            self.display.whiteBalance[0] = int(1.5226 * 4096)
            self.display.whiteBalance[1] = int(1.0723 * 4096)
            self.display.whiteBalance[2] = int(1.5655 * 4096)
            
            self.currentState = 'calibrating'
            self.runGenerator(self.startCalibration({'analog':True, 'zeroTimeBlackCal':True}))
        else:
            self.currentState = 'idle'
        self.emitStateChanged(reason='(re)initialization complete')

    #===============================================================================================
    #Method('getSensorCapabilities',    arguments='',      returns='a{sv}'),
    #Method('getSensorSettings',        arguments='',      returns='a{sv}'),
    #Method('getSensorLimits',          arguments='a{sv}', returns='a{sv}'),
    #Method('setSensorSettings',        arguments='a{sv}', returns='a{sv}'),

    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def getSensorCapabilities(self):
        return {'failed':'Not Implemented Yet - poke otter', 'id':self.ERROR_NOT_IMPLEMENTED_YET}

    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def getSensorSettings(self):
        returnGeom = dict(vars(self.camera.sensor.getCurrentGeometry()))
        returnGeom['framePeriod'] = self.camera.sensor.getCurrentPeriod()
        returnGeom['frameRate']   = 1.0 / returnGeom['framePeriod']
        returnGeom['exposure']    = self.camera.sensor.getCurrentExposure()
        return returnGeom

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def getSensorLimits(self, args):
        minPeriod, maxPeriod = self.camera.sensor.getPeriodRange(self.camera.sensor.getCurrentGeometry())
        return {'minPeriod':minPeriod, 'maxPeriod':maxPeriod}

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def setSensorSettings(self, args):
        geom = self.camera.sensor.getCurrentGeometry()
        geom.hRes      = args.get('hRes',      geom.hRes)
        geom.vRes      = args.get('vRes',      geom.vRes)
        geom.hOffset   = args.get('hOffset',   geom.hOffset)
        geom.vOffset   = args.get('vOffset',   geom.vOffset)
        geom.vDarkRows = args.get('vDarkRows', geom.vDarkRows)
        geom.bitDepth  = args.get('bitDepth',  geom.bitDepth)

        # set default value
        framePeriod, _ = self.camera.sensor.getPeriodRange(geom)

        # check if we have a frameRate field and if so convert it to framePeriod
        frameRate = args.get('frameRate')
        if frameRate:
            framePeriod = 1.0 / frameRate

        # if we have a framePeriod explicit field, override frameRate or default value
        framePeriod = args.get('framePeriod', framePeriod)

        # set exposure or use a default of 95% framePeriod
        exposurePeriod = args.get('exposure', framePeriod * 0.95) # self.camera.sensor.getExposureRange(geom))

        logging.info('framePeriod, exposurePeriod: %f, %f', framePeriod, exposurePeriod)
        # set up video
        self.camera.sensor.setResolution(geom)
        self.camera.sensor.setFramePeriod(framePeriod)
        self.camera.sensor.setExposurePeriod(exposurePeriod)
        self.camera.setupRecordRegion(geom, self.camera.REC_REGION_START)
        self.camera.setupDisplayTiming(geom)

        # tell video pipeline to restart
        self.callLater(0.1, self.pokeCamPipelineToRestart, geom)

        # start a calibration loop
        self.runGenerator(self.startCalibration({'analog':True, 'zeroTimeBlackCal':True}))

        # get the current config so we can return the real values
        appliedGeometry = self.getSensorSettings()
        self.emitStateChanged(reason='resolution changed', details={'geometry':appliedGeometry})
        return appliedGeometry

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def setSensorTiming(self, args):
        # check if we have a frameRate field and if so convert it to framePeriod
        frameRate = args.get('frameRate')
        if frameRate:
            framePeriod = 1.0 / frameRate

        # if we have a framePeriod explicit field, override frameRate or default value
        framePeriod = args.get('framePeriod', framePeriod)

        # set exposure or use a default of 95% framePeriod
        exposurePeriod = args.get('exposure', framePeriod * 0.95) # self.camera.sensor.getExposureRange(geom))

        self.camera.sensor.setFramePeriod(framePeriod)
        self.camera.sensor.setExposurePeriod(exposurePeriod)

        returnData = dict()
        returnData['framePeriod'] = self.camera.sensor.getCurrentPeriod()
        returnData['frameRate']   = 1.0 / returnData['framePeriod']
        returnData['exposure']    = self.camera.sensor.getCurrentExposure()
        return returnData

    #===============================================================================================
    #Method('getIoCapabilities',        arguments='',      returns='a{sv}'),
    #Method('getIoMapping',             arguments='a{sv}', returns='a{sv}'),
    #Method('setIoMapping',             arguments='a{sv}', returns='a{sv}'),
    #Signal('ioEvent',                  arguments='a{sv}'),

    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def dbusGetIoCapabilities(self):
        return {'failed':'Not Implemented Yet - poke otter', 'id':self.ERROR_NOT_IMPLEMENTED_YET}

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def getIoMapping(self, args):
        return self.io.getConfiguration()

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def setIoMapping(self, args):
        self.io.setConfiguration(args)
        return self.io.getConfiguration()
    
    #===============================================================================================
    #Method('getCalCapabilities',       arguments='',      returns='a{sv}'),
    #Method('calibrate',                arguments='a{sv}', returns='a{sv}'),
    
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def getCalCapabilities(self):
        return {'failed':'Not Implemented Yet - poke otter', 'id':self.ERROR_NOT_IMPLEMENTED_YET}

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def calibrate(self, args):
        self.currentState = 'calibrating'
        self.runGenerator(self.startCalibration(args))
        return self.status()

    def startCalibration(self, args):
        self.emitStateChanged()

        blackCal         = args.get('blackCal') or args.get('fpn')
        zeroTimeBlackCal = args.get('zeroTimeBlackCal') or args.get('basic')
        analogCal        = args.get('analogCal') or args.get('analog') or args.get('basic')

        if analogCal:
            logging.info('starting analog calibration')
            yield from self.camera.sensor.startAnalogCal()

        if zeroTimeBlackCal:
            logging.info('starting zero time black calibration')
            yield from self.camera.startZeroTimeBlackCal()
        elif blackCal:
            logging.info('starting standard black calibration')
            yield from self.camera.startBlackCal()
                
        self.currentState = 'idle'
        self.emitStateChanged(reason='calibration complete')
    
    #===============================================================================================
    #Method('getColorMatrix',           arguments='',      returns='a{sv}'),
    #Method('setColorMatrix',           arguments='a{sv}', returns='a{sv}'),
    #Method('getWhiteBalance',          arguments='',      returns='a{sv}'),
    #Method('setWhiteBalance',          arguments='a{sv}', returns='a{sv}'),
    
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def getColorMatrix(self):
        colorMatrix = [self.display.colorMatrix[0]/4096.0,self.display.colorMatrix[1]/4096.0,self.display.colorMatrix[2]/4096.0,
                       self.display.colorMatrix[3]/4096.0,self.display.colorMatrix[4]/4096.0,self.display.colorMatrix[5]/4096.0,
                       self.display.colorMatrix[6]/4096.0,self.display.colorMatrix[7]/4096.0,self.display.colorMatrix[8]/4096.0]
        logging.info('colorMatrix: %s', str(colorMatrix))
        return colorMatrix
    
    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def setColorMatrix(self, args):
        # TODO: implement setting colorMatrix
        return self.getColorMatrix()

    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def getWhiteBalance(self):
        red   = self.display.whiteBalance[0] / 4096.0
        green = self.display.whiteBalance[1] / 4096.0
        blue  = self.display.whiteBalance[2] / 4096.0
        whiteBalance = {'red':red, 'green':green, 'blue':blue}
        logging.info('whiteBalance: %s (type:%s)', str(whiteBalance), str(type(whiteBalance)))
        return whiteBalance

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def setWhiteBalance(self, args):
        red   = args.get('red')
        green = args.get('green')
        blue  = args.get('blue')
        if not red or not green or not blue:
            return {'failed':'args does not contain all of red, green and blue values', 'id':self.VALUE_ERROR}
        
        self.display.whiteBalance[0] = int(red   * 4096)
        self.display.whiteBalance[1] = int(green * 4096)
        self.display.whiteBalance[2] = int(blue  * 4096)
        return self.getWhiteBalance()

    #===============================================================================================
    #Method('getSequencerCapabilities', arguments='',      returns='a{sv}'),
    #Method('getSequencerProgram',      arguments='a{sv}', returns='a{sv}'),
    #Method('setSequencerProgram',      arguments='a{sv}', returns='a{sv}'),
    #Method('startRecord',              arguments='a{sv}', returns='a{sv}'),
    #Method('stopRecord',               arguments='a{sv}', returns='a{sv}'),
    
    @dbus.service.method(interface, in_signature='', out_signature='a{sv}')
    def getSequencerCapabilities(self):
        return {'failed':'Not Implemented Yet - poke otter', 'id':self.ERROR_NOT_IMPLEMENTED_YET}

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def getSequencerProgram(self, args):
        return {'failed':'Not Implemented Yet - poke otter', 'id':self.ERROR_NOT_IMPLEMENTED_YET}

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def setSequencerProgram(self, args):
        return {'failed':'Not Implemented Yet - poke otter', 'id':self.ERROR_NOT_IMPLEMENTED_YET}

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def startRecord(self, args):
        if self.currentState != 'idle':
            return {'failed':'busy', 'state':self.currentState}

        self.currentState = 'recording'
        self.runGenerator(self.doStartRecord(args))
        return {'success':'started recording'}

    @dbus.service.method(interface, in_signature='a{sv}', out_signature='a{sv}')
    def stopRecord(self, args):
        self.camera.stopRecording()
        return {'success':'stopped recording'}

    def doStartRecord(self, args):
        self.emitStateChanged()

        # send flush
        
        geometry = self.camera.sensor.getCurrentGeometry()

        # set up some defaults
        if not 'blkTermFull'     in args: args['blkTermFull']     = True
        if not 'recTermMemory'   in args: args['recTermMemory']   = True
        if not 'recTermBlockEnd' in args: args['recTermBlockEnd'] = True

        # use blockSize if given; otherwise use nFrames or use getRecordMaxFrames
        nFrames = args.get('nFrames', self.camera.getRecordingMaxFrames(geometry))
        args['blockSize'] = args.get('blockSize', nFrames)

        # override a few variables
        args['nextState'] = 0
        
        # TODO: make this use args
        program = [ seqcommand(**args) ]

        logging.info('Recording program: %s', program[0])
        
        yield from self.camera.startRecording(program)
        self.camera.stopRecording()
        
        self.currentState = 'idle'
        self.emitStateChanged(reason='recording complete')

# Run the control API
if __name__ == "__main__":
    # Enable logging.
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s [%(funcName)s] %(message)s')

    ## TODO: Initialize the FPGA.
    ## TODO: This needs to go into an init script somewhere.

    # Use the GLib mainloop.
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    mainloop = GLib.MainLoop()

    # Setup resources.
    cam  = camera(lux1310())
    bus  = dbus.SystemBus()
    name = dbus.service.BusName('com.krontech.chronos.control', bus=bus)
    obj  = controlApi(bus, '/com/krontech/chronos/control', mainloop, cam)

    # Run the mainloop.
    logging.info("Running control service...")
    mainloop.run()
