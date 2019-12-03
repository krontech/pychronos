# IO system for Chronos high speed cameras
import os
import logging
import pychronos
import pychronos.regmaps as regmaps

from . import props
from .props import camProperty as camProperty

class io:
    def __init__(self, onChange=None):
        self.onChange = onChange

        # Get the hardware assets to control the IO.
        self.regs = regmaps.ioInterface()
        self.io1Pwm = pychronos.pwm(1, 10000, 0.366)
        self.io2Pwm = pychronos.pwm(2, 10000, 0.366)
    
    def doReset(self):
        # Enable the new IO block.
        self.regs.enable = True

    def __propChange(self, name):
        """Quick and dirty wrapper to throw an on-change event by name"""
        try:
            logging.info("%s changed", name)
            self.onChange(name, getattr(self, name))
        except Exception as e:
            logging.debug("onChange handler failed: %s", e)
            pass
    
    # Get an IO source configuration
    def getIoSource(self, src):
        ret = {
            "source": src.source,
            "invert": src.invert,
            "debounce": src.debounce
        }
        # Drive strength only exists for IO1 and IO2.
        try:
            ret["drive"] = getattr(src, "drive")
        except AttributeError as e:
            pass
        # TODO: For full backwards compatibility we need to
        # re-implement shutterTriggersFrame too, but I don't
        # think there is anything that uses it in the wild?
        # Return the dict.
        # TODO: For full backwrads compatibility we need to
        # re-implement the delay timing values, but I think
        # those are better done as a separate parameter.
        return ret
    
    # Configure an IO source configuration
    def setIoSource(self, src, values):
        src.source = values.get("source", "none")
        src.invert = values.get("invert", False)
        src.deboucne = values.get("debounce", False)
        # Drive strength only exists for IO1 and IO2.
        try:
            src.drive = values.get("drive", 0)
        except AttributeError as e:
            pass

    #===============================================================================================
    # API Parameters: IO Configuration Group
    @camProperty()
    def ioDelayTime(self):
        """float: Delay time, in seconds, for the programmable delay block"""
        return self.regs.delayTime
    @ioDelayTime.setter
    def ioDelayTime(self, value):
        self.regs.delayTime = value
    
    @camProperty(notify=True, save=True)
    def ioMappingIo1(self):
        """dict: Ouput driver 1 configuration"""
        return self.getIoSource(self.regs.io1)
    @ioMappingIo1.setter
    def ioMappingIo1(self, value):
        self.setIoSource(self.regs.io1, value)
        self.__propChange("ioMappingIo1")

    @camProperty(notify=True, save=True)
    def ioMappingIo2(self):
        """dict: Output driver 2 configuration"""
        return self.getIoSource(self.regs.io2)
    @ioMappingIo2.setter
    def ioMappingIo2(self, value):
        self.setIoSource(self.regs.io2, value)
        self.__propChange("ioMappingIo2")
    
    @camProperty(notify=True, save=True)
    def ioMappingCombOr1(self):
        """dict: Combinatorial block OR input 1 configuration"""
        return self.getIoSource(self.regs.combOr1)
    @ioMappingCombOr1.setter
    def ioMappingCombOr1(self, value):
        self.setIoSource(self.regs.combOr1, value)
        self.__propChange("ioMappingCombOr1")
    
    @camProperty(notify=True, save=True)
    def ioMappingCombOr2(self):
        """dict: Combinatorial block OR input 2 configuration"""
        return self.getIoSource(self.regs.combOr2)
    @ioMappingCombOr2.setter
    def ioMappingCombOr2(self, value):
        self.setIoSource(self.regs.combOr2, value)
        self.__propChange("ioMappingCombOr2")
    
    @camProperty(notify=True, save=True)
    def ioMappingCombOr3(self):
        """dict: Combinatorial block OR input 3 configuration"""
        return self.getIoSource(self.regs.combOr3)
    @ioMappingCombOr3.setter
    def ioMappingCombOr3(self, value):
        self.setIoSource(self.regs.combOr3, value)
        self.__propChange("ioMappingCombOr3")

    @camProperty(notify=True, save=True)
    def ioMappingCombAnd(self):
        """dict: Combinatorial block AND input configuration"""
        return self.getIoSource(self.regs.combAnd)
    @ioMappingCombAnd.setter
    def ioMappingCombAnd(self, value):
        self.setIoSource(self.regs.combAnd, values)
        self.__propChange("ioMappingCombAnd")

    @camProperty(notify=True, save=True)
    def ioMappingCombXor(self):
        """dict: Combinatorial block XOR input configuration"""
        return self.getIoSource(self.regs.combXor)
    @ioMappingCombXor.setter
    def ioMappingCombXor(self, value):
        self.setIoSource(self.regs.combXor, value)
        self.__propChange("ioMappingCombXor")

    @camProperty(notify=True, save=True)
    def ioMappingDelay(self):
        """dict: Programmable delay block input configuration"""
        return self.getIoSource(self.regs.delay)
    @ioMappingDelay.setter
    def ioMappingDelay(self, value):
        self.setIoSource(self.regs.delay, value)
        self.__propChange("ioMappingDelay")

    @camProperty(notify=True, save=True)
    def ioMappingToggleSet(self):
        """dict: Toggle/flip-flop block SET input configuration"""
        return self.getIoSource(self.regs.toggleSet)
    @ioMappingToggleSet.setter
    def ioMappingToggleSet(self, value):
        self.setIoSource(self.regs.toggleSet, value)
        self.__propChange("ioMappingToggleSet")

    @camProperty(notify=True, save=True)
    def ioMappingToggleClear(self):
        """dict: Toggle/flip-flop block CLEAR input configuration"""
        return self.getIoSource(self.regs.toggleClear)
    @ioMappingToggleClear.setter
    def ioMappingToggleClear(self, value):
        self.setIoSource(self.regs.toggleClear, values)
        self.__propChange("ioMappingToggleClear")
        
    @camProperty(notify=True, save=True)
    def ioMappingToggleFlip(self):
        """dict: Toggle/flip-flop block FLIP input configuration"""
        return self.getIoSource(self.regs.toggleFlip)
    @ioMappingToggleFlip.setter
    def ioMappingToggleFlip(self, value):
        self.setIoSource(self.regs.toggleFlip, value)
        self.__propChange("ioMappingToggleFlip")
    
    @camProperty(notify=True, save=True)
    def ioMappingShutter(self):
        """dict: Timing block shutter control signal configuration"""
        return self.getIoSource(self.regs.shutter)
    @ioMappingShutter.setter
    def ioMappingShutter(self, value):
        self.setIoSource(self.regs.shutter, value)
        self.__propChange("ioMappingShutter")
    
    @camProperty(notify=True, save=True)
    def ioMappingStartRec(self):
        """dict: Recording start signal configuration"""
        return self.getIoSource(self.regs.start)
    @ioMappingStartRec.setter
    def ioMappingStartRec(self, value):
        self.setIoSource(self.regs.start, value)
        self.__propChange("ioMappingStartRec")

    @camProperty(notify=True, save=True)
    def ioMappingStopRec(self):
        """dict: Recording stop signal configuration"""
        return self.getIoSource(self.regs.stop)
    @ioMappingStopRec.setter
    def ioMappingStopRec(self, value):
        self.setIoSource(self.regs.stop, value)
        self.__propChange("ioMappingStopRec")

    @camProperty(notify=True, save=True)
    def ioMappingTrigger(self):
        """dict: Recording trigger signal configuration"""
        return self.getIoSource(self.regs.trigger)
    @ioMappingTrigger.setter
    def ioMappingTrigger(self, value):
        self.setIoSource(self.regs.trigger, value)
        self.__propChange("ioMappingTrigger")

    __io1PwmScale = 6.838 # 100% PWM triggers at 6.838 volts.
    @camProperty(notify=True, save=True)
    def ioThresholdIo1(self):
        """float: Voltage threshold at which trigger input signal 1 should go high."""
        return float(self.io1Pwm.duty * self.__io1PwmScale)
    @ioThresholdIo1.setter
    def ioThresholdIo1(self, value):
        dutyCycle = (value / self.__io1PwmScale)
        if (dutyCycle < 0.001):
            dutyCycle = 0.001
        if (dutyCycle > 0.999):
            dutyCycle = 0.999
        self.io1Pwm.duty = dutyCycle
    
    __io2PwmScale = 6.838 # 100% PWM triggers at 6.838 volts.
    @camProperty(notify=True, save=True)
    def ioThresholdIo2(self):
        """float: Voltage threshold at which trigger input signal 2 should go high."""
        return float(self.io2Pwm.duty * self.__io2PwmScale)
    @ioThresholdIo2.setter
    def ioThresholdIo2(self, value):
        dutyCycle = (value / self.__io2PwmScale)
        if (dutyCycle < 0.001):
            dutyCycle = 0.001
        if (dutyCycle > 0.999):
            dutyCycle = 0.999
        self.io2Pwm.duty = dutyCycle

    @camProperty()
    def ioSourceStatus(self):
        """dict: The available IO signals and their current values."""
        return {
            'none': False,
            'io1': self.regs.sourceIo1 == 1,
            'io2': self.regs.sourceIo2 == 1,
            'io3': self.regs.sourceIo3 == 1,
            'comb': self.regs.sourceComb == 1,
            'software': self.regs.sourceSoftware == 1,
            'delay': self.regs.sourceDelay == 1,
            'toggle': self.regs.sourceToggle == 1,
            'shutter': self.regs.sourceShutter == 1,
            'recording': self.regs.sourceRecording == 1,
            'dispFrame': self.regs.sourceDispFrame == 1,
            'startRec': self.regs.sourceStartRec == 1,
            'endRec': self.regs.sourceEndRec == 1,
            'nextSeg': self.regs.sourceNextSeg == 1,
            'timingIo': self.regs.sourceTimingIo == 1,
            'alwaysHigh': True
        }
    
    @camProperty()
    def ioOutputStatus(self):
        """dict: The output signals from the IO block and their current values."""
        return {
            'io1': self.regs.io1_SignalOut == 1,
            'io2': self.regs.io2_SignalOut == 1,
            'comb': self.regs.combSignalOut == 1,
            'delay': self.regs.delaySignalOut == 1,
            'start': self.regs.startSignalOut == 1,
            'stop': self.regs.stopSignalOut == 1,
            'toggle': self.regs.toggleSignalOut == 1,
            'shutter': self.regs.shutterSignalOut == 1
        }

    @camProperty()
    def ioDetailedStatus(self):
        return {
            'detailedComb': {
                'or1': self.regs.combOr1SignalOut == 1,
                'or2': self.regs.combOr2SignalOut == 1,
                'or3': self.regs.combOr3SignalOut == 1,
                'xor': self.regs.combXOrSignalOut == 1,
                'and': self.regs.combAndSignalOut == 1
            },
            'sources': self.ioSourceStatus,
            'outputs': self.ioOutputStatus,
            'edgeTimers': {
                'io1': {
                    'rising': self.regs.io1TimeSinceRising / self.regs.IO_BLOCK_FREQUENCY,
                    'falling': self.regs.io1TimeSinceFalling / self.regs.IO_BLOCK_FREQUENCY
                },
                'io2': {
                    'rising': self.regs.io2TimeSinceRising / self.regs.IO_BLOCK_FREQUENCY,
                    'falling': self.regs.io2TimeSinceFalling / self.regs.IO_BLOCK_FREQUENCY
                },
                'start': {
                    'rising': self.regs.startTimeSinceRising / self.regs.IO_BLOCK_FREQUENCY,
                    'falling': self.regs.startTimeSinceFalling / self.regs.IO_BLOCK_FREQUENCY
                },
                'stop': {
                    'rising': self.regs.stopTimeSinceRising / self.regs.IO_BLOCK_FREQUENCY,
                    'falling': self.regs.stopTimeSinceFalling / self.regs.IO_BLOCK_FREQUENCY
                },
                'shutter': {
                    'rising': self.regs.shutterTimeSinceRising / self.regs.IO_BLOCK_FREQUENCY,
                    'falling': self.regs.shutterTimeSinceFalling / self.regs.IO_BLOCK_FREQUENCY
                },
                'toggle': {
                    'rising': self.regs.toggleTimeSinceRising / self.regs.IO_BLOCK_FREQUENCY,
                    'falling': self.regs.toggleTimeSinceFalling / self.regs.IO_BLOCK_FREQUENCY
                },
                'interrupt': {
                    'rising': self.regs.interruptTimeSinceRising / self.regs.IO_BLOCK_FREQUENCY,
                    'falling': self.regs.interruptTimeSinceFalling / self.regs.IO_BLOCK_FREQUENCY
                }
            }
        }
