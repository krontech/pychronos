import pychronos
import time
import logging
from pychronos.regmaps import timing, ioInterface

class lux2100timing(timing):
    TIMING_HZ = 75000000

    PROGRAM_NONE           = -1
    PROGRAM_STANDARD       = 0
    PROGRAM_SHUTTER_GATING = 1
    PROGRAM_2POINT_HDR     = 2
    PROGRAM_3POINT_HDR     = 3
    PROGRAM_FRAME_TRIG     = 4
    PROGRAM_N_FRAME_TRIG   = 5
    
    def __init__(self):
        super().__init__()

        # make sure some sane values are in the internal backing
        # registers in case the user just sets the integration and frame times
        self.__disableFrameTrig = False
        self.__program          = self.PROGRAM_NONE
        self.__txnWidth         = 50
        
        self.io = ioInterface()

        if self.version >= 0 and self.subver >= 2:
            self.useMinLinesWait = True
        else:
            self.useMinLinesWait = False

    def setPulsedPattern(self, wavetableLength, hSync=2):
        self.pulsedAbnLowPeriod = wavetableLength
        self.pulsedAbnHighPeriod = hSync

    def singleFrame(self):
        self.requestFrame = 1

    def programInterm(self, readoutTime=90000, timeout=0.01):
        """This programs an blank program which will make sure the readout
        section has enough time to complete readout before switching to a
        new timing program.
        This has all IO disabled including the one driving the io block...
        """
        logging.debug('programInterm - flip')
        self.runProgram(prog=[self.NONE + readoutTime, self.NONE | self.TIMING_RESTART], timeout=timeout)

    def programShutterGating(self, t2Time=17, readoutTime=90000, timeout=0.01):
        self.__program = self.PROGRAM_SHUTTER_GATING
        self.__disableFrameTrig = False

        logging.debug('programShutterGating')
        # make sure the readout completes
        self.programInterm(readoutTime, timeout)
        self.io.shutterTriggersFrame = False
        
        prog = [self.PRSTN + t2Time]
        prog.append(self.ABN + self.PRSTN     | self.TIMING_WAIT_FOR_ACTIVE)
        prog.append(self.IODRIVE + self.PRSTN | self.TIMING_WAIT_FOR_INACTIVE)
        prog.append(self.TXN + self.PRSTN     | self.__txnWidth)
        if self.useMinLinesWait:
            prog.append(self.TXN + self.PRSTN | self.TIMING_WAIT_FOR_NLINES)
        prog.append(self.NONE | self.TIMING_RESTART)
        
        logging.debug('programShutterGating - flip')
        self.runProgram(prog, timeout)
        self.io.shutterTriggersFrame = True
        
    def programTriggerFrames(self, frameTime, integrationTime, t2Time=17, readoutTime=90000, timeout=0.01):
        frameTime       = int(frameTime)
        integrationTime = int(integrationTime)
        t2Time          = int(t2Time)
        
        if (frameTime <= integrationTime):
            logging.error("frameTime (%d) must be longer than integrationTime (%d)", frameTime, integrationTime)
            integrationTime = int(frameTime * 0.95)

        logging.debug('TriggerFrames: %d, %d', frameTime, integrationTime)

        if self.__program != self.PROGRAM_FRAME_TRIG:
            # make sure the readout completes
            self.programInterm(readoutTime, timeout)
            self.io.shutterTriggersFrame = False

        self.__program          = self.PROGRAM_FRAME_TRIG
        self.__disableFrameTrig = False

        logging.debug('programTriggerFrames - flip')
        self.runProgram(timeout=timeout, prog=[
            self.PRSTN + t2Time,                                  # Delay before ABN goes low (must be t2 time)
            self.ABN + self.PRSTN + self.TIMING_WAIT_FOR_ACTIVE,  # Assert ABN until the trigger rises.
            self.ABN + self.PRSTN + (frameTime - integrationTime - t2Time),     # ABN falls
            self.IODRIVE + self.PRSTN + (integrationTime - self.__txnWidth),    # ABN raises
            self.IODRIVE + self.PRSTN + self.TXN + self.__txnWidth,             # TXN falls
            self.TIMING_RESTART                                                 # TXN raises and cycle restarts
        ])
        self.io.shutterTriggersFrame = True
        
    def programStandard(self, frameTime, integrationTime, t2Time=17, disableFrameTrig=True, readoutTime=90000, timeout=0.01):
        frameTime       = int(frameTime)
        integrationTime = int(integrationTime)
        t2Time          = int(t2Time)
        abnTime         = frameTime - integrationTime - t2Time
        if (frameTime <= integrationTime):
            logging.error("frameTime (%d) must be longer than integrationTime (%d)", frameTime, integrationTime)
            integrationTime = int(frameTime * 0.95)

        logging.debug('ProgramStandard: %d, %d', frameTime, integrationTime)

        origShutterTriggersFrame = self.io.shutterTriggersFrame
        if self.__program != self.PROGRAM_STANDARD:
            # make sure the readout completes
            self.programInterm(readoutTime, timeout)
            self.io.shutterTriggersFrame = False

        self.__program          = self.PROGRAM_STANDARD
        self.__disableFrameTrig = disableFrameTrig
        
        # Program preamble: delay before ABN falling (t2 time)
        prog = [ self.PRSTN + t2Time ]
        prog.append(self.ABN + self.PRSTN + (frameTime - integrationTime - t2Time))     # ABN falls
        prog.append(self.IODRIVE + self.PRSTN + (integrationTime - self.__txnWidth))    # ABN raises
        prog.append(self.IODRIVE + self.PRSTN + self.TXN + self.__txnWidth)             # TXN falls
        prog.append(self.TIMING_RESTART)                              # TXN raises and cycle restarts

        logging.debug('programStandard - flip')
        self.runProgram(prog, timeout)
        self.io.shutterTriggersFrame = origShutterTriggersFrame
