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
        self.__program          = self.PROGRAM_NONE
        self.__t2time           = 13 # SOF_DELAY+3, I think.
        self.__txnWidth         = 50
        
        self.io = ioInterface()

    def reset(self):
        if self.version >= 1:
            self.timingEnable = True
            self.wavetableLatch = True
            self.signalConfig = 0xB00

    def programInterm(self, readoutTime=90000, timeout=0.01):
        self.programRun = False
        self.programBreak = True
        self.__program = self.PROGRAM_NONE

    def programShutterGating(self, readoutTime=90000, timeout=0.01):
        logging.debug('programShutterGating')

        # Stop the the timing program.
        self.programRun = False
        self.programBreak = True
        self.__program = self.PROGRAM_SHUTTER_GATING
        
        # Load and execute the shutter gating program.
        self.runProgram(timeout=timeout, prog=[
            self.NONE +               self.__t2time,                 # Setup time until linevalid.
            self.ABN +                self.TIMING_WAIT_FOR_ACTIVE,   # Assert ABN until trigger goes high.
            self.ABN + self.IODRIVE + self.__txnWidth,               # Keep ABN asserted to adjust for TXN width.
            self.IODRIVE +            self.TIMING_WAIT_FOR_INACTIVE, # Release ABN until trigger goes low.
            self.TXN +                self.__txnWidth,               # Assert TXN.
            self.TIMING_RESTART                                      # TXN raises and cycle restarts
        ])
        self.io.shutterTriggersFrame = True
        
    def programTriggerFrames(self, frameTime, integrationTime, readoutTime=90000, timeout=0.01):
        frameTime       = int(frameTime)
        integrationTime = int(integrationTime)
        if (frameTime <= integrationTime):
            logging.error("frameTime (%d) must be longer than integrationTime (%d)", frameTime, integrationTime)
            integrationTime = int(frameTime * 0.95)
        
        logging.debug('TriggerFrames: %d, %d', frameTime, integrationTime)

        # If we are already running this program, just update the arguments.
        if (self.__program == self.PROGRAM_FRAME_TRIG):
            self.programRun = False
            self.operands[0] = integrationTime - self.__txnWidth
            self.operands[1] = frameTime - integrationTime - self.__t2time
            self.programRun = False
            return

        # Stop the the timing program.
        self.programRun = False
        self.programBreak = True
        self.__program = self.PROGRAM_FRAME_TRIG

        # Standard program uses the following arguments.
        # R0 = Exposure time - TXN width.
        # R1 = Frame period - Exposure Time - Setup Time.
        self.operands[0] = integrationTime - self.__txnWidth
        self.operands[1] = frameTime - integrationTime - self.__t2time
        self.runProgram(timeout=timeout, prog=[
            self.NONE +    self.__t2time,                       # Setup time until linevalid.
            self.ABN +     self.TIMING_WAIT_FOR_ACTIVE,         # Assert ABN and wait for the trigger to go high.
            self.ABN +     self.OPCODE_DELAY_REGISTER + 1,      # Assert ABN for argument in R1
            self.IODRIVE + self.OPCODE_DELAY_REGISTER + 0,      # Release ABN for argument in R0
            self.IODRIVE + self.TXN + self.__txnWidth,          # TXN falls
            self.TIMING_RESTART                                 # TXN raises and cycle restarts
        ])
        self.io.shutterTriggersFrame = True
        
    def programStandard(self, frameTime, integrationTime, readoutTime=90000, timeout=0.01):
        frameTime       = int(frameTime)
        integrationTime = int(integrationTime)
        if (frameTime <= integrationTime):
            logging.error("frameTime (%d) must be longer than integrationTime (%d)", frameTime, integrationTime)
            integrationTime = int(frameTime * 0.95)

        logging.debug('ProgramStandard: %d, %d', frameTime, integrationTime)

        # If we are already running this program, just update the arguments.
        if (self.__program == self.PROGRAM_STANDARD):
            self.programRun = False
            self.operands[0] = integrationTime - self.__txnWidth
            self.operands[1] = frameTime - integrationTime - self.__t2time
            self.programRun = False
            return

        # Stop the the timing program.
        self.programRun = False
        self.programBreak = True
        self.__program = self.PROGRAM_STANDARD
        
        # Standard program uses the following arguments.
        # R0 = Exposure time - TXN width.
        # R1 = Frame period - Exposure Time - Setup Time.
        self.operands[0] = integrationTime - self.__txnWidth
        self.operands[1] = frameTime - integrationTime - self.__t2time
        self.runProgram(timeout=timeout, prog=[
            self.NONE +    self.__t2time,                       # Setup time until linevalid.
            self.ABN +     self.OPCODE_DELAY_REGISTER + 1,      # Assert ABN for argument in R1
            self.IODRIVE + self.OPCODE_DELAY_REGISTER + 0,      # Release ABN for argument in R0
            self.IODRIVE + self.TXN + self.__txnWidth,          # TXN falls
            self.TIMING_RESTART                                 # TXN raises and cycle restarts
        ])
        self.io.shutterTriggersFrame = False
