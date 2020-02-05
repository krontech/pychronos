import pychronos
import time
import logging
from pychronos.regmaps import timing, ioInterface

class lux1310timing(timing):
    TIMING_HZ = 90000000

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
        self.__t2time           = 17
        self.__txnWidth         = 50
        
        self.io = ioInterface()

    def reset(self):
        if self.version >= 1:
            self.timingEnable = True
            self.wavetableLatch = True

    def programInterm(self):
        self.timingRun = False
        self.timingBreak = True
        self.__program = self.PROGRAM_NONE

    def programShutterGating(self, timeout=0.01):
        logging.debug('programShutterGating')

        # Stop the the timing program.
        self.timingRun = False
        self.timingBreak = True
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
        
    def programTriggerFrames(self, frameTime, integrationTime, timeout=0.01):
        frameTime       = int(frameTime)
        integrationTime = int(integrationTime)
        if (frameTime <= integrationTime):
            logging.error("frameTime (%d) must be longer than integrationTime (%d)", frameTime, integrationTime)
            integrationTime = int(frameTime * 0.95)
        
        logging.debug('TriggerFrames: %d, %d', frameTime, integrationTime)

        # If we are already running this program, just update the arguments.
        if (self.__program == self.PROGRAM_FRAME_TRIG):
            self.timingRun = False
            self.operands[0] = integrationTime - self.__txnWidth
            self.operands[1] = frameTime - integrationTime - self.__t2time
            self.timingRun = True
            return

        # Stop the the timing program.
        self.timingRun = False
        self.timingBreak = True
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
        
    def programStandard(self, frameTime, integrationTime, timeout=0.01):
        frameTime       = int(frameTime)
        integrationTime = int(integrationTime)
        if (frameTime <= integrationTime):
            logging.error("frameTime (%d) must be longer than integrationTime (%d)", frameTime, integrationTime)
            integrationTime = int(frameTime * 0.95)

        logging.debug('ProgramStandard: %d, %d', frameTime, integrationTime)

        # If we are already running this program, just update the arguments.
        if (self.__program == self.PROGRAM_STANDARD):
            self.timingRun = False
            self.operands[0] = integrationTime - self.__txnWidth
            self.operands[1] = frameTime - integrationTime - self.__t2time
            self.timingRun = True
            return

        # Stop the the timing program.
        self.timingRun = False
        self.timingBreak = True
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
    
    def programHDR_2slope(self, frameTime, integration1, integration2, timeout=0.01):
        if (integration1 + integration2 + self.__t2time + 50) > frameTime:
            logging.error("frameTime (%d) must be longer than integrationTime (%d)", frameTime, integrationTime)
            integrationTime = frameTime * 0.95

        logging.debug('ProgramHDR_2slope: %d, %d, %d', frameTime, integration1, integration2)

        # Stop the the timing program.
        self.timingRun = False
        self.timingBreak = True
        self.__program = self.PROGRAM_STANDARD
        
        #... hmm... need to set up some other stuff too
        logging.debug('programHDR-2slope - flip')
        self.runProgram(timeout=timeout, prog=[
            self.NONE  + self.__t2time,
            self.ABN   + 50,
            self.NONE  + int(integration1) - (15+45),
            self.PRSTN + 15,                     # PRSTN \__ to TXN \__
            self.PRSTN + self.TXN + 45,          # TXN ___
            self.PRSTN + 60,                     # TXN __/ to TXN \__
            self.TXN   + int(integration2) - 15
        ])
        self.io.shutterTriggersFrame = False

    def programHDR_3slope(self, frameTime, integration1, integration2, integration3, timeout=0.01):
        if (integration1 + integration2 + integration3 + self.__t2time + 50) > frameTime:
            raise ValueError("frameTime (%d) must be longer than total integrationTime (%d)" % (frameTime, integrationTime))

        logging.debug('ProgramHDR_3slope: %d, %d, %d, %d', frameTime, integration1, integration2, integration3)

        # Stop the the timing program.
        self.timingRun = False
        self.timingBreak = True
        self.__program = self.PROGRAM_STANDARD

        #... hmm... need to set up some other stuff too
        logging.debug('programHDR-3slope - flip')
        self.runProgram(timeout=timeout, prog=[
            self.NONE  + self.__t2time,
            self.ABN   + 50,
            self.NONE  + int(integration1) - (15+45),
            self.PRSTN + 15,                     # PRSTN \__ to TXN \__
            self.PRSTN + self.TXN + 45,          # TXN ___
            self.PRSTN + 60,                     # TXN __/ to TXN \__
            self.PRSTN + self.TXN + int(integration2) - 60,
            self.PRSTN + 15,
            self.TXN   + int(integration3) - 15
        ])
        self.io.shutterTriggersFrame = False
