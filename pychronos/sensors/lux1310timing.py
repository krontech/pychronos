import pychronos
import time
import logging
from . import lux1310regs
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
        self.__frameTime        = self.TIMING_HZ // 1000 # 1ms
        self.__integrationTime  = int(self.__frameTime * 0.9)
        self.__t2Time           = 17
        self.__txnWidth         = 50
        self.__disableFrameTrig = False
        self.__disableIoDrive   = False
        self.__nFrames          = 5
        self.__program          = self.PROGRAM_NONE
        
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

    @property
    def frameTime(self):
        logging.debug("frameTime: %d", self.__frameTime)
        return self.__frameTime
    @frameTime.setter
    def frameTime(self, value):
        if self.__program == self.PROGRAM_STANDARD:
            self.programStandard(value, self.__integrationTime, self.__t2Time, self.__disableFrameTrig, self.__disableIoDrive)
        elif self.__program == self.PROGRAM_FRAME_TRIG:
            self.programTriggerFrames(value, self.__integrationTime, self.__t2Time, self.__disableIoDrive)
        elif self.__program == self.PROGRAM_N_FRAME_TRIG:
            self.programTriggerNFrames(value, self.__integrationTime, self.__nFrames, self.__t2Time, self.__disableIoDrive)
        elif self.__program == self.PROGRAM_SHUTTER_GATING:
            pass
            
    @property
    def integrationTime(self):
        logging.debug("integrationTime: %d", self.__integrationTime)
        return self.__integrationTime
    @integrationTime.setter
    def integrationTime(self, value):
        if self.__program == self.PROGRAM_STANDARD:
            self.programStandard(self.__frameTime, value, self.__t2Time, self.__disableFrameTrig, self.__disableIoDrive)
        elif self.__program == self.PROGRAM_FRAME_TRIG:
            self.programTriggerFrames(self.__frameTime, value, self.__t2Time, self.__disableIoDrive)
        elif self.__program == self.PROGRAM_N_FRAME_TRIG:
            self.programTriggerNFrames(self.__frameTime, value, self.__nFrames, self.__t2Time, self.__disableIoDrive)
        elif self.__program == self.PROGRAM_SHUTTER_GATING:
            pass

    def programInterm(self, readoutTime=90000, timeout=0.01):
        """This programs an blank program which will make sure the readout
        section has enough time to complete readout before switching to a
        new timing program.
        This has all IO disabled including the one driving the io block...
        """
        logging.debug('programInterm - flip')
        self.__program = self.PROGRAM_NONE
        self.runProgram(prog=[self.NONE + readoutTime, self.NONE | self.TIMING_RESTART], timeout=timeout)

    def programShutterGating(self, t2Time=17, readoutTime=90000, timeout=0.01):
        self.__program = self.PROGRAM_SHUTTER_GATING
        self.__t2Time           = t2Time
        self.__disableFrameTrig = False

        logging.debug('programShutterGating')
        # make sure the readout completes
        self.programInterm(readoutTime, timeout)
        self.io.shutterTriggersFrame = False
        
        prog = [self.NONE + t2Time]
        prog.append(self.ABN                 | self.TIMING_WAIT_FOR_ACTIVE)
        prog.append(self.IODRIVE | self.NONE | self.TIMING_WAIT_FOR_INACTIVE)
        prog.append(self.TXN  | 0x31)
        if self.useMinLinesWait:
            prog.append(self.TXN | self.TIMING_WAIT_FOR_NLINES)
        prog.append(self.NONE | self.TIMING_RESTART)
        
        logging.debug('programShutterGating - flip')
        self.runProgram(prog, timeout)
        self.io.shutterTriggersFrame = True
        
    def programTriggerFrames(self, frameTime, integrationTime, t2Time=17, disableIoDrive=False, readoutTime=90000, timeout=0.01):
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
        self.__frameTime        = frameTime
        self.__integrationTime  = integrationTime
        self.__t2Time           = t2Time
        self.__disableFrameTrig = False
        self.__disableIoDrive   = disableIoDrive


        if disableIoDrive: ioDrive = 0
        else:              ioDrive = self.IODRIVE

        logging.debug('programTriggerFrames - flip')
        self.runProgram(timeout=timeout, prog=[
            self.NONE + t2Time,                      # period before ABN goes low (must be t2 time)
            # ABN Falls here
            self.ABN | self.TIMING_WAIT_FOR_ACTIVE,  # (hence why the hold happens after the first command)
            self.ABN + (frameTime - integrationTime),
            ioDrive + self.NONE + (integrationTime), # ABN raises
            ioDrive + self.TXN + 0x31,               # TXN falls
            self.TIMING_RESTART                      # TXN raises and cycle restarts
        ])
        self.io.shutterTriggersFrame = True
        
    def programStandard(self, frameTime, integrationTime, t2Time=17, disableFrameTrig=True, disableIoDrive=False, readoutTime=90000, timeout=0.01):
        frameTime       = int(frameTime)
        integrationTime = int(integrationTime)
        t2Time          = int(t2Time)
        if (frameTime <= integrationTime):
            logging.error("frameTime (%d) must be longer than integrationTime (%d)", frameTime, integrationTime)
            integrationTime = int(frameTime * 0.95)

        logging.debug('ProgramStandard: %d, %d', frameTime, integrationTime)

        # Special case: if just changing the frame and integration time, then we can shortcut...
        if (self.__program == self.PROGRAM_STANDARD) and (self.__t2Time == t2Time):
            logging.debug('ProgramStandard: bypassing flip')
            self.__frameTime = frameTime
            self.__integrationTime = integrationTime
            self.inhibitTiming = True
            self.operands[0] = integrationTime - self.__txnWidth
            self.operands[1] = frameTime - integrationTime - self.__t2Time
            self.inhibitTiming = False
            return

        origShutterTriggersFrame = self.io.shutterTriggersFrame
        if self.__program != self.PROGRAM_STANDARD:
            # make sure the readout completes
            self.programInterm(readoutTime, timeout)
            self.io.shutterTriggersFrame = False

        self.__program          = self.PROGRAM_STANDARD
        self.__frameTime        = frameTime
        self.__integrationTime  = integrationTime
        self.__t2Time           = t2Time
        self.__disableFrameTrig = disableFrameTrig
        self.__disableIoDrive   = disableIoDrive

        if disableIoDrive: ioDrive = 0
        else:              ioDrive = self.IODRIVE

        # Standard program uses the following arguments.
        # R0 = Exposure time - TXN width.
        # R1 = Frame period - Exposure Time - Setup Time.
        self.operands[0] = integrationTime - self.__txnWidth
        self.operands[1] = frameTime - integrationTime - self.__t2Time
        logging.debug('ProgramStandard: ABN width = %d', frameTime - integrationTime - self.__txnWidth - self.__t2Time)

        # Program preamble: delay before ABN falling (t2 time)
        prog = [
            self.NONE +    t2Time,                              # Setup time until linevalid.
            self.ABN +     self.OPCODE_DELAY_REGISTER + 1,      # Assert ABN for argument in R1
            self.IODRIVE + self.OPCODE_DELAY_REGISTER + 0,      # Release ABN for argument in R0
            self.IODRIVE + self.TXN + self.__txnWidth,          # TXN falls
            self.TIMING_RESTART                                 # TXN raises and cycle restarts
        ]

        logging.debug('programStandard - flip')
        self.runProgram(prog, timeout)
        self.io.shutterTriggersFrame = origShutterTriggersFrame

                
    def programSpecial(self, frameTime, integrationTime, t2Time, readoutTime=90000, timeout=0.01):
        wavetableTime = self.pulsedAbnHighPeriod + self.pulsedAbnLowPeriod

        preFrameTime = (frameTime - integrationTime) // (wavetableTime+2)
        preFrameTime *= (wavetableTime+2)
        preFrameTime -= 16
        
        logging.debug('ProgramSpecial: %d, %d', frameTime, integrationTime)
        # make sure the readout completes
        self.programInterm(readoutTime, timeout)
        origShutterTriggersFrame = self.io.shutterTriggersFrame
        self.io.shutterTriggersFrame = False

        logging.debug('programSpecial - flip')
        self.runProgram(timeout=timeout, prog=[
            self.NONE + t2Time,
            self.ABN + self.TIMING_WAIT_FOR_INACTIVE,
            self.ABN + self.TIMING_WAIT_FOR_ACTIVE,
            self.ABN + preFrameTime - (wavetableTime+2),
            self.ABN + (wavetableTime+2),
            self.IODRIVE + self.NONE + (integrationTime),
            self.IODRIVE + self.TXN + 6,
            self.NONE + self.TIMING_RESTART
        ])
        self.io.shutterTriggersFrame = origShutterTriggersFrame

    def programHDR_2slope(self, frameTime, integration1, integration2, t2Time=17, VDR1=2.5, VDR2 = 2.0, readoutTime=90000, timeout=0.01):
        if (integration1 + integration2 + t2Time + 50) > frameTime:
            logging.error("frameTime (%d) must be longer than integrationTime (%d)", frameTime, integrationTime)
            integrationTime = frameTime * 0.95

        logging.debug('ProgramHDR_2slope: %d, %d, %d', frameTime, integration1, integration2)

        # make sure the readout completes
        self.programInterm(readoutTime, timeout)
        origShutterTriggersFrame = self.io.shutterTriggersFrame
        self.io.shutterTriggersFrame = False
        
        #... hmm... need to set up some other stuff too
        logging.debug('programHDR-2slope - flip')
        self.runProgram(timeout=timeout, prog=[
            self.NONE  + t2Time,
            self.ABN   + self.TIMING_WAIT_FOR_INACTIVE,
            self.ABN   + self.TIMING_WAIT_FOR_ACTIVE,
            self.ABN   + 50,
            self.NONE  + int(integration1) - (15+45),
            self.PRSTN + 15,                     # PRSTN \__ to TXN \__
            self.PRSTN + self.TXN + 45,          # TXN ___
            self.PRSTN + 60,                     # TXN __/ to TXN \__
            self.TXN   + int(integration2) - 15
        ])
        self.io.shutterTriggersFrame = origShutterTriggersFrame

    def programHDR_3slope(self, frameTime, integration1, integration2, integration3, t2Time=17, VDR1=2.5, VDR2 = 2.0, readoutTime=90000, timeout=0.01):
        if (integration1 + integration2 + integration3 + t2Time + 50) > frameTime:
            raise ValueError("frameTime (%d) must be longer than total integrationTime (%d)" % (frameTime, integrationTime))

        logging.debug('ProgramHDR_3slope: %d, %d, %d, %d', frameTime, integration1, integration2, integration3)
        # make sure the readout completes
        self.programInterm(readoutTime, timeout)
        origShutterTriggersFrame = self.io.shutterTriggersFrame
        self.io.shutterTriggersFrame = False

        #... hmm... need to set up some other stuff too
        logging.debug('programHDR-3slope - flip')
        self.runProgram(timeout=timeout, prog=[
            self.NONE  + t2Time,
            self.ABN   + 50,
            self.NONE  + int(integration1) - (15+45),
            self.PRSTN + 15,                     # PRSTN \__ to TXN \__
            self.PRSTN + self.TXN + 45,          # TXN ___
            self.PRSTN + 60,                     # TXN __/ to TXN \__
            self.PRSTN + self.TXN + int(integration2) - 60,
            self.PRSTN + 15,
            self.TXN   + int(integration3) - 15
        ])
        self.io.shutterTriggersFrame = origShutterTriggersFrame


    def programTriggerNFrames(self, frameTime, integrationTime, nFrames=5, t2Time=17, disableIoDrive=False, readoutTime=90000, timeout=0.01):
        frameTime       = int(frameTime)
        integrationTime = int(integrationTime)
        t2Time          = int(t2Time)
        if (frameTime <= integrationTime):
            logging.error("frameTime (%d) must be longer than integrationTime (%d)", frameTime, integrationTime)
            integrationTime = frameTime * 0.95

        logging.debug('ProgramTriggerNFrames: %d, %d', frameTime, integrationTime)
        # make sure the readout completes
        self.programInterm(readoutTime, timeout)
        origShutterTriggersFrame = self.io.shutterTriggersFrame
        self.io.shutterTriggersFrame = False

        self.__program          = self.PROGRAM_N_FRAME_TRIG
        self.__frameTime        = frameTime
        self.__integrationTime  = integrationTime
        self.__t2Time           = t2Time
        self.__disableFrameTrig = False
        self.__disableIoDrive   = disableIoDrive
        self.__nFrames          = nFrames

        if disableIoDrive: ioDrive = 0
        else:              ioDrive = self.IODRIVE
        
        # Program preamble - delay ABN by t2 time, then wait for trigger active.
        prog = [
            self.NONE + t2Time,
            self.ABN | self.TIMING_WAIT_FOR_ACTIVE
        ]
        # For each frame: pulse ABN, delay the exposure time and pulse TXN.
        for i in range(nFrames):
            prog.append(self.ABN + (frameTime - integrationTime))
            prog.append(ioDrive + self.NONE + (integrationTime)) # ABN raises
            prog.append(ioDrive + self.TXN + 0x31)               # TXN falls
        # Terminate the program
        prog.append(self.TIMING_RESTART)
        
        logging.debug('programTriggerNFrames - flip')
        self.runProgram(prog, timeout)
        self.io.shutterTriggersFrame = origShutterTriggersFrame


if __name__ == '__main__':
    timing = lux1310timing()
    lux    = lux1310regs()
    timing.inhibitTiming = False
    waveTableLength = lux.regRdoutDly
    print ("wavetable: %d" % waveTableLength)
    t2Time = lux.regSofDelay + 3
    print ("t2Time: %d" % t2Time)
    timing.setPulsedPattern(waveTableLength, hSync=1)
    timing.useAbnPulsedMode = False
    
    while (True):
        time.sleep(0.1)
        frameTime       = int(lux.framePeriod * 0.9)
        integrationTime = int(lux.intTime * 0.9)

        timing.programStandard(frameTime, integrationTime, t2Time=t2Time)
        #timing.programSpecial(frameTime, integrationTime, t2Time=t2Time)
        #timing.programHDR_3slope(frameTime, int(integrationTime * 0.9), int(integrationTime * 0.09), int(integrationTime * 0.01), VDR1=2.5, VDR2=2.0)
        
