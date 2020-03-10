# Common types that might be useful across the camera.
from enum import Enum

class TallyModes(Enum):
    """Mode in which the recording LEDs should operate.

    Attributes:
        off: All recording LEDs on the camera are turned off.
        on: All recording LEDs on the camera are turned on.
        auto: The recording LEDs on the camera are on whenever the `status` property is equal to 'recording'.
    """
    off = 0
    on = 1
    auto = 2

class RecModes(Enum):
    """Mode in which the recording sequencer stores frames into video memory.

    Attributes:
        normal: Frames are saved continuously into a ring buffer of up to `recMaxFrames` in length
            until the recording is terminated by the recording end trigger.
        segmented: Up to `recMaxFrames` of video memory is divided into `recSegments` number of of
            ring buffers. The camera saves video into one ring buffer at a time, switching to the
            next ring buffer at each recording trigger.
        burst: Each rising edge of the recording trigger starts a new segment in video memory,
            with frames being saved for as long as the recording trigger is active.
    """
    normal = 0
    segmented = 1
    burst = 2

class ExposureModes(Enum):
    """Mode in which frame and exposure timing should operate.
    
    Attributes:
        normal: Frame and exposure timing operate on fixed periods and are free-running.
        frameTrigger: Frame starts on the rising edge of the trigger signal, and exposes
            the frame for `exposurePeriod` nanoseconds. Once readout completes, the camera
            will wait for another rising edge before starting the next frame. In this mode,
            the `framePeriod` property constrains the minimum time between frames.
        shutterGating: Frame starts on the rising edge of the trigger signal, and exposes
            the frame for as long as the trigger signal is held high, regardless of the
            `exposurePeriod` property. Once readout completes, the camera will wait for
            another rising edge before starting the next frame. When in this mode, the
            `framePeriod` property constrains the minimum time between frames. 
    """
    normal = 0
    frameTrigger = 1
    shutterGating = 2
    # These two are not documented, because they're experimental for now.
    # They seem to work fine on the LUX1310, but we need FPGA help to process
    # the non-linear data from the sensor.
    hdr2slope = 3
    hdr3slope = 4
