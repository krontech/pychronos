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

# Enumeration type for recording modes.
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