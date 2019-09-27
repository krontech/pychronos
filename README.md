PyChronos
=========

This project includes the python bindings and camera control software for the
Chronos High Speed Cameras. The camera uses an FPGA to handle the recording of
high-speed video, storage of the video data and playback of video to the CPU.
This project mainly deals with the task of operating the FPGA, image sensor,
and the recording of video into video data. Reading video data out of the FPGA
and rendering it to the various display interfaces is managed by the 
[chronos-cli](https://github.com/krontech/chronos-cli) project.

This project includes several major components:
 * `libpychronos`: C-to-Python bindings that provide an abstraction for the FPGA
   and other hardware assets on the Camera. This is an internal module that you
   shouldn't need to access directly.
 * `pychronos`: A python module which provides a collection of classes and sub-
   modules for the various parts of the camera. Notable groups include:
   * `regmaps`: Classes which represent various register groups in the FPGA.
   * `sensors`: Classes which represent the image sensors supported on the camera.
   * `camera`: A class which represents the state machine of the camera.
 * `cam_control`: A D-Bus binding program which exports an API to manipulate
   the `camera` class.


Building and Installing
-----------------------
**To build the `pychronos` module**, you will need the `python3-dev` and `python3-dbus`
packages for your camera's operating system. Once installed, you can use the `setup.py`
script to build and install the module on your camera, using the following commands:
 * `python3 setup.py build` to build the pychronos package from your checkout.
 * `python3 setup.py install` to install the pychronos package on your camera.

**For quick development**, you can also use `make inplace` to generate an in-place
version of the `pychronos` package that can be imported from the root of the
project checkout without installing to the system.

As much as we love backwards compatibility, the `pychronos` module required Python
version 3.4 and newer. This is due to extensive use of generator functions to handle
cooperative multithreading for long-running tasks.

**To build the documentation**, you will need the following prereqs installed on your
computer. For example, on linux,
```
sudo apt install python3 build-essential python3-sphinx sphinx-doc python3-numpy
```

**To run the API webserver**, you will need the following additional packages.
They can be installed on the camera by running
```
apt install python3-twisted python3-txdbus
```

In the base folder of the project, you can then build the documentation. This
build will be output to the `~/docs` folder, so that [github
pages](https://krontech.github.io/pychronos/) recognises it and
serves it to the world.
```
make inplace
cd docs_src
make html #-OR-
make watch html #for development
```



Chronos Control Interface
=========================
The D-Bus API to the `cam_control` program is expressed as a set of parameters most
of which can be queried via a `get` command, updated via a `set` command. Or, when
updated internally by the camera, may emit a `notify` signal. For each parameter
listed, the `G`, `S`, and `N` columns indicate which of these operations a parameter
supports.

Some settings can be expressed in mutiple encodings, in which case there are multiple
parameters which all set the same underlying value. Each of these encodings support
the `get` and `set` methods but typically only one will emit a `notify` signal on change.
As an example, the exposure can be expressed either as a unit of time via the `exposurePeriod`
parameter, as a ratio between minimum and maximum via the `exposurePercent` parameter,
or as a fraction of the frame period via the `shutterAngle`. Setting exposure via any
of these parameters is allowed, but only the `exposurePeriod` will be reported in a
`notify` signal.

The parameters are further divided into the following subgroups:
 * Lens Control Group
 * Exposure Control Group
 * Gain Control Group
 * Display Group
 * Camera Information Group
 * Sensor Information Group
 * Camera Status Group
 * Color Space Group
 * Recording Group

Each parameter is marked with the following flags:

 * `G`: The parameter's current value can be queried via the `get` command.
 * `S`: The parameter's value can be updated via the `set` command.
 * `N`: Changes to the parameter's value will be reported via the `notify` signal.
 * `x`: The parameter is planned, but not yet implemented.

Each parameter also defines a type as follows:

| API Type | D-Bus Signatures   | Python Types | Description
|:---------|:-------------------|:-------------|:-----------
| `bool`   | `b`                | `boolean`    | Either `true` or `false`.
| `float`  | `t`                | `float`      | Floating-point number.
| `int`    | `i`                | `int`        | Integer type, supporting up to 32-bit precision.
| `enum`   | `s`                | `str`        | The description of each type must specify the allowed values.
| `array`  | `ad`               | `list`       | An array of floating point values.
| `string` | `s`                | `str`        | A character string, which should support UTF-8 encoding.
| `dict`   | `a{sv}`            | `dict`       | An array of name/value pairs. Values may contain any type (including another `dict`).

### Focus Control Parameters
This is pure speculation - nothing is really implemented here until we get
electronic lens control up and running.

| Parameter         | G | S | N | Type   | Min   | Max   | Description
|:------------------|:--|:--|:--|:-------|:------|:------|:-----------
| `focusPercent`    | x | x | x | float  | 0.0   | 100.0 | 0.0 for the nearest possible focus and 100.0 for furthest focus (infinity).
| `focusNormalized` | x | x |   | float  | 0.0   | 1.0   | 0.0 for the nearest possible focus and 1.0 for furthest focus (infinity).
| `focusDistance`   | x | x |   | float  |       |       | Distance to the focus subject in meters.
| `apertureFStop`   | x | x | x | float  |       |       | Aperture value where Fnumber = sqrt(2^AV).
| `aperturePercent` | x | x |   | float  | 0.0   | 100.0 | Aperture size as a percentage from 0 (smallest), to 100 (widest).

### Exposure Control Parameters
| Parameter            | G | S | N | Type   | Min   | Max   | Description
|:---------------------|:--|:--|:--|:-------|:------|:------|:-----------
| `exposurePeriod`     |`G`|`S`|`N`| int    |       |       | Exposure time in nanoseconds.
| `exposurePercent`    |`G`|`S`|   | float  | 0.0   | 100.0 | Exposure time as a percentage of way between the minimum (0%) and maximum (100%) exposures, at the current resolution.
| `exposureNormalized` |`G`|`S`|   | float  | 0.0   | 1.0   | As with `exposurePercent`, but between 0 and 1 instead of 0 and 100.
| `shutterAngle`       |`G`|`S`|   | int    | 0     | 36000 | Exposure time relative to frame period in hundredths of degrees.
| `exposureMin`        |`G`|   |`N`| int    |       |       | Minimum exposure time at the current resolution and frame period.
| `exposureMax`        |`G`|   |`N`| int    |       |       | Maximum exposure time at the current resolution and frame period.
| `exposureMode`       |`G`|`G`|`G`| enum   |       |       | Frame exposure mode as one of `normal`, `frameTrigger`, `shutterGating`, `hdr2slope`, `hdr3slope`.

### Gain Control Parameters
| Parameter         | G | S | N | Type   | Min   | Max   | Description
|:------------------|:--|:--|:--|:-------|:------|:------|:-----------
| `currentIso`      |`G`|`S`|   | int    |       |       | ISO number of the sensor at the current gain setting.
| `currentGain`     |`G`|`S`|`N`| int    |       |       | The gain as a multiplier of the `sensorIso` parameter.

### Display Parameters
These API parameters are proxy values for the equivalent parameters in the D-Bus
API to the `chronos-cli` program.

| Parameter           | G | S | N | Type   | Min   | Max   | Description
|:--------------------|:--|:--|:--|:-------|:------|:------|:-----------
| `overlayEnable`     |`G`|`x`|`x`| bool   |       |       |
| `overlayFormat`     |`G`|`x`|`x`| string |       |       | A `printf`-style format string to set the overlay text.
| `zebraLevel`        |`x`|`x`|`x`| float  | 0.0   | 1.0   | Fraction of the pixel's full scale value at which to apply zebra stripes.
| `focusPeakingLevel` |`x`|`x`|`x`| float  | 0.0   | 1.0   | Focus peaking edge detection sensitivity (0 to disable, or 1.0 for maximum).
| `focusPeakingColor` |`G`|`x`|`x`| enum   |       |       | One of Red, Green, Blue, Cyan, Magenta, Yellow, White and Black.
| `videoState`        |`G`|   |`x`| enum   |       |       | One of `paused`, `live`, `playback` or `filesave`.
| `playbackRate`      |`G`|`x`|`x`| int    |       |       | Framerate for plabyack when `videoState` is in `playback`.
| `playbackPosition`  |`G`|`S`|   | int    |       |       | Current frame number being displayed.
| `playbackStart`     |`G`|`S`|`N`| int    |       |       | Initial frame to display when `videoStart` enters `playback`.
| `playbackLength`    |`G`|`S`|`N`| int    |       |       | Number of frames to play back before returning to `playbackStart`.

### Camera Info Parameters
| Parameter         | G | S | N | Type   | Min   | Max   | Description
|:------------------|:--|:--|:--|:-------|:------|:------|:-----------
|`cameraApiVersion` |`G`|   |   | string |       |       | The string "0.9" for this release of the API specification.
|`cameraFpgaVersion`|`G`|   |   | string |       |       | The major and minor version numbers of the FPGA image.
|`cameraMemoryGB`   |`G`|   |   | float  |       |       | Amount of RAM installed, in units of `GiB`.
|`cameraModel`      |`G`|   |   | string |       |       | Camera model number (eg: "CR14-1.0").
|`cameraSerial`     |`G`|   |   | string |       |       | Camera unique serial number.
|`cameraDescription`|`G`|`S`|`N`| string |       |       | User description of camera.
|`cameraIdNumber`   |`G`|`S`|`N`| int    |       |       | User-assigned camera number for ordering and identification.
|`cameraTallyMode`  |`G`|`S`|`N`| enum   |       |       | Control of the recording LEDs as one of `auto`, `off`, `top`, `back` or `on`.

### Sensor Info Parameters
| Parameter           | G | S | N | Type   | Min   | Max   | Description
|:--------------------|:--|:--|:--|:-------|:------|:------|:-----------
|`sensorName`         |`G`|   |   | string |       |       | Descriptive string of the image sensor.
|`sensorColorPattern` |`G`|   |   | string |       |       | String of ‘R’ ‘G’ and ‘B’ that defines the color filter pattern in left-to-right and top-to-bottom order or ‘mono’ for monochrome sensors.
|`sensorBitDepth`     |`G`|   |   | int    |       |       | Number of bits per pixel as recorded by the image sensor.
|`sensorIso`          |`G`|   |   | int    |       |       | Base ISO of the image sensor at a gain of 1x (or 0 dB).
|`sensorMaxGain`      |`G`|   |   | int    |       |       | Maximum gain of the image sensor as a multiple of `sensorIso`.
|`sensorPixelRate`    |`G`|   |   | int    |       |       | Approximate pixel rate of the image sensor in pixels per second.
|`sensorVMax`         |`G`|   |   | int    |       |       | Maximum vertical resolution of the image sensor.
|`sensorVMin`         |`G`|   |   | int    |       |       | Minimum vertical resolution of the image sensor.
|`sensorVIncrement`   |`G`|   |   | int    |       |       | Minimum quantization of vertical resolutions.
|`sensorHMax`         |`G`|   |   | int    |       |       | Maximum horizontal resolution of the image sensor.
|`sensorHMin`         |`G`|   |   | int    |       |       | Minimum horizontal resolution of the image sensor.
|`sensorHIncrement`   |`G`|   |   | int    |       |       | Minimum quantization of horizontal resolutions.
|`sensorVDark`        |`G`|   |   | int    |       |       | Number of vertical dark rows (not included in sensorVMax).

### Camera Status Parameters
| Parameter                                  | G | S | N | Type   | Min   | Max   | Description
|:-------------------------------------------|:--|:--|:--|:-------|:------|:------|:-----------
|`state`                                     |`G`|   |`N`| enum   |       |       | One of `idle`, `recording`, `reset` and others???? TBD.
|`error`                                     |   |   |`x`| string |       |       | Included in a notification dictionary if, and only if, an operation fails due to an error.
|`externalStorage`                           |`G`|   |   | dict   |       |       | Dictionary of dictionaries describing the external storage devices.
|`dateTime`                                  |`G`|`x`|`x`| string |       |       | ISO-8601 formatted date and time string.

### Power and Battery Parameters
| Parameter                                  | G | S | N | Type   | Min   | Max   | Description
|:-------------------------------------------|:--|:--|:--|:-------|:------|:------|:-----------
|`externalPower`                             |`x`|   |`x`| bool   | False | True  | True when the AC adaptor is present, and False when on battery power.
|`batteryPresent`                            |`x`|   |   | bool   | False | True  | True when a battery is present, and False when there is no battery.
|`batteryChargeNormalized`                   |`x`|   |   | float  | 0.0   | 1.0   | Estimated battery charge, with 0.0 being fully depleted and 1.0 being fully charged.
|`batteryChargePercent`                      |`x`|   |   | float  | 0.0   | 100.0 | Estimated battery charge, with 0% being fully depleted and 100% being fully charged.
|`batteryVoltage`                            |`x`|   |   | float  | 0.0   | 14ish | A measure of the power the removable battery is putting out, in volts. A happy battery outputs between 12v and 12.5v. This value is graphed on the battery screen on the Chronos.
|`saveAndPowerDownLowBatteryLevelNormalized` |`x`|`x`|`x`| float  | 0.0   | 1.0   | Equivalent to `saveAndPowerDownLowBatteryLevelPercent`, but based against `batteryChargeNormalized` which is always 1% of `batteryChargePercent`.
|`saveAndPowerDownLowBatteryLevelPercent`    |`x`|`x`|`x`| float  | 0.0   | 100.0 | Turn off the camera if the battery charge level, reported by `batteryChargePercent`, falls below this level. The camera will start saving any recorded footage before it powers down. If this level is too low, the camera may run out of battery and stop before it finishes saving.
|`saveAndPowerDownWhenLowBattery`            |`x`|`x`|`x`| bool   | False | True  | Should the camera try to turn off gracefully when the battery is low? The low level is set by `saveAndPowerDownLowBatteryLevelPercent` (or `saveAndPowerDownLowBatteryLevelNormalized`). The opposite of `powerOnWhenMainsConnected`. See `powerOnWhenMainsConnected` for an example which sets the camera to turn on and off when external power is supplied.
|`powerOnWhenMainsConnected`                 |`x`|`x`|`x`| bool   | False | True  | Set to `True` to have the camera turn itself on when it is plugged in. The inverse of this, turning off when the charger is disconnected, is achieved by setting the camera to turn off at any battery percentage. For example, to make the camera turn off when it is unpowered and turn on when it is powered again - effectively only using the battery to finish saving - you could make the following call: `api.set({ 'powerOnWhenMainsConnected':True, 'saveAndPowerDownWhenLowBattery':True, 'saveAndPowerDownLowBatteryLevelPercent':100.0 })`.
|`backlightEnabled`                          |`x`|`x`|`x`| bool   |       |       | True if the LCD on the back of the camera is lit. Can be set to False to dim the screen and save a small amount of power.

### Camera Network Parameters
| Parameter         | G | S | N | Type   | Min   | Max   | Description
|:------------------|:--|:--|:--|:-------|:------|:------|:-----------
|`networkInterfaces`|`x`|   |   | dict   |       |       | Dictionary of dictionaries describing the network interfaces.
|`networkHostname`  |`G`|`S`|   | string |       |       | Hostname to be used for system calls, DHCP requests, etc.


### Color Space
TODO: As a longer term plan, these parameters should be moved into the display
group and made more a part of the video display system.

| Parameter         | G | S | N | Type   | Min   | Max   | Description
|:------------------|:--|:--|:--|:-------|:------|:------|:-----------
|`wbMatrix`         |`G`|`S`|`N`| array  |       |       | Array of Red, Green, and Blue gain coefficients to achieve white balance.
|`wbCustom`         |`G`|`S`|`N`| array  |       |       | Automatically-generated white balance gain coefficients.
|`wbTemperature`    |`x`|`x`|`x`| int    | 1800  | 10000 | Estimated lighting temperature in degrees Kelvin.
|`colorMatrix`      |`G`|`S`|`N`| array  |       |       | Array of 9 floats describing the 3x3 color matrix from image sensor color space in to sRGB, stored in row-scan order.

### IO Group
| Parameter         | G | S | N | Type   | Min   | Max   | Description
|:------------------|:--|:--|:--|:-------|:------|:------|:-----------
|`ioMapping`        |`G`|`S`|`N`| dict   |       |       | A dictionary of the output mappings
|`ioDelayTime`      |`G`|`S`|   | float  | 0.0   |       | An alias to ioMapping.delay.delayTime

The `ioMapping` dictionary contains the following members:

| Output            | Note | Description
|:------------------|:-----|:-----------
|`io1`              | 1    | Output Driver - IO 1
|`io2`              | 1    | Output Driver - IO 2
|`combOr1`          |      | Combinatorial OR 1 input
|`combOr2`          |      | Combinatorial OR 2 input
|`combOr3`          |      | Combinatorial OR 3 input
|`combAnd`          |      | Combinatorial AND input
|`combXOr`          |      | Combinatorial XOR input
|`delay`            | 2    | Delay source
|`toggleSet`        |      | Toggle block SET source
|`toggleClear`      |      | Toggle block CLEAR source
|`toggleFlip`       |      | Toggle block FLIP source
|`shutter`          | 3    | Shutter source - used for Shutter Gatting (while active, open shutter)
|`io1In`            | 4    | input configuration for io1
|`io2In`            | 4    | input configuration for io2

1 - Parameters for IO drive strength
2 - Parameters for delay function
3 - Parameters for shutter function
4 - Special input configuration

Each of the `Output` blocks is a dictionary that contains:

| Parameter            | Type  | Min | Max | Description
|:---------------------|:------|:----|:----|:-----------
|`source`              | enum  |     |     | Which input is connected to this - see list bellow
|`invert`              | bool  |     |     | If true, invert the input
|`debounce`            | bool  |     |     | If true, a debounce circuit is used
|`driveStrength`       | int   | 0   | 3   | For IO drive pins only - sets the output drive strength (io1: 0, 1mA, 20mA, 21mA; io2: 0, 20mA, 20mA, 20mA)
|`shutterTriggersFrame`| bool  |     |     | If true, shutter signal is forwarded to timing block. If false, this function is disabled
|`delayTime`           | float | 0.0 |     | The time the delay block delays the incoming signal for in seconds. Values of 0.00000001 through 32768.0 are possible with varying internal resolution

The source can be one of the following

| Source               | Description
|:---------------------|:-----------
|`none`                | Always 0
|`io1`                 | Input 1
|`io2`                 | Input 2
|`io3`                 | Isolated input
|`comb`                | Combinatorial block output
|`delay`               | Delay block output
|`toggle`              | Toggle block output
|`shutter`             | Signal from the old timing engine - currently integrating
|`recording`           | Signal from the record sequencer - active while not in live preview mode
|`dispFrame`           | Signal from the record sequencer - pulses on each frame
|`startRec`            | Signal from the record sequencer - pulses when recording starts
|`endRec`              | Signal from the record sequencer - pulses when recording stopped
|`nextSeg`             | Signal from the record sequencer - pulses on new segment
|`timingIo`            | Signal from the new timing engine - adjustable, normally integrating
|`alwaysHigh`          | Always 1


The input configuration has a single parameter:

| Parameter            | type  | Min | Max | Description
|:---------------------|:------|:----|:----|:-----------
|`threshold`           | float | 0.0 | 5.0 | Input threshhold in volts




### Recording Group
| Parameter         | G | S | N | Type   | Min   | Max   | Description
|:------------------|:--|:--|:--|:-------|:------|:------|:-----------
|`recMode`          |`G`|`S`|`N`| enum   |       |       | One of `normal`, `segmented` or `burst`
|`recMaxFrames`     |`G`|`S`|`N`| int    |       |       | Maximum number of frames available for the recording buffer.
|`recSegments`      |`G`|`S`|`N`| int    | 1     |       | Number of memory segments supported when in segmented recording mode.
|`recPreBurst`      |`G`|`S`|`N`| int    | 0     |       | Number of frames leading up to the trigger to record when in gated burst mode.
|`resolution`       |`G`|`S`|`N`| dict   |       |       | Dict describing the resolution settings. For example, `gdbus call --system --dest ca.krontech.chronos.control  --object-path /ca/krontech/chronos/control --method ca.krontech.chronos.control.get "['resolution']"` yields `({'resolution': <{'vOffset': <4>, 'hRes': <1280>, 'vDarkRows': <4>, 'bitDepth': <12>, 'hOffset': <0>, 'minFrameTime': <0.00093492222222222221>, 'vRes': <1020>}>},)`. `bitDepth` is min and max 12 at the moment. The optional `minFrameTime` is used to select the optimal wavetable when switching resolutions. This is useful when dialing the camera back, such as when recording at 500fps full-frame instead of 1000fps.
|`minFramePeriod`   |`G`|   |`N`| int    |       |       | Minimum frame period for the current resolution settings.
|`cameraMaxFrames`  |`G`|   |`N`| int    |       |       | Maximum number of frames the camera's memory can save at the current resolution.
|`framePeriod`      |`G`|`S`|`N`| int    |       |       | Time in nanoseconds to record a single frame (or minimum time for frame sync and shutter gating).
|`frameRate`        |`G`|`S`|   | float  |       |       | Estimated frame rate in frames per second (reciprocal of `framePeriod`)
|`totalFrames`      |`x`|   |   | int    |       |       | Total number of frames in memory recorded by the camera.
|`totalSegments`    |`x`|   |   | int    |       |       | Total number of segments in memory recorded by the camera.

The `resolution` dictionary contains the following members:

| Parameter         | Type  | Description
|:------------------|:------|:-----------
|`hRes`             | int   | Number of active horizontal pixels in each row.
|`vRes`             | int   | Number of active vertical pixels in each column.
|`hOffset`          | int   | Horizontal offset to the left of the sensor to the start of active pixels.
|`vOffset`          | int   | Vertical offset from the top of the sensor to the start of active pixels.
|`vDarkRows`        | int   | Number of optical black rows to read out at the top of the frame.
|`bitDepth`         | int   | Bit depth to sample the image at.
|`minFrameTime`     | float | Minimum frame time at this resolution.

When setting resolution, the `hRes` and `vRes` members are required to be specified. All
other members of the dictionary are optional and may be used to further tune the sensor's
operation if desired.

Control Methods
---------------
In addition to the parameters which can be manipulated to setup the camera,
the API also includes a set of methods which perform state changes. A list
of the supported methods are as follows:

| Method                      | S | Arguments        | State Change | Description
|:----------------------------|:--|:-----------------|:-------------|:-----------
| `get`                       |`G`| array of names   |              | Retrieve one or more parameters from the control API.
| `set`                       |`S`| dict(parameters) |              | Modify one or more parameters in the control API.
| `startAutoWhiteBalance`     |`S`| dict(none)       | `whitebal`   | Take a reference image from the live display and compute the white balance.
| `revertAutoWhiteBalance`    |`S`| none             |              | This copies the contents of `wbCustom` into `wbMatrix`.
| `startAutoFocus`            |   | dict(location)   |              | Attempt to automatically focus the camera on a subject.
| `startCalibration`          |`S`| dict(calTypes)   | varies       | Perform full calibration operations. Dict can have `blackCal`, `analogCal`, ior `zeroTimeBlackCal` set to true or false.
| `startRecording`            |`S`| none             | `recording`  | Begin recording video data to memory.
| `stopRecording`             |`S`| none             | `idle`       | Terimnate recording of video data to memory.
| `flushRecording`            |`S`| none             |              | Flush recoreded video data from memory.
| `startFilesave`             |   | dict             |              | TBD: A proxy for the `filesave` method in the Video API.
| `softTrigger`               |`S`| none             |              | Generate a software trigger event.
| `revertToDefaults`          |   | none             |              | Revert all settings to their default values (with optional parameter overrides).
| `softReset`                 |`S`| none             | `reset`      | Perform a soft reset and initialization of the FPGA and image sensor.
| `getResolutionTimingLimits` |`S`| dict(resolution) |              | Test if a resolution is valid and return the timing limits (framerate) at that resolution. Example: `call --system --dest ca.krontech.chronos.control --object-path /ca/krontech/chronos/control --method ca.krontech.chronos.control.getResolutionTimingLimits "{'hRes': <1280>, 'vRes': <1020>}"` → `({'minFramePeriod': <931277>, 'exposureMin': <1000>, 'cameraMaxFrames': <17542>, 'exposureMax': <925722>},)`. Maximum framerate is `1e9 / minFramePeriod`.

All methods return a dictionary of parameters, normally this will just include
the status dictionary, which minimally includes `state`, but may also include an
`error` parameter in the event that the operation failed due an error.

One exception is the `getResolutionTimingLimits` method, which queries the camera if
the requested resolution is valid. If the resolution settings are valid, then the
dict will return the following values as though the resolution and minimum frame
period had been applied:
 * `cameraMaxFrames`
 * `minFramePeriod`
 * `exposureMin`
 * `exposureMax`

Otherwise, the `getResolutionTimingLimits` method will return a status dictionary with a
parameter of `error` set to "Invalid Resolution"

Control Signals
---------------
When parameter values are changed, either explicity via the `set` method, or
autonomously during the camera's operation, a D-Bus `notify` signal is generated.
The modified parameters are included in a dictionary as the argumenets of the
signal.
