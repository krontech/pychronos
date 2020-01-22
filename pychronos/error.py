
class CameraError(RuntimeError):
    pass

class CalibrationError(CameraError):
    pass

class SignalClippingError(CameraError):
    pass

class LowSignalError(CameraError):
    pass
