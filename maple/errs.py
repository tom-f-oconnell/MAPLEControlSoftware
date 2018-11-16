
class LimitSwitchHitError(Exception):
    pass

class FlyManipulatorCrashError(LimitSwitchHitError):
    pass

class SurfaceNotFoundError(Exception):
    pass

class NeedToHomeError(Exception):
    pass
