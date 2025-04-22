import SensorErrorType

class SensorError(Exception):
    def __init__(self, error_type: SensorErrorType, message: str):
        self.error_type = error_type
        self.message = message
        super().__init__(message)