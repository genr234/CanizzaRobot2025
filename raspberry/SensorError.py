from SensorErrorType import SensorErrorType #Più che altro questo


class SensorError(Exception):
    def __init__(self, error_type: SensorErrorType, message: str):
        self.error_type = error_type
        self.message = message
        super().__init__(message)

    def __str__(self):
        return f"[{self.error_type.name}] {self.message}"