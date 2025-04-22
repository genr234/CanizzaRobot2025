from enum import Enum

class SensorErrorType(Enum):
    TIMEOUT = 1
    COMMUNICATION = 2
    INVALID_DATA = 3
