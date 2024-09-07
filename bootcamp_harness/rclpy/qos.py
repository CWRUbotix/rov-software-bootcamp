from enum import Enum


class QoSPresetProfiles(Enum):
    UNKNOWN = 0
    DEFAULT = 1
    SYSTEM_DEFAULT = 2
    SENSOR_DATA = 3
    SERVICES_DEFAULT = 4
    PARAMETERS = 5
    PARAMETER_EVENTS = 6
    ACTION_STATUS_DEFAULT = 7
    BEST_AVAILABLE = 8


QoSProfile = int
