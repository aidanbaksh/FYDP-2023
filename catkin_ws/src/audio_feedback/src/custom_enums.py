
class HazardType(Enum):
    CURB = 0
    OBJECT = 1
    TIP = 2
    @classmethod
    def has_value(cls, value):
        return value in cls._value2member_map_

class Direction(Enum):
    FRONT = 0
    FRONT_RIGHT = 1
    RIGHT = 2
    BACK_RIGHT = 3
    BACK = 4
    BACK_LEFT = 5
    LEFT = 6
    FRONT_LEFT = 7
    @classmethod
    def has_value(cls, value):
        return value in cls._value2member_map_

class Severity(Enum):
    LOW = 0
    MEDIUM = 1
    HIGH = 2
    @classmethod
    def has_value(cls, value):
        return value in cls._value2member_map_
