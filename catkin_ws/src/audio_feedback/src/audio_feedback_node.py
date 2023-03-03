#!/usr/bin/env python3

import time
from enum import Enum

import rospy
from std_msgs.msg import Float32, Bool
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest


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


class SoundManager:
    def __init__(self):
        rospy.init_node('audio_feedback')

        # subscribe to the audio_warnings topic
        self._audio_warning_subscriber = rospy.Subscriber(
            '/audio_warnings', Float32, self._audio_warning_callback
        )

        self._sound_handle = SoundClient(blocking=True)
        rospy.sleep(0.5)

        self._last_played_dict = {}
        self._sev_time_delays = {Severity.LOW: 10, Severity.MEDIUM: 5, Severity.HIGH: 0}
    
    def _audio_warning_callback(self, msg: Float32) -> None:
        h_type = 0
        dir = 0
        sev = 0
        if HazardType.has_value(h_type):
            h_type = HazardType(h_type)
        else:
            print("Invalid Hazard Type: {}".format(h_type)) # change these to rospy warns
            return False
        
        if Direction.has_value(dir):
            dir = Direction(dir)
        else:
            print("Invalid Direction: {}".format(dir))
            return False

        if Severity.has_value(sev):
            sev = Severity(sev)
        else:
            print("Invalid Severity: {}".format(sev))
            return False
        
        self.setPathFromInputs(h_type, dir)
        if self.checkTimeDelay(sev):
            print("Playing: {}".format(self._request_path)) #Change to warning/log message
            self._sound_handle.say("Hi! I am working.")
            self.setLastPlayedTime(self._request_path)
        else:
            # Change to warning/log message
            print("Received message ({},{},{}), but was last played too recently, so not reapeating.".format(h_type, dir, sev))
        
        return True

    def checkTimeDelay(self, sev: Severity) -> bool:
        delay = self._sev_time_delays[sev]
        last_time = self.getLastPlayedTime(self._request_path)
        cur_time = time.time()
        if cur_time-last_time > delay :
            return True
        return False

    def getLastPlayedTime(self, path):
        if path in self._last_played_dict.keys():
            return self._last_played_dict[path]
        else:
            return 0

    def setLastPlayedTime(self, path):
        cur_time = time.time() ## Modify to get the time from ROS
        self._last_played_dict[path] = cur_time

    def setPathFromInputs(self, h_type: HazardType, dir: Direction) -> bool:
        path = "audio_warnings/"

        match h_type:
            case HazardType.CURB: path += "curb/curb"
            case HazardType.OBJECT: path += "object/object"
            case HazardType.TIP: path += "tip/tip"
            case _: return False
        
        match dir:
            case Direction.FRONT: path += "_front"
            case Direction.FRONT_RIGHT: path += "_front_right"
            case Direction.RIGHT: path += "_right"
            case Direction.BACK_RIGHT: path += "_back_right"
            case Direction.BACK: path += "_back"
            case Direction.BACK_LEFT: path += "_back_left"
            case Direction.LEFT: path += "_left"
            case Direction.FRONT_LEFT: path += "_front_left"
            case _: return False

        self._request_path = path
        return True
        
        

if __name__ == '__main__':
    manager = SoundManager()





