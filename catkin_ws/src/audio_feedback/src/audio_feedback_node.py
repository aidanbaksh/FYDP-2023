#!/usr/bin/env python3

from enum import Enum

import rospy
from std_msgs.msg import Float32, Bool
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
from audio_feedback.msg import AudioWarning


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
            '/audio_warnings', AudioWarning, self._audio_warning_callback
        )

        self._sound_handle = SoundClient(blocking=True)
        rospy.sleep(0.5)

        self._last_played_dict = {}
        self._node_start_time = rospy.Time.now()

        self._sev_time_delays = {
            Severity.LOW: rospy.Duration(10),
            Severity.MEDIUM: rospy.Duration(5),
            Severity.HIGH: rospy.Duration(0)
            }
    
    def _audio_warning_callback(self, msg: AudioWarning) -> None:
        h_type = msg.warning_type
        dir = msg.direction
        sev = msg.severity
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
        cur_time = rospy.Time.now()
        if cur_time-last_time > delay :
            return True
        return False

    def getLastPlayedTime(self, path):
        if path in self._last_played_dict.keys():
            return self._last_played_dict[path]
        else:
            return self._node_start_time

    def setLastPlayedTime(self, path):
        self._last_played_dict[path] = rospy.Time.now()

    def setPathFromInputs(self, h_type: HazardType, dir: Direction) -> None:
        path = "audio_warnings/"

        if h_type == HazardType.CURB:
            path += "curb/curb"
        elif h_type == HazardType.OBJECT:
            path += "object/object"
        elif h_type == HazardType.TIP:
            path += "tip/tip"
        
        if dir == Direction.FRONT:
            path += "_front"
        elif dir == Direction.FRONT_RIGHT: 
            path += "_front_right"
        elif dir == Direction.RIGHT: 
            path += "_right"
        elif dir == Direction.BACK_RIGHT: 
            path += "_back_right"
        elif dir == Direction.BACK: 
            path += "_back"
        elif dir == Direction.BACK_LEFT: 
            path += "_back_left"
        elif dir == Direction.LEFT: 
            path += "_left"
        elif dir == Direction.FRONT_LEFT: 
            path += "_front_left"

        self._request_path = path
        
        

if __name__ == '__main__':
    manager = SoundManager()
    rospy.spin()





