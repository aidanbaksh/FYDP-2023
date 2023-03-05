#!/usr/bin/env python3

from enum import Enum

import rospy
from std_msgs.msg import Float32, Bool
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
from audio_feedback.msg import AudioWarning

from custom_enums import HazardType, Direction, Severity



class SoundManager:
    def __init__(self):
        rospy.init_node('aggregator')

        # subscribe to the audio_warnings topic
        self.left_US_subscriber = rospy.Subscriber(
            '/left_US', AudioWarning, self._audio_warning_callback
        )

        self.right_US_subscriber = rospy.Subscriber(
            '/left_US', AudioWarning, self._audio_warning_callback
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
    

if __name__ == '__main__':
    manager = SoundManager()
    rospy.spin()





