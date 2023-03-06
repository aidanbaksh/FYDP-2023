#!/usr/bin/env python3

from enum import Enum

import rospy
from std_msgs.msg import Float32, Bool
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
from audio_feedback.msg import AudioWarning, LidarCurb, LidarObject, UltrasonicCurb, UltrasonicObject

from custom_enums import HazardType, Direction, Severity



class Aggregator:
    def __init__(self):
        rospy.init_node('aggregator')

        # the one and only publisher
        self._publisher = rospy.Publisher('/audio_warnings', AudioWarning, queue_size=10)

        # subscribe to the various detection topics
        self._l_US_subscriber = rospy.Subscriber(
            '/left_us_detect', UltrasonicObject, self._l_US_callback
        )

        self._r_US_subscriber = rospy.Subscriber(
            '/right_us_detect', UltrasonicObject, self._r_US_callback
        )

        self._fl_US_subscriber = rospy.Subscriber(
            '/front_left_us_detect', UltrasonicObject, self._fl_US_callback
        )

        self._fr_US_subscriber = rospy.Subscriber(
            '/front_right_us_detect', UltrasonicObject, self._fr_US_callback
        )

        self._fls_US_subscriber = rospy.Subscriber(
            '/front_left_side_us_detect', UltrasonicObject, self._fls_US_callback
        )

        self._frs_US_subscriber = rospy.Subscriber(
            '/front_right_side_us_detect', UltrasonicObject, self._frs_US_callback
        )

        self._b_US_subscriber = rospy.Subscriber(
            '/back_us_detect', UltrasonicObject, self._b_US_callback
        )

        self._lidar_curb_subscriber = rospy.Subscriber(
            '/lidar_curb_detect', LidarCurb, self._lidar_curb_callback
        )

        self._lidar_object_subscriber = rospy.Subscriber(
            '/lidar_object_detect', LidarObject, self._lidar_curb_callback
        )

        # self._warning_wait_time = rospy.Duration(10)
        # self._curb_warning_times = {dir: rospy.Time.now() for dir in Direction.__members__}
        # self._object_warning_times = {dir: rospy.Time.now() for dir in Direction.__members__}


        

    # Ultrasonic callbacks
    def _l_US_callback(self, msg: UltrasonicObject) -> None:
        self._process_US_object_detection(msg, Direction.LEFT)

    def _r_US_callback(self, msg: UltrasonicObject) -> None:
        self._process_US_object_detection(msg, Direction.RIGHT)

    def _fl_US_callback(self, msg: UltrasonicObject) -> None:
        self._process_US_object_detection(msg, Direction.FRONT_LEFT)

    def _fr_US_callback(self, msg: UltrasonicObject) -> None:
        self._process_US_object_detection(msg, Direction.FRONT_RIGHT)

    def _fls_US_callback(self, msg: UltrasonicObject) -> None:
        self._process_US_object_detection(msg, Direction.LEFT)

    def _frs_US_callback(self, msg: UltrasonicObject) -> None:
        self._process_US_object_detection(msg, Direction.RIGHT)

    def _b_US_callback(self, msg: UltrasonicObject) -> None:
        self._process_US_object_detection(msg, Direction.BACK)

    def _process_US_object_detection(self, msg: UltrasonicObject, dir: Direction) -> None:
        if msg.detection == True:
            self._publish_message(HazardType.OBJECT, dir, Severity.LOW)
        
        

    def _lidar_curb_callback(self, msg: LidarObject) -> None:
        if msg.front > 0:
            self._publish_message(HazardType.CURB, Direction.FRONT, Severity.LOW)
        if msg.front_right > 0:
            self._publish_message(HazardType.CURB, Direction.FRONT_RIGHT, Severity.LOW)
        if msg.front_left > 0:
            self._publish_message(HazardType.CURB, Direction.FRONT_LEFT, Severity.LOW)
        if msg.back > 0:
            self._publish_message(HazardType.CURB, Direction.BACK, Severity.LOW)

    def _lidar_object_callback(self, msg: LidarObject) -> None:
        if msg.front == True:
            self._publish_message(HazardType.OBJECT, Direction.FRONT, Severity.LOW)
        if msg.front_right == True:
            self._publish_message(HazardType.OBJECT, Direction.FRONT_RIGHT, Severity.LOW)
        if msg.front_left == True:
            self._publish_message(HazardType.OBJECT, Direction.FRONT_LEFT, Severity.LOW)
        if msg.back == True:
            self._publish_message(HazardType.OBJECT, Direction.BACK, Severity.LOW)

    def _publish_message(self, h_type: HazardType, dir: Direction, sev: Severity) -> None:
            new_msg = AudioWarning()
            new_msg.warning_type = h_type
            new_msg.direction = dir
            new_msg.severity = sev
            self._publisher.publish(new_msg)


if __name__ == '__main__':
    agg = Aggregator()
    rospy.spin()





