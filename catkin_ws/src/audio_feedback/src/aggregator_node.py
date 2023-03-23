#!/usr/bin/env python3

from enum import Enum

import rospy
from std_msgs.msg import Float32, Bool
from audio_feedback.msg import AudioWarning, LidarCurb, LidarObject, UltrasonicCurb, UltrasonicObject, TipWarning

from custom_enums import HazardType, Direction, Severity

US_TOPIC_STRING = "ultrasonics/{}/{}_detect"
LIDAR_TOPIC_STRING = "lidar_{}_detect"




class Aggregator:
    def __init__(self):
        rospy.init_node('aggregator')

        # the one and only publisher
        self._publisher = rospy.Publisher('/audio_warnings', AudioWarning, queue_size=10)

        # subscribe to the various detection topics
        self._l_US_subscriber = rospy.Subscriber(
            US_TOPIC_STRING.format('left', 'object'), UltrasonicObject, self._l_US_callback
        )

        self._r_US_subscriber = rospy.Subscriber(
            US_TOPIC_STRING.format('right', 'object'), UltrasonicObject, self._r_US_callback
        )

        self._fl_US_subscriber = rospy.Subscriber(
            US_TOPIC_STRING.format('front_left', 'object'), UltrasonicObject, self._fl_US_callback
        )

        self._fr_US_subscriber = rospy.Subscriber(
            US_TOPIC_STRING.format('front_right', 'object'), UltrasonicObject, self._fr_US_callback
        )

        self._fls_US_subscriber = rospy.Subscriber(
            US_TOPIC_STRING.format('front_left_side', 'object'), UltrasonicObject, self._fls_US_callback
        )

        self._frs_US_subscriber = rospy.Subscriber(
            US_TOPIC_STRING.format('front_right_side', 'object'), UltrasonicObject, self._frs_US_callback
        )

        self._b_US_subscriber = rospy.Subscriber(
            US_TOPIC_STRING.format('back', 'object'), UltrasonicObject, self._b_US_callback
        )

        self._lidar_curb_subscriber = rospy.Subscriber(
            '/lidar/curb_detect', LidarCurb, self._lidar_curb_callback
        )

        self._lidar_object_subscriber = rospy.Subscriber(
            '/lidar/object_detect', LidarObject, self._lidar_object_callback
        )

        self._tip_warning_subscriber = rospy.Subscriber(
            '/imu/tip_warning', TipWarning, self._tip_warning_callback
        )

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
        if msg.front:
            self._publish_message(HazardType.OBJECT, Direction.FRONT, Severity.LOW)
        if msg.front_right:
            self._publish_message(HazardType.OBJECT, Direction.FRONT_RIGHT, Severity.LOW)
        if msg.front_left:
            self._publish_message(HazardType.OBJECT, Direction.FRONT_LEFT, Severity.LOW)
        if msg.back:
            self._publish_message(HazardType.OBJECT, Direction.BACK, Severity.LOW)

    def _tip_warning_callback(self, msg: TipWarning) -> None:
        self._publish_message(HazardType.TIP, Direction(msg.direction), Severity(msg.severity))

    def _publish_message(self, h_type: HazardType, dir: Direction, sev: Severity) -> None:
            new_msg = AudioWarning()
            new_msg.warning_type = h_type
            new_msg.direction = dir
            new_msg.severity = sev
            self._publisher.publish(new_msg)


if __name__ == '__main__':
    agg = Aggregator()
    rospy.spin()





