#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool


class UltrasonicObstacleDetector:
    def __init__(self, input_topic: str, threshold: float) -> None:
        self._input_topic = input_topic

        self._detected_count = 0
        self._not_detected_count = 0
        self._detection_threshold = threshold
        self._detected_flag = False

    def start(self) -> None:
        # infer topic to use for publishing detected obstacles
        output_topic = '{ultrasonic}/object_detect'.format(ultrasonic=self._input_topic)
        self._detection_publisher = rospy.Publisher(output_topic, Bool, queue_size=10)

        # set up subscriber for ultrasonic readings
        rospy.Subscriber(self._input_topic, Float32, self._ultrasonic_reading_callback)

        rospy.loginfo('Starting ultrasonic object detection for %s', self._input_topic)

    def _ultrasonic_reading_callback(self, msg: Float32) -> None:
        distance = msg.data
        if distance <= self._detection_threshold:
            self._detected_count += 1
            if self._detected_count > 5:
                self._not_detected_count = 0
                self._detected_flag = True
        else:
            self._not_detected_count += 1
            if self._not_detected_count > 5:
                self._detected_count = 0
                self._detected_flag = False

        self._detection_publisher.publish(self._detected_flag)
    

def main() -> None:
    # setup node
    rospy.init_node('ultrasonic_obj_detector')

    # print('params', rospy.get_param_names())


    # get parameters
    ultrasonic_topic = rospy.get_param('~ultrasonic_topic')
    threshold = rospy.get_param('~threshold')

    # run detector
    detector = UltrasonicObstacleDetector(ultrasonic_topic, threshold)
    detector.start()
    rospy.spin()


if __name__ == '__main__':
    main()
