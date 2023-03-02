#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool


detected_count = 0
not_detected_count = 0
detection_thresh = 0
detected_flag = False
def reading_cb(data):
    distance = data.data
    if distance < detection_thresh:
        detected_count = detected_count+1
        if detected_count > 5:
            not_detected_count = 0
            detected_flag = True
    else:
        not_detected_count=not_detected_count+1
        if not_detected_count > 5:
            detected_count = 0
            detected_flag = False
    pub.publish(detected_flag)


def listener():
    rospy.init_node('ultrasonic_obj_detector', anonymous = True)
    input_path = rospy.get_param('~input_path', "ultrasonic")
    output_path = rospy.get_param('~output_path', "test")
    print(input_path)
    print(output_path)
    sub = rospy.Subscriber(input_path, Float32, reading_cb)
    pub = rospy.Publisher(output_path, Bool, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()