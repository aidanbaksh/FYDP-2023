#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool

diff_threshold = 4.0
raw_data_arr = []
rolling_avg_sum = 0
rolling_avg_size = 5.0

def reading_cb(data):
    global rolling_avg_sum, rolling_avg_size, diff_threshold, raw_data_arr
    distance = data.data
    # distance = data
    if(len(raw_data_arr) < 5):
        raw_data_arr.append(distance)
        rolling_avg_sum = rolling_avg_sum + distance
        return
    cur_avg = rolling_avg_sum / rolling_avg_size
    if(abs(cur_avg - distance) > diff_threshold):
        # print("CURB")
        pub.publish(True)
    else:
        # print("NO CURB")
        pub.publish(False)
    pop_val = raw_data_arr.pop(0)
    rolling_avg_sum = rolling_avg_sum - pop_val
    raw_data_arr.append(distance)
    rolling_avg_sum = rolling_avg_sum + distance
    

def listener():
    rospy.init_node('ultrasonic_curb_detector', anonymous = True)
    input_path = rospy.get_param('~input_path', "ultrasonic")
    output_path = rospy.get_param('~output_path', "test")
    print(input_path)
    print(output_path)
    sub = rospy.Subscriber(input_path, Float32, reading_cb)
    pub = rospy.Publisher(output_path, Bool, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    # distance_arr = [10.0, 10.0, 9.0, 11.0, 10.0, 10.0, 10.0, 15.0]
    # for i in distance_arr:
    #     reading_cb(i)
    listener()