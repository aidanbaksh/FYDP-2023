#!/usr/bin/env python3
# read saved transforms and publish them

import rospy
import tf2_ros

from lidar_merge import serialize


def main(args=None):
    rospy.init_node('publish_lidar_transforms')

    # read calibrated transforms to file
    transforms = serialize.read()

    # setup transform broadcaster
    tf_publisher = tf2_ros.StaticTransformBroadcaster()    

    # publish stored transforms repeatedly for 1 minnute
    for i in range(20):
        tf_publisher.sendTransform(transforms)
        rospy.sleep(3)


if __name__ == '__main__':
    main()
