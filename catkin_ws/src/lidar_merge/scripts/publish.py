#!/usr/bin/env python3
# read saved transforms and publish them

import rospy
import tf2_ros

from lidar_merge import serialize


def main(args=None):
    rospy.init_node('publish_lidar_transforms')

    # read calibrated transforms to file
    transforms = serialize.read()

    # publish stored transforms
    tf_publisher = tf2_ros.StaticTransformBroadcaster()
    tf_publisher.sendTransform(transforms)
    
    rospy.sleep(5)  # wait for transforms to get published


if __name__ == '__main__':
    main()
