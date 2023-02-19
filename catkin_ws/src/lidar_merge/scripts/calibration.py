#!/usr/bin/env python3

import numpy as np

import rospy
from rospy.node import Node

from geometry_msgs.msg import TransformStamped


# the base frame is the IMU frame which is assumed to be at the COG of the wheelchair
# y axis of wheelchair is forward backwards
# z axis of wheelchair is up
# x axis of wheelchair is parallel to the axel of the wheels

# enforce constraints that front lidars are:
#   1. symmetric about y axis (|x| is equal)
#   2. have the same y offset
#   3. have the same z offset

# the front lidar mount is free to move in pitch and yaw
# from the top view: CCW yaw is +ve
# from the side view: upward pitch is +ve

# enforce constraints the back lidar is:
#   1. along the y axis
#   2. only has a pitch angle (no yaw)


class Calibration(Node):
    N_LIDARS: int = 3

    # lidar initial guess are (pitch, yaw)
    FRONT_LEFT_MOUNT_GUESS: tuple[float] = (-20, 10)
    FRONT_RIGHT_MOUNT_GUESS: tuple[float] = (-20, -10)
    FRONT_POST_ANGLE_GUESS: float = 30

    FRONT_MOUNT_X_OFFSET: float = 0.2  # 20cm
    FRONT_MOUNT_Y_OFFSET: float = 0.6  # approx 2ft
    FRONT_MOUNT_Z_OFFSET: float = 0.1  # 10cm 

    # back lidar mount can only pitch
    BACK_MOUNT_PITCH_GUESS: float = -30

    BACK_MOUNT_Y_OFFSET: float = -0.039  # 39mm
    BACK_MOUNT_Z_OFFSET: float = -0.025  # 2.5cm

    def __init__(self):
        super().__init__('lidar_merge_calibration')
        
        self._tf_publisher = self.create_publisher(TransformStamped, '/tf_static', N_LIDARS)

    """Performs the calibration"""
    def run(self) -> None:
        front_left_tf = self._generate_front_lidar_tf(
            *FRONT_LEFT_MOUNT_GUESS, FRONT_POST_ANGLE_GUESS,
            np.array([-FRONT_MOUNT_X_OFFSET, FRONT_MOUNT_Y_OFFSET, FRONT_MOUNT_Z_OFFSET])
        )
        front_right_tf = self._generate_front_lidar_tf(
            *FRONT_RIGHT_MOUNT_GUESS, FRONT_POST_ANGLE_GUESS,
            np.array([FRONT_MOUNT_X_OFFSET, FRONT_MOUNT_Y_OFFSET, FRONT_MOUNT_Z_OFFSET])
        )
        back_tf = self._generate_back_lidar_tf(
            BACK_MOUNT_PITCH_GUESS,
            np.array([0, BACK_MOUNT_Y_OFFSET, BACK_MOUNT_Z_OFFSET])
        )

        pass

    
    def _generate_front_lidar_tf(self, pitch: float, yaw: float, post_angle: float, offset: np.ndarray) -> np.ndarray:
        qx, qy, qz = offset
        return np.array([
            [],
            [],
            [],
            [0, 0, 0, 1],
        ])

    def _generate_back_lidar_tf(self, pitch: float, offset: np.ndarray) -> np.ndarray:
        qx, qy, qz = offset
        assert(qx == 0)
        return np.array([
            [],
            [],
            [],
            [0, 0, 0, 1],
        ])


# TODO: make an action server
def main(args=None):
    rospy.init(args=args)

    calibration = Calibration()

    calibration.run()

    # destroy the node explicitly
    calibration.destroy_node()
    rospy.shutdown()


if __name__ == '__main__':
    main()
