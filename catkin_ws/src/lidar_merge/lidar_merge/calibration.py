import collections
from typing import Tuple

import numpy as np
import scipy.optimize

import rospy

import tf_conversions
import tf2_ros

from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped

from lidar_merge import constants


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


# TODO: make an action server
class Calibration:
    # this value is set by the technician / installer
    FRONT_MOUNT_Y_OFFSET_GUESS: float = 0.6  # approx 2ft

    # lidar initial guess are (pitch, yaw)
    FRONT_LEFT_MOUNT_GUESS: Tuple[float] = (-20, 10)
    FRONT_RIGHT_MOUNT_GUESS: Tuple[float] = (-20, -10)
    FRONT_POST_ANGLE_GUESS: float = 30

    FRONT_MOUNT_X_OFFSET_GUESS: float = 0.2  # 20cm
    FRONT_MOUNT_Z_OFFSET_GUESS: float = 0.1  # 10cm 

    # back lidar mount can only pitch
    BACK_MOUNT_PITCH_GUESS: float = -30

    NUM_DISTANCE_READINGS: int = 100

    def __init__(self):
        rospy.init_node('lidar_merge_calibration')

        # initialize back distance subscriber
        self._back_distance_subscriber = rospy.Subscriber(
            '/ultrasonics/back_housing', Float32, self._ultrasonic_reading_callback
        )
        # keep track of readings from back ultrasonic
        self._back_distance_readings = collections.deque([], Calibration.NUM_DISTANCE_READINGS)

        # publisher of calibrated transforms
        self._tf_publisher = tf2_ros.StaticTransformBroadcaster()

    """Records an ultrasonic reading for a given sensor in self._housing_distance_readings"""
    def _ultrasonic_reading_callback(self, msg: Float32) -> None:
        self._back_distance_readings.appendLeft(float(msg))

    """Performs the calibration"""
    def run(self) -> None:
        print('waiting for enough measurements')
        # # wait until we have enough distance measurements from back ultrasonic before proceeding
        # while len(self._back_distance_readings) < self._back_distance_readings.maxlen:
        #     rospy.spin()

        # delete ultrasonic distance subscriptions as they are no longer needed
        self._back_distance_subscriber.unregister()
        self._back_distance_subscriber = None

        # # calculate average distance
        # back_ultrasonic_dist = np.mean(self._back_distance_readings)

        # get centroid of back lidar readings

        # can solve for back sensor pitch and distance to ground plane

        # calculate distance to ground plane of each lidar
        
        # extract planes to figure out z offset of front sensors

        # setup initial guess vector
        # NOTE: this should match how values are extracted in _tfs_from_solution
        initial_guess = np.array([
            *Calibration.FRONT_LEFT_MOUNT_GUESS,
            *Calibration.FRONT_RIGHT_MOUNT_GUESS,
            Calibration.FRONT_POST_ANGLE_GUESS,
            Calibration.FRONT_MOUNT_X_OFFSET_GUESS,
            Calibration.FRONT_MOUNT_Y_OFFSET_GUESS,
            Calibration.FRONT_MOUNT_Z_OFFSET_GUESS,
            Calibration.BACK_MOUNT_PITCH_GUESS,
        ])

        # optimize to find best guess for lidar positions
        result = scipy.optimize.minimize(self._compute_cost, initial_guess, options={'maxiter': 1})

        print('optimize result', result)

        # handle error if solver failed to converge
        if not result.success:
            rospy.logfatal("Calibration failed to converge to a solution!")
            rospy.info("Optimization message: %s", result.message)
            return  # just give up

        # extract solution and publish results
        front_left_tf, front_right_tf, back_tf = self._tfs_from_solution(result.x)
        self._publish_tf(constants.FRONT_LEFT_FRAME, front_left_tf)
        self._publish_tf(constants.FRONT_RIGHT_FRAME, front_right_tf)
        self._publish_tf(constants.BACK_FRAME, back_tf)

    def _tfs_from_solution(self, x: np.ndarray) -> Tuple[np.ndarray]:
        # extract values from current solution vector
        # NOTE: this should match the initial guesses provided to the solver
        front_left_pitch  = x[0]
        front_left_yaw    = x[1]
        front_right_pitch = x[2]
        front_right_yaw   = x[3]
        front_post_angle  = x[4]
        front_x_offset    = x[5]
        front_y_offset    = x[6]
        front_z_offset    = x[7]
        back_pitch        = x[8]

        # construct transformation matrices from current solution
        front_left_tf = self._front_lidar_tf_matrix(
            front_left_pitch, front_left_yaw, front_post_angle,
            np.array([-front_x_offset, front_y_offset, front_z_offset])
        )
        front_right_tf = self._front_lidar_tf_matrix(
            front_right_pitch, front_right_yaw, front_post_angle,
            np.array([front_x_offset, front_y_offset, front_z_offset])
        )
        back_tf = self._back_lidar_tf_matrix(
            back_pitch,
            np.array([0, constants.BACK_MOUNT_Y_OFFSET, constants.BACK_MOUNT_Z_OFFSET])
        )
        return (front_left_tf, front_right_tf, back_tf)

    def _compute_cost(self, x: np.ndarray) -> None:
        front_left_tf, front_right_tf, back_tf = self._tfs_from_solution(x)

        # get point cloud as (4, *) array then
        # left_points_global = np.dot(front_left_tf, left_points)

        # transform all lidar points into world frame

        # extract ground plane to determine z offset and pitch
        # https://github.com/leomariga/pyRANSAC-3D

        # compute cost

        # TODO: out of bounds estimtaes should have very high cost
        return 1

    """Convenience method to generate front lidar transformation matrix"""
    def _front_lidar_tf_matrix(self, pitch: float, yaw: float, post_angle: float, offset: np.ndarray) -> np.ndarray:
        global_pitch = post_angle + pitch
        return self._tf_matrix((global_pitch, yaw, 0), offset)

    """Convenience method to generate back lidar transformation matrix"""
    def _back_lidar_tf_matrix(self, pitch: float, offset: np.ndarray) -> np.ndarray:
        assert(offset[0] == 0)  # along x axis
        return self._tf_matrix((pitch, 0, 0), offset)

    """Generates a transformation matrix from Euler angles and an offset from the wheelchair frame"""
    def _tf_matrix(self, euler_angles: Tuple[float], offset: np.ndarray):
        # generate rotation matrix from euler angles
        # the order of euler angles is imporant
        # the first adjustment angle is the pitch of the sensor housing, then the yaw
        # as a result, x is the first angle, then z, then y
        # to accomplush this, use 'r'otating frame
        # https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py#L77-L91
        T = tf_conversions.transformations.euler_matrix(*np.radians(euler_angles), 'rxzy')
        # add translation offset
        T[0:3, 3] = offset
        return T

    """Convenience method which formats and publishes a transform"""
    def _publish_tf(self, lidar_frame: str, T: np.ndarray) -> TransformStamped:
        # create message with frame ids
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = constants.WHEELCHAIR_FRAME
        t.child_frame_id = lidar_frame
        # convert transformation matrix into translation and rotation
        t.transform.translation = tf_conversions.transformations.translation_from_matrix(T)
        t.transform.rotation = tf_conversions.transformations.quaternion_from_matrix(T)
        # publish
        self._tf_publisher.sendTransform(t)


# essentially the main function, called by calibrate script
def run():
    calibration = Calibration()
    calibration.run()

    rospy.spin()
