import collections
from typing import Tuple

import numpy as np
import scipy.optimize

import rospy

import tf_conversions
import tf2_ros

from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import pyransac3d as pyrsc

from lidar_merge import constants


# the base frame is the IMU frame which is assumed to be at the COG of the wheelchair
# x axis of wheelchair is forward backwards
# y axis of wheelchair is parallel to the axel of the wheels
# z axis of wheelchair is up

# enforce constraints that front lidars are:
#   1. symmetric about x axis (|y| is equal)
#   2. have the same x offset
#   3. have the same z offset

# the front lidar mount is free to move in pitch and yaw
# from the top view: CCW yaw is +ve
# from the side view: downward pitch is +ve

# enforce constraints the back lidar is:
#   1. along the x axis
#   2. only has a pitch angle (no yaw)


# TODO: make an action server
class Calibration:
    # this value is set by the technician / installer
    FRONT_MOUNT_X_OFFSET_GUESS: float = 0.6  # approx 2ft

    # lidar initial guess are (pitch, yaw)
    FRONT_LEFT_MOUNT_GUESS: Tuple[float] = (40, 10)
    FRONT_RIGHT_MOUNT_GUESS: Tuple[float] = (40, -10)
    FRONT_POST_ANGLE_GUESS: float = -30

    FRONT_MOUNT_Y_OFFSET_GUESS: float = 0.2  # 20cm
    FRONT_MOUNT_Z_OFFSET_GUESS: float = 0.05  # 5cm 

    # back lidar mount can only pitch
    BACK_MOUNT_PITCH_GUESS: float = 20
    BACK_MOUNT_HEIGHT_GUESS: float = 25

    MAX_NUM_BACK_ULTRASONIC_READINGS: int = 50
    MIN_NUM_BACK_LIDAR_READINGS: int = 30

    def __init__(self):
        rospy.init_node('lidar_merge_calibration')

        # initialize back distance subscriber
        # TODO: change ultrasonic topic
        self._back_ultrasonic_subscriber = rospy.Subscriber(
            '/ultrasonics/left', Float32, self._ultrasonic_reading_callback
        )
        # keep track of readings from back ultrasonic
        self._back_ultrasonic_distances = collections.deque([], Calibration.MAX_NUM_BACK_ULTRASONIC_READINGS)

        # initialize back lidar subscriber
        self._back_lidar2d_subscriber = rospy.Subscriber(
            '/scan_2D_1', PointCloud2, self._back_lidar2d_callback
        )
         # keep track of distances from back lidar
        self._back_lidar_distances = collections.deque([], Calibration.MIN_NUM_BACK_LIDAR_READINGS)

        # will be initialized later when needed
        self._back_lidar3d_subscriber = None
        self._back_lidar_points = None

        # publisher of calibrated transforms
        self._tf_publisher = tf2_ros.StaticTransformBroadcaster()

    """Records an ultrasonic reading for a given sensor in self._housing_distance_readings"""
    def _ultrasonic_reading_callback(self, msg: Float32) -> None:
        self._back_ultrasonic_distances.appendleft(msg.data)

    def _position_to_distance(self, point: Tuple[float, float, float]) -> float:
        return np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)

    """Records an ultrasonic reading for a given sensor in self._back_lidar_distances"""
    def _back_lidar2d_callback(self, msg: PointCloud2) -> None:
        # read pointclud msg        
        points = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z'))
        # convert points to distances
        distances = np.fromiter([self._position_to_distance(pt) for pt in points], np.float64)
        # remove outliers from pointcloud
        q1, q3 = np.percentile(distances, [25, 75])
        iqr = q3 - q1
        to_keep = (q1 - 1.5*iqr <= distances) & (distances <= q3 + 1.5*iqr)
        distances = np.sort(distances[to_keep])

        # center distance will be min distance
        # print(len(distances))
        center_dist = np.mean(distances[0:min(10, len(distances))])
        self._back_lidar_distances.appendleft(center_dist)

    """Records an ultrasonic reading for a given sensor in self._back_lidar_distances"""
    def _back_lidar3d_callback(self, msg: PointCloud2) -> None:
        # read pointclud msg        
        self._back_lidar_points = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z'))

    """Corrects 2D lidar distance measurement using fit model"""
    def _lidar_distance_offset(self, lidar_distance: float) -> float:
        return 0.1629 * lidar_distance + 1.8676

    """Performs the calibration"""
    def run(self) -> None:
        rospy.loginfo("Starting calibration...")

        wait_for_back_data = rospy.Rate(10)  # 10 Hz

        # wait until we have enough lidar readings before proceeding
        # no explicit requirement on ultrasonics, just a max number since they have a faster sample rate
        while len(self._back_lidar_distances) < self._back_lidar_distances.maxlen:
            rospy.loginfo("Got %d lidar measurements.", len(self._back_lidar_distances))
            wait_for_back_data.sleep()

        # delete back ultrasonic and lidar subscriptions as they are no longer needed
        self._back_ultrasonic_subscriber.unregister()
        self._back_ultrasonic_subscriber = None
        self._back_lidar2d_subscriber.unregister()
        self._back_lidar2d_subscriber = None

        # calculate average distance
        back_ultrasonic_dist = float(np.mean(self._back_ultrasonic_distances))
        back_lidar_dist = float(np.mean(self._back_lidar_distances)) * 100  # convert from m to cm

        # add calculated correction factor to lidar distance
        back_lidar_dist += self._lidar_distance_offset(back_lidar_dist)

        print('All ultrasonics', self._back_ultrasonic_distances)

        # print measured distances
        rospy.loginfo("Back Ultrasonic Distance (cm): %f", back_ultrasonic_dist)
        rospy.loginfo("Back Lidar Distance (cm): %f", back_lidar_dist)

        # add offsets to intersection point
        back_ultrasonic_dist += constants.LIDAR_MOUNT_ULTRASONIC_TO_INTERSECTION
        back_lidar_dist += constants.LIDAR_MOUNT_LIDAR_TO_INTERSECTION

        # compute angle and distance to ground
        def back_angle_and_height(x: Tuple[float, float]) -> np.ndarray:
            alpha, h = x
            return (
                h/back_ultrasonic_dist - np.cos(np.deg2rad(alpha)),
                h/back_lidar_dist - np.cos(np.deg2rad(constants.BACK_MOUNT_LIDAR_ULTRASONIC_ANGLE))
            )
        alpha, back_mount_height = scipy.optimize.fsolve(
            back_angle_and_height,
            (Calibration.BACK_MOUNT_PITCH_GUESS, Calibration.BACK_MOUNT_HEIGHT_GUESS)
        )
        back_mount_pitch = alpha + constants.BACK_MOUNT_LIDAR_ULTRASONIC_ANGLE

        rospy.loginfo('Back Pitch Angle (deg): %f', back_mount_pitch)
        rospy.loginfo('Back Height (cm): %f', back_mount_height)

        # subscribe to 3d pointcloud from back lidar
        self._back_lidar3d_subscriber = rospy.Subscriber(
            '/scan_3D_1', PointCloud2, self._back_lidar2d_callback
        )
        # wait for reading from back lidar
        while self._back_lidar_points is None:
            wait_for_back_data.sleep()
        # delete lidar 3d subscriptions as it is no longer needed
        self._back_lidar3d_subscriber.unregister()
        self._back_lidar3d_subscriber = None

        # extract ground plane from back lidar
        back_plane_eqn, _ = pyrsc.Plane().fit(self._back_lidar_points, 0.01)

        print('Back plane:', back_plane_eqn)
        rospy.loginfo('Ground Plane (from back lidar): [%f, %f, %f, %f]', *back_plane_eqn)

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

        # handle error if solver failed to converge
        if not result.success:
            rospy.logfatal("Calibration failed to converge to a solution!")
            rospy.loginfo("Optimization message: %s", result.message)
            return  # just give up

        # extract transform matrices from solution
        front_left_tf, front_right_tf, back_tf = self._tfs_from_solution(result.x)
        # format transforms as geometry_msgs.TransformStamped
        formatted_tfs = []
        formatted_tfs.append(self._format_tf(constants.FRONT_LEFT_FRAME, front_left_tf))
        formatted_tfs.append(self._format_tf(constants.FRONT_RIGHT_FRAME, front_right_tf))
        formatted_tfs.append(self._format_tf(constants.BACK_FRAME, back_tf))
        # publish transforms
        # all have to be published at once due to a bug with StaticTransformBroadcaster
        # https://answers.ros.org/question/287469/unable-to-publish-multiple-static-transformations-using-tf/
        self._tf_publisher.sendTransform(formatted_tfs)
        rospy.loginfo("Published calibrated transforms")

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
            np.array([front_x_offset, front_y_offset, front_z_offset])
        )
        front_right_tf = self._front_lidar_tf_matrix(
            front_right_pitch, front_right_yaw, front_post_angle,
            np.array([front_x_offset, -front_y_offset, front_z_offset])
        )
        back_tf = self._back_lidar_tf_matrix(
            back_pitch,
            np.array([constants.BACK_MOUNT_X_OFFSET, 0, constants.BACK_MOUNT_Z_OFFSET])
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
        euler_angles = (global_pitch, yaw, 0)
        # generate rotation matrix from euler angles
        # the order of euler angles is imporant
        # the first adjustment angle is the pitch of the sensor housing, then the yaw
        # as a result, y is the first angle, then z, then x
        # to accomplush this, use 'r'otating frame
        # https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py#L77-L91
        T = tf_conversions.transformations.euler_matrix(*np.radians(euler_angles), 'ryzx')
        # add translation offset
        T[0:3, 3] = offset
        return T

    """Convenience method to generate back lidar transformation matrix"""
    def _back_lidar_tf_matrix(self, pitch: float, offset: np.ndarray) -> np.ndarray:
        assert(offset[1] == 0)  # on xz plane

        euler_angles = (pitch, 180, 0)
        # generate rotation matrix from euler angles
        # the only adjustment angle is the pitch of the sensor housing,
        # however, the back sensor housing points backwards, as a result rotate 180 around z first
        # to accomplush this, use 's'tatic frame
        # https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py#L77-L91
        T = tf_conversions.transformations.euler_matrix(*np.radians(euler_angles), 'syzx')
        # add translation offset
        T[0:3, 3] = offset
        return T

    """Convenience method which formats transformation matrix as a TransformStamped"""
    def _format_tf(self, lidar_frame: str, T: np.ndarray) -> TransformStamped:
        # create message with frame ids
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = constants.WHEELCHAIR_FRAME
        t.child_frame_id = lidar_frame

        # convert transformation matrix into translation and rotation
        x, y, z = tf_conversions.transformations.translation_from_matrix(T)
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        qx, qy, qz, qw = tf_conversions.transformations.quaternion_from_matrix(T)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        return t


# essentially the main function, called by calibrate script
def run():
    calibration = Calibration()
    calibration.run()

    rospy.spin()
