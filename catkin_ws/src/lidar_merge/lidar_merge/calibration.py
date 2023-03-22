import collections
import itertools
from typing import Callable, Optional, Tuple, List 

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
from lidar_merge.constants import LiDAR


# the base frame is the COG of the wheelchair (name of frame is wheelchair)
# the COG of the wheelchair is assumed to be at the IMU
# x axis of wheelchair is forward backwards
# y axis of wheelchair is parallel to the axel of the wheels
# z axis of wheelchair is up

# enforce constraints that front lidars are:
#   1. symmetric about x axis (|y| is equal)
#   2. have the same x offset
#   3. have the same z offset

# enforce constraints the back lidar is:
#   1. along the x axis
#   2. only has a pitch angle (no yaw)


class Calibration:
    # back lidar mount can only pitch
    BACK_MOUNT_PITCH_GUESS: float = 20
    BACK_MOUNT_HEIGHT_GUESS: float = 25

    MAX_NUM_BACK_ULTRASONIC_READINGS: int = 50
    MIN_NUM_BACK_LIDAR_READINGS: int = 5

    def __init__(self):
        rospy.init_node('calibrate_lidar_transforms')

        # initialize back distance subscriber
        # TODO: change ultrasonic topic
        self._back_ultrasonic_subscriber = rospy.Subscriber(
            constants.BACK_ULTRASONIC_TOPIC, Float32, self._ultrasonic_reading_callback
        )
        # keep track of readings from back ultrasonic
        self._back_ultrasonic_distances = collections.deque([], Calibration.MAX_NUM_BACK_ULTRASONIC_READINGS)

        # initialize back lidar subscriber
        self._back_lidar2d_subscriber = rospy.Subscriber(
            constants.BACK_LIDAR_2D_TOPIC, PointCloud2, self._back_lidar2d_callback
        )
         # keep track of distances from back lidar
        self._back_lidar_distances = collections.deque([], Calibration.MIN_NUM_BACK_LIDAR_READINGS)

        # publisher of calibrated transforms
        self._tf_publisher = tf2_ros.StaticTransformBroadcaster()

        self.wait_for_lidar_data = rospy.Rate(10)  # 10 Hz

        self.filtered_pts = {} # TODO: REMOVE ME!

    """Records an ultrasonic reading for a given sensor in self._housing_distance_readings"""
    def _ultrasonic_reading_callback(self, msg: Float32) -> None:
        self._back_ultrasonic_distances.appendleft(msg.data)

    """Corrects 2D lidar distance measurement using fit model"""
    def _lidar_distance_offset(self, lidar_distance: float) -> float:
        return 0.0802 * lidar_distance + 4.2054 + constants.BACK_MOUNT_LIDAR_TO_FRONT_CM

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

    """Takes a reading from a lidar sensor and finds a ground plane"""
    def _get_ground_plane(self, lidar: LiDAR) -> np.ndarray:

        def filter_lidar(points: np.ndarray) -> np.ndarray:
            # only keep points which have x coordinate in bounds
            return points[
                (constants.GROUND_PLANE_FILTER_X_MIN_THRESHOLD <= points[:,0]) &
                (points[:,0] <= constants.GROUND_PLANE_FILTER_X_MAX_THRESHOLD)
            ]

        ground_plane_eqn = self._get_lidar_plane(lidar, filter_lidar)

        # flip the planes so the normal points up
        if ground_plane_eqn[3] < 0:
            ground_plane_eqn = -ground_plane_eqn

        rospy.loginfo('Ground Plane (%s): [%f, %f, %f, %f]', lidar.value, *ground_plane_eqn)
        return ground_plane_eqn

        """Takes a reading from a lidar sensor and finds a wall plane"""
    def _get_wall_plane(self, lidar: LiDAR, R_lidar: np.ndarray, lidar_to_ground: float) -> np.ndarray:

        def filter_lidar(points: np.ndarray) -> np.ndarray:
            # only keep points which have x coordinate in bounds
            points = points[
                constants.GROUND_PLANE_FILTER_X_MIN_THRESHOLD <= points[:,0]
            ]

            points = points@np.linalg.inv(R_lidar)
            # only keep points which have z coordinate a certain threshold above the ground plane
            return points[
                points[:,2] >= (constants.WALL_PLANE_FILTER_MIN_Z_THRESHOLD - lidar_to_ground)
            ]

        wall_plane_eqn = self._get_lidar_plane(lidar, filter_lidar)

        # flip the planes so it points in the positive x direction
        if wall_plane_eqn[0] < 0:
            wall_plane_eqn = -wall_plane_eqn

        rospy.loginfo('Wall Plane (%s): [%f, %f, %f, %f]', lidar.value, *wall_plane_eqn)
        return wall_plane_eqn

    """Takes a reading from a lidar sensor and finds a plane"""
    def _get_lidar_plane(self, lidar: LiDAR,
        filter_func: Optional[Callable[[np.ndarray],np.ndarray]]) -> np.ndarray:

        lidar3d_points: np.ndarray = None

        """Records an ultrasonic reading for a given sensor in self._back_lidar_distances"""
        def lidar3d_callback(msg: PointCloud2) -> None:
            nonlocal lidar3d_points # sets lidar3d_points with result

            # read pointclud msg        
            points_gen = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z'))

            # clone the generator to get its length
            points_gen, it = itertools.tee(points_gen)
            n_points = sum(1 for _ in it)

            # read points as 1d array then reshape to Nx3
            flattened_pts = itertools.chain.from_iterable(points_gen)
            pts_1d = np.fromiter(flattened_pts, np.float64)
            lidar3d_points = pts_1d.reshape(n_points, 3)

        # subscribe to 3d pointcloud topic for all lidars
        lidar3d_subscriber = rospy.Subscriber(
            constants.LIDAR_3D_TOPCS[lidar], PointCloud2, lidar3d_callback
        )

        # wait for reading from lidar
        while lidar3d_points is None:
            self.wait_for_lidar_data.sleep()

        # delete lidar 3d subscriptions as it is no longer needed
        lidar3d_subscriber.unregister()
        lidar3d_subscriber = None

        # filter points
        lidar3d_points = filter_func(lidar3d_points)

        self.filtered_pts[lidar] = lidar3d_points  # TODO: REMOVE ME!

        plane_eqn, _ = pyrsc.Plane().fit(lidar3d_points, constants.PLANE_RANSAC_THRESHOLD)
        return np.array(plane_eqn)  # convert to np.ndarray
        

    """Gets normal vector from plane equation of the form Ax + Bx + Cx + D = 0"""
    def _get_plane_normal_vector(self, plane_eqn: np.ndarray) -> np.ndarray:
        return plane_eqn[0:3] # A, B, and C are elements of normal vector

    """Gets the rotation matrix which rotates a to be collinear with b"""
    def _rotation_matrix_for_plane(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        # https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d

        # normalize vectors
        a = a/np.linalg.norm(a)
        b = b/np.linalg.norm(b)

        v = np.cross(a, b)
        c = np.dot(a, b)
        v_x = np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])
        R = np.identity(3) + v_x + (1/(1+c))*v_x@v_x
        return R

    """Performs the calibration"""
    def run(self) -> List[TransformStamped]:
        rospy.loginfo("Starting calibration...")

        # wait until we have enough lidar readings before proceeding
        # no explicit requirement on ultrasonics, just a max number since they have a faster sample rate
        while len(self._back_lidar_distances) < self._back_lidar_distances.maxlen:
            rospy.loginfo("Got %d lidar measurements.", len(self._back_lidar_distances))
            self.wait_for_lidar_data.sleep()

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

        # print measured distances
        rospy.loginfo("Back Ultrasonic Distance (cm): %f", back_ultrasonic_dist)
        rospy.loginfo("Back Lidar Distance (cm): %f", back_lidar_dist)

        # add offsets to intersection point
        back_ultrasonic_dist += constants.BACK_MOUNT_ULTRASONIC_TO_INTERSECTION_CM
        back_lidar_dist += constants.BACK_MOUNT_LIDAR_TO_INTERSECTION_CM

        # compute angle and distance to ground
        def back_angle_and_height(x: Tuple[float, float]) -> np.ndarray:
            alpha, h = x
            return (
                h/back_ultrasonic_dist - np.cos(np.deg2rad(alpha)),
                h/back_lidar_dist - np.cos(np.deg2rad(constants.BACK_MOUNT_LIDAR_ULTRASONIC_ANGLE))
            )
        alpha, sensor_intersection_height = scipy.optimize.fsolve(
            back_angle_and_height,
            (Calibration.BACK_MOUNT_PITCH_GUESS, Calibration.BACK_MOUNT_HEIGHT_GUESS)
        )
        back_lidar_pitch = alpha + constants.BACK_MOUNT_LIDAR_ULTRASONIC_ANGLE

        rospy.loginfo('Back Pitch Angle (deg): %f', back_lidar_pitch)
        rospy.loginfo('Sensor Intersection Height (cm): %f', sensor_intersection_height)

        back_pivot_to_lidar_z_offset = constants.BACK_MOUNT_PIVOT_TO_LIDAR_DIST_CM*np.cos(
            np.radians(constants.BACK_MOUNT_PIVOT_TO_LIDAR_ANGLE + back_lidar_pitch)
        )  # should be generally positive
        back_pivot_to_lidar_x_offset = constants.BACK_MOUNT_PIVOT_TO_LIDAR_DIST_CM*np.sin(
            np.radians(constants.BACK_MOUNT_PIVOT_TO_LIDAR_ANGLE + back_lidar_pitch)
        )  # should always be negative

        sensor_intersection_to_lidar_z_offset = (
            -constants.BACK_MOUNT_ULTRASONIC_TO_INTERSECTION_CM*np.cos(np.radians(back_lidar_pitch))
        ) 

        # convert calibrated height for the back mount from cm to m
        sensor_intersection_height /= 100
        back_pivot_to_lidar_z_offset /= 100
        back_pivot_to_lidar_x_offset /= 100
        sensor_intersection_to_lidar_z_offset /= 100

        # construct transform for back lidar
        back_lidar_x_offset = constants.BACK_PIVOT_X_OFFSET + back_pivot_to_lidar_x_offset
        back_lidar_z_offset = constants.BACK_PIVOT_Z_OFFSET + back_pivot_to_lidar_z_offset
        back_tf = self._back_lidar_tf_matrix(
            back_lidar_pitch,
            np.array([back_lidar_x_offset, 0, back_lidar_z_offset])
        )

        # calculate height of wheelchair frame from ground plane
        back_lidar_height = sensor_intersection_height + sensor_intersection_to_lidar_z_offset
        wheelchair_to_ground_z = back_lidar_height - back_pivot_to_lidar_z_offset - constants.BACK_PIVOT_Z_OFFSET

        rospy.loginfo('Back Lidar Height (cm): %f', back_lidar_height * 100)
        rospy.loginfo('Wheelchair to Ground Height (cm): %f', wheelchair_to_ground_z * 100)

        # extract ground planes from each lidar
        back_ground_plane_eqn = self._get_ground_plane(LiDAR.BACK)
        left_ground_plane_eqn = self._get_ground_plane(LiDAR.LEFT)
        right_ground_plane_eqn = self._get_ground_plane(LiDAR.RIGHT)

        # find rotation to make ground plane equivalent to xy plane
        e_3 = np.array([0, 0, 1])

        # compute plane normals
        left_ground_plane_normal = self._get_plane_normal_vector(left_ground_plane_eqn)
        right_ground_plane_normal = self._get_plane_normal_vector(right_ground_plane_eqn)

        R_left_ground_plane = self._rotation_matrix_for_plane(left_ground_plane_normal, e_3)
        R_right_ground_plane = self._rotation_matrix_for_plane(right_ground_plane_normal, e_3)

        # build transformation matrices from initial guess
        front_left_tf, front_right_tf = self._front_lidar_tfs_initial_guess()

        # apply rotation to align plane
        front_left_tf = self._apply_rotation_to_tf(front_left_tf, R_left_ground_plane)
        front_right_tf = self._apply_rotation_to_tf(front_right_tf, R_right_ground_plane)

        # calculate distance to ground plane of each lidar
        back_plane_height = back_ground_plane_eqn[3]
        left_plane_height = left_ground_plane_eqn[3]
        right_plane_height = right_ground_plane_eqn[3]

        assert(back_plane_height >= 0)
        assert(left_plane_height >= 0)
        assert(right_plane_height >= 0)

        left_lidar_z_offset = back_lidar_z_offset + left_plane_height - back_plane_height
        right_lidar_z_offset = back_lidar_z_offset + right_plane_height - back_plane_height

        # update z offset for left and right lidar
        front_left_tf[2,3] = left_lidar_z_offset
        front_right_tf[2,3] = right_lidar_z_offset

        rospy.loginfo('Back Z Offset (cm): %f', back_lidar_z_offset * 100)
        rospy.loginfo('Left Z Offset (cm): %f', left_lidar_z_offset * 100)
        rospy.loginfo('Right Z Offset (cm): %f', right_lidar_z_offset * 100)

        # wait for 20 while user reorients wheelchair to detect forward orientation
        rospy.loginfo('Please reorient the mobility device towards a wall...')
        rospy.sleep(12)
        rospy.loginfo('Will continue calibration shortly...')
        rospy.sleep(3)
        rospy.loginfo('Continuing calibration...')

        # extract wall planes from each lidar
        left_wall_plane_eqn = self._get_wall_plane(LiDAR.LEFT, R_left_ground_plane, wheelchair_to_ground_z + left_lidar_z_offset)
        right_wall_plane_eqn = self._get_wall_plane(LiDAR.RIGHT, R_right_ground_plane, wheelchair_to_ground_z + right_lidar_z_offset)

        # find rotation to make wall plane normal to x axis
        e_1 = np.array([1, 0, 0])

        # compute plane normals
        left_wall_plane_normal = self._get_plane_normal_vector(left_wall_plane_eqn)
        right_wall_plane_normal = self._get_plane_normal_vector(right_wall_plane_eqn)

        # ignore z components in rotation
        left_wall_plane_normal[2] = 0
        right_wall_plane_normal[2] = 0

        R_left_wall_plane = self._rotation_matrix_for_plane(left_wall_plane_normal, e_1)
        R_right_wall_plane = self._rotation_matrix_for_plane(right_wall_plane_normal, e_1)

        # apply rotation to align plane
        front_left_tf = self._apply_rotation_to_tf(front_left_tf, R_left_wall_plane)
        front_right_tf = self._apply_rotation_to_tf(front_right_tf, R_right_wall_plane)

        # format transforms as geometry_msgs.TransformStamped
        formatted_tfs = []
        formatted_tfs.append(self._ground_to_wheelchair(wheelchair_to_ground_z))
        formatted_tfs.append(self._format_tf(constants.FRONT_LEFT_FRAME, front_left_tf))
        formatted_tfs.append(self._format_tf(constants.FRONT_RIGHT_FRAME, front_right_tf))
        formatted_tfs.append(self._format_tf(constants.BACK_FRAME, back_tf))
        # publish transforms
        # all have to be published at once due to a bug with StaticTransformBroadcaster
        # https://answers.ros.org/question/287469/unable-to-publish-multiple-static-transformations-using-tf/
        self._tf_publisher.sendTransform(formatted_tfs)
        rospy.loginfo("Published calibrated transforms")

        # debugging !!
        from std_msgs.msg import Header
        from geometry_msgs.msg import PolygonStamped, Point32

        frames = [
            constants.BACK_FRAME,
            constants.FRONT_LEFT_FRAME,
            constants.FRONT_RIGHT_FRAME,
        ]
        topic_postfix = [
            'back',
            'front_left',
            'front_right',
        ]
        lidars = [LiDAR.BACK, LiDAR.LEFT, LiDAR.RIGHT ]
        plane_eqs = [back_ground_plane_eqn, left_ground_plane_eqn, right_ground_plane_eqn]

        filt_pubs = []
        plane_pubs = []

        filts = []
        planes = []

        for i, fr in enumerate(frames):
            # filtered points
            filt_topic = '/filtered_' + topic_postfix[i]
            filt_pubs.append(
                rospy.Publisher(filt_topic, PointCloud2, queue_size=1)
            )

            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = fr
            filts.append(
                pc2.create_cloud_xyz32(h, self.filtered_pts[lidars[i]])
            )

            # detected ground planes
            plane_topic = '/plane_' + topic_postfix[i]
            plane_pubs.append(
                rospy.Publisher(plane_topic, PolygonStamped, queue_size=5)
            )

            p = PolygonStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = fr
            p.polygon.points = []
            for (y, z) in [(0.1, 0.1), (0.1, -0.1), (-0.1, -0.1), (-0.1, 0.1)]:
                pt = Point32()
                pt.y = y
                pt.z = z
                pt.x = (
                    plane_eqs[i][1] * y + plane_eqs[i][2] * z + plane_eqs[i][3]
                ) / -plane_eqs[i][0]
                p.polygon.points.append(pt)
            planes.append(p)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            for i in range(3):
                plane_pubs[i].publish(planes[i])
                filt_pubs[i].publish(filts[i])

            rate.sleep()

        return formatted_tfs

    def _apply_rotation_to_tf(self, T: np.ndarray, R: np.ndarray) -> np.ndarray:
        assert(T.shape == (4, 4))
        assert(R.shape == (3,3))
        
        T[0:3,0:3] = R@T[0:3,0:3]
        return T

    def _front_lidar_tfs_initial_guess(self) -> Tuple[np.ndarray, np.ndarray]:
        front_left_tf = self._front_lidar_tf_matrix(
            np.array([
                constants.FRONT_MOUNT_X_OFFSET,
                constants.FRONT_MOUNT_Y_OFFSET,
                -1  # does not require a guess, calculated by calibration
            ])
        )
        front_right_tf = self._front_lidar_tf_matrix(
            np.array([
                constants.FRONT_MOUNT_X_OFFSET,
                -constants.FRONT_MOUNT_Y_OFFSET,
                -1  # does not require a guess, calculated by calibration
            ])
        )
        return (front_left_tf, front_right_tf)

    """Convenience method to generate front lidar transformation matrix"""
    def _front_lidar_tf_matrix(self, offset: np.ndarray) -> np.ndarray:
        T = np.identity(4)
        T[0:3, 3] = offset  # add translation offset
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
        T[0:3, 3] = offset  # add translation offset
        return T

    def _ground_to_wheelchair(self, wheelchair_z_offset: float) -> TransformStamped:
        # create message with frame ids
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = constants.GROUND_PLANE_FRAME
        t.child_frame_id = constants.WHEELCHAIR_FRAME

        # wheelchair is directly above ground plane
        t.transform.translation.z = wheelchair_z_offset
        # have to set homogenous coordinate otherwise rotation is infinite which does not make sense
        t.transform.rotation.w = 1

        return t

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
def run() -> List[TransformStamped]:
    calibration = Calibration()
    return calibration.run()
    
    # don't spin after calibration is run so node shuts down
