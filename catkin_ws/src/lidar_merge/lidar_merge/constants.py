from enum import Enum


class LiDAR(Enum):
    LEFT = 'left',
    RIGHT = 'right',
    BACK = 'back',


# frame ids
GROUND_PLANE_FRAME: str = 'ground_plane'
WHEELCHAIR_FRAME: str = 'wheelchair'
FRONT_LEFT_FRAME: str = 'front_left_lidar'
FRONT_RIGHT_FRAME: str = 'front_right_lidar'
BACK_FRAME: str = 'back_lidar'

# lidar topics
BACK_LIDAR_2D_TOPIC: str = '/scan_2D_0'
BACK_ULTRASONIC_TOPIC: str = '/ultrasonics/back'

LIDAR_3D_TOPCS = {
    LiDAR.BACK: '/scan_3D_0',
    LiDAR.LEFT: '/scan_3D_1',
    LiDAR.RIGHT: '/scan_3D_2',
}

# these values are known from the sensor mount design
BACK_PIVOT_X_OFFSET: float = -0.15  # 1cm
BACK_PIVOT_Z_OFFSET: float = -0.1  # 1cm
BACK_MOUNT_LIDAR_ULTRASONIC_ANGLE: float = 30

# this value is set by the technician / installer
FRONT_MOUNT_X_OFFSET: float = 0.6  # approx 2ft

# distance from measurement point to intersection of sensors
BACK_MOUNT_ULTRASONIC_TO_INTERSECTION_CM: float = 6.963
BACK_MOUNT_LIDAR_TO_FRONT_CM: float = 1.950
BACK_MOUNT_LIDAR_TO_INTERSECTION_CM: float = 5.073
BACK_MOUNT_PIVOT_TO_LIDAR_DIST_CM: float = 3.5
BACK_MOUNT_PIVOT_TO_LIDAR_ANGLE: float = -58

GROUND_PLANE_RANSAC_THRESHOLD: float = 0.01
LIDAR_FILTER_X_MIN_THRESHOLD: float = 0.225
LIDAR_FILTER_X_MAX_THRESHOLD: float = 0.75
