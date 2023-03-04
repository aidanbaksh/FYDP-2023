# frame ids
GROUND_PLANE_FRAME: str = 'ground_plane'
WHEELCHAIR_FRAME: str = 'wheelchair'
FRONT_LEFT_FRAME: str = 'front_left_lidar'
FRONT_RIGHT_FRAME: str = 'front_right_lidar'
BACK_FRAME: str = 'back_lidar'

# lidar topics
BACK_LIDAR_2D_TOPIC: str = '/scan_2D_0'
BACK_LIDAR_3D_TOPIC: str = '/scan_3D_0'
BACK_ULTRASONIC_TOPIC: str = '/ultrasonics/back'

# these values are known from the sensor mount design
BACK_PIVOT_X_OFFSET: float = -0.15  # 1cm
BACK_PIVOT_Z_OFFSET: float = -0.1  # 1cm
BACK_MOUNT_LIDAR_ULTRASONIC_ANGLE: float = 30

# distance from measurement point to intersection of sensors
BACK_MOUNT_ULTRASONIC_TO_INTERSECTION_CM: float = 6.963
BACK_MOUNT_LIDAR_TO_FRONT_CM: float = 1.950
BACK_MOUNT_LIDAR_TO_INTERSECTION_CM: float = 5.073
BACK_MOUNT_PIVOT_TO_LIDAR_DIST_CM: float = 3.5
BACK_MOUNT_PIVOT_TO_LIDAR_ANGLE: float = -58
