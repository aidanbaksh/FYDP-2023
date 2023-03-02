# frame ids
WHEELCHAIR_FRAME: str = 'wheelchair'
FRONT_LEFT_FRAME: str = 'front_left_lidar'
FRONT_RIGHT_FRAME: str = 'front_right_lidar'
BACK_FRAME: str = 'back_lidar'

# these values are known from the sensor mount design
BACK_MOUNT_X_OFFSET: float = -0.039  # 39mm
BACK_MOUNT_Z_OFFSET: float = -0.025  # 2.5cm
BACK_MOUNT_LIDAR_ULTRASONIC_ANGLE: float = 30

# distance from measurement point to intersection of sensors
LIDAR_MOUNT_ULTRASONIC_TO_INTERSECTION: float = 6.5
LIDAR_MOUNT_LIDAR_TO_INTERSECTION: float = 5