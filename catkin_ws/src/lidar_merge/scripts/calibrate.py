#!/usr/bin/env python3
# essentially just a wrapper script that calls lidar_merge.calibration.run()

from lidar_merge import calibration


def main(args=None):
    calibration.run()


if __name__ == '__main__':
    main()
