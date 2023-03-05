#!/usr/bin/env python3
# essentially just a wrapper script that calls lidar_merge.calibration.run()

from lidar_merge import calibration, serialize


def main(args=None):
    # run calibration
    tfs = calibration.run()
    # write calibrated transforms to file
    serialize.write(tfs)



if __name__ == '__main__':
    main()
