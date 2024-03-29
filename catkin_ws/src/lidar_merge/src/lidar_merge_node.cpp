#include <ros/ros.h>
#include "lidar_merge/PointCloudMerger.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, lidar_merge::NODE_NAME);
    ros::NodeHandle nh;

    // wait for calibration to complete
    // this is guaranteed to be at least 20 seconds
    ros::Duration(20).sleep();

    lidar_merge::PointCloudMerger merger(nh);
    merger.get_lidar_transforms();
    merger.start();

    // spin the node so it never exits
    ros::spin();

    return 0;
}