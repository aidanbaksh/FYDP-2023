#include <ros/ros.h>

#include "lidar_merge/PointCloudMerger.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, lidar_merge::NODE_NAME);
    ros::NodeHandle nh;

    lidar_merge::PointCloudMerger merger(nh);

    // spin the node so it never exits
    // effectively equivalent to joining the device reader thread
    ros::spin();

    return 0;
}