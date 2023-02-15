#include <ros/ros.h>

#include "i2c/I2C_Manager.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, i2c_node::NODE_NAME);
    ros::NodeHandle nh;

    i2c_node::I2C_Manager i2c(nh);
    i2c.init();

    // spin the node so it never exits
    // effectively equivalent to joining the device reader thread
    ros::spin();

    return 0;
}
