#include <ros/ros.h>
#include <signal.h>

#include "i2c/I2C_Manager.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, i2c_node::NODE_NAME);
    ros::NodeHandle nh;

    i2c_node::I2C_Manager i2c(nh);
    i2c.init();

    return 0;
}
