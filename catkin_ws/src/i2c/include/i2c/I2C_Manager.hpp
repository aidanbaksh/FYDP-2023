#pragma once

#include <thread>
#include <atomic>
#include <string>

#include <ros/ros.h>
#include "i2c/i2c.h" // libi2c

namespace i2c_node {

inline static const std::string NODE_NAME = "i2c_manager"; // must be extern to avoid link errors


class I2C_Manager {
    static constexpr unsigned int BUS_NUM = 0;
    inline static const std::string BUS_NAME = "/dev/i2c-" + std::to_string(I2C_Manager::BUS_NUM);

    static constexpr unsigned int ARDUINO_ADDR = 0x10;
    static constexpr unsigned int IMU_ADDR = 0x20;

public:
    I2C_Manager(ros::NodeHandle&);

    // disallow copy and move
    I2C_Manager(I2C_Manager &) = delete;
    I2C_Manager(I2C_Manager &&) = delete;

    ~I2C_Manager();

    void init();
    void close();

private:
    // intermittendly publishes updates
    void timer_callback();

    // thread that actually communicates with devices
    void read_devices();

    ros::NodeHandle& nh;

    std::atomic_flag data_available;
    std::thread device_reader;

    int bus_fd;
    I2CDevice arduino, imu;
};

}
