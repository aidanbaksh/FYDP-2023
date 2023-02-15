#pragma once

#include <thread>
#include <atomic>
#include <array>
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

    static constexpr size_t NUM_ULTRASONICS = 2; // 1 per ultrasonic
    static const std::array<std::string, NUM_ULTRASONICS> ULTRASONIC_TOPICS;

    static constexpr size_t PUBLISH_INTERVAL_MS = 100;

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
    void timer_callback(const ros::TimerEvent&);

    // thread that actually communicates with devices
    void read_devices();

    ros::NodeHandle& nh;

    std::atomic_flag data_available, stopped;
    ros::Timer timer;
    std::array<ros::Publisher, NUM_ULTRASONICS> publishers;

    int bus_fd;
    I2CDevice arduino, imu;

    int32_t ultrasonic_buffer[NUM_ULTRASONICS];
    std::thread device_reader;
};

}
