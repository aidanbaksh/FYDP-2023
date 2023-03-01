#pragma once

#include <thread>
#include <atomic>
#include <array>
#include <string>
#include <tuple>

#include <ros/ros.h>
#include "i2c/i2c.h" // libi2c

namespace i2c_node {

inline static const std::string NODE_NAME = "i2c_manager"; // must be extern to avoid link errors


namespace imu_corrections {
    // acceleration data correction
    static constexpr int16_t ACCEL_X = -250;
    static constexpr int16_t ACCEL_Y = 36;
    static constexpr int16_t ACCEL_Z = 1200;

    // temperature correction
    static constexpr int16_t TEMP = -1400;

    // gyro correction
    static constexpr int16_t GYRO_X = -335;
    static constexpr int16_t GYRO_Y = 250;
    static constexpr int16_t GYRO_Z = 170;
};


namespace addr {
    static constexpr unsigned int ARDUINO = 0x10;

    namespace imu {
        static constexpr unsigned int DEVICE = 0x68;

        namespace internal {
            static constexpr unsigned int ACCEL = 0x3B;
            static constexpr unsigned int GYRO = 0x43;
            static constexpr unsigned int RESET = 0x6B;
            static constexpr unsigned int ACCEL_CONFIG = 0x1C;
            static constexpr unsigned int GYRO_CONFIG = 0x1B;
        };
    };
};


class I2C_Manager {
    static constexpr size_t PUBLISH_INTERVAL_MS = 100;

    static constexpr unsigned int BUS_NUM = 0;
    inline static const std::string BUS_NAME = "/dev/i2c-" + std::to_string(I2C_Manager::BUS_NUM);

    static constexpr size_t NUM_ULTRASONICS = 2; // 1 per ultrasonic
    static const std::array<std::string, NUM_ULTRASONICS> ULTRASONIC_TOPICS;

    static constexpr size_t NUM_IMU_AXES = 3;
    inline static const std::string IMU_TOPIC = "/imu";

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

    // wrapper on libi2c function that initializes device struct
    void init_device(I2CDevice &, unsigned short, unsigned int);

    // wrapper on libi2c function that writes a single byte to configure the IMU
    bool configure_imu(unsigned int, int8_t);

    // wrapper on libi2c function that does logging and computes sizeof buffer
    bool read_device(const I2CDevice&, unsigned int, void *);

    // thread that actually communicates with devices
    void get_data();


    // convert acceleration to angle
    std::pair<double, double> get_angle(const int16_t, const int16_t, const int16_t) const;

    ros::NodeHandle& nh;

    std::atomic_flag data_available, stopped;
    ros::Timer timer;
    std::array<ros::Publisher, NUM_ULTRASONICS> ultrasonic_publishers;
    ros::Publisher imu_publisher;

    int bus_fd;
    I2CDevice arduino, imu;

    int32_t ultrasonic_buffer[NUM_ULTRASONICS];

    int16_t imu_buffer[NUM_IMU_AXES];
    std::tuple<int16_t, int16_t, int16_t> imu_accel;
    std::tuple<int16_t, int16_t, int16_t> imu_gyro;

    std::thread device_reader;
};

}
