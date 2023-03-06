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
    // NOTE: these correction factors depend on the full scale range of each sensor
    // https://github.com/adafruit/Adafruit_MPU6050/blob/master/Adafruit_MPU6050.cpp#L660-L689
    // Accelerometer scale is +-2g
    // Gyroscope scale is +- 250 deg/s

    static constexpr double ACCEL_SCALE = 16384;
    static constexpr double GYRO_SCALE = 131;

    static constexpr double SENSORS_GRAVITY_STANDARD = 9.80665;
    static constexpr double SENSORS_DPS_TO_RADS = M_PI / 180.0;
};


namespace addr {
    namespace arduino {
        static constexpr unsigned int DEVICE = 0x10;
    };

    namespace imu {
        static constexpr unsigned int DEVICE = 0x68;

        // See internal register map
        // https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
        namespace internal {
            // used for device reset and clock select
            static constexpr unsigned int PWR_MGMT_1 = 0x6B;
            // used to reset individual sensors and initialize serial interface
            static constexpr unsigned int SIGNAL_PATH_RESET = 0x68;
            // used to reset individual sensors and initialize serial interface
            static constexpr unsigned int SMPRT_DIV = 0x19;
            // configures digital LPF for both accelerometer and gyroscope
            static constexpr unsigned int CONFIG = 0x1A;
            // configures scale of accelerometer
            static constexpr unsigned int ACCEL_CONFIG = 0x1C;
            // configures scale of gyro
            static constexpr unsigned int GYRO_CONFIG = 0x1B;

            // technically the address of ACCEL_XOUT_H but we read all 6 ACCEL_OUT registers at a time
            static constexpr unsigned int ACCEL_OUT = 0x3B;
            // technically the address of GYRO_XOUT_H but we read all 6 ACCEL_OUT registers at a time
            static constexpr unsigned int GYRO_OUT = 0x43;
        };
    };
};


class I2C_Manager {
    static constexpr size_t PUBLISH_INTERVAL_MS = 20; // 20Hz

    static constexpr unsigned int BUS_NUM = 0;
    inline static const std::string BUS_NAME = "/dev/i2c-" + std::to_string(I2C_Manager::BUS_NUM);

    static constexpr size_t NUM_ULTRASONICS = 7;
    static const std::array<std::string, NUM_ULTRASONICS> ULTRASONIC_TOPICS;

    static constexpr size_t NUM_IMU_AXES = 3;
    inline static const std::string IMU_TOPIC = "/imu";

public:
    I2C_Manager(ros::NodeHandle&, ros::MultiThreadedSpinner&);

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

    // ROS node handle
    ros::NodeHandle& nh;

    // flags for coordination between i2c reader thread and ROS timer callback
    std::atomic_flag data_available, stopped;
    ros::Timer timer;

    // ROS publishers
    std::array<ros::Publisher, NUM_ULTRASONICS> ultrasonic_publishers;
    ros::Publisher imu_publisher;
    
    // needed to avoid being blocking by audio warnings
    ros::MultiThreadedSpinner& spinner; 

    // i2c bus file descriptor and devices
    int bus_fd;
    I2CDevice arduino, imu;

    // buffer of ultrasonic readings
    uint8_t ultrasonic_buffer[NUM_ULTRASONICS];
    // buffer used to read accel and gyro
    // each reading is stored in 2 registers, so we have to read them individual then combine
    int8_t imu_buffer[NUM_IMU_AXES*2];
    std::tuple<double, double, double> imu_accel; // reading in m/s^2
    std::tuple<double, double, double> imu_gyro; // reading in rad/s

    std::thread device_reader;
};

}
