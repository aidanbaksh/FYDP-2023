#include "i2c/I2C_Manager.hpp"
#include "i2c/IMU.h"

#include <cassert>
#include <cstring>  // needed for std::memset
#include <iostream>

#include "std_msgs/Float32.h"

namespace i2c_node {

// the ultrasonic topics must be listed in the order the Jetson recieves data
const std::array<std::string, I2C_Manager::NUM_ULTRASONICS> I2C_Manager::ULTRASONIC_TOPICS = {
    "/ultrasonics/right",
    "/ultrasonics/front_right",
    "/ultrasonics/front_right_side",
    "/ultrasonics/back",
    "/ultrasonics/front_left_side",
    "/ultrasonics/front_left",
    "/ultrasonics/left",
};

I2C_Manager::I2C_Manager(ros::NodeHandle& nh):
    nh(nh),
    data_available(ATOMIC_FLAG_INIT),
    stopped(ATOMIC_FLAG_INIT),
    timer(),
    ultrasonic_publishers(),
    imu_publisher(),
    bus_fd(-1),
    arduino(),
    imu(),
    device_reader(&I2C_Manager::get_data, this)

{
    // thread is not stopped;
    stopped.clear();
    
    // initially, no data is available 
    // set the flag so the device_reader thread blocks until init is called
    data_available.test_and_set(std::memory_order_seq_cst);

    // create publishers for ultrasonics and imu
    for (size_t i = 0; i < NUM_ULTRASONICS; ++i) {
        ultrasonic_publishers[i] = nh.advertise<std_msgs::Float32>(ULTRASONIC_TOPICS[i], 5);
    }
    imu_publisher = nh.advertise<i2c::IMU>(IMU_TOPIC, 5);

    // clear data buffer
    std::memset(ultrasonic_buffer, 0, sizeof(ultrasonic_buffer));
}

I2C_Manager::~I2C_Manager() {
    // set stop flag for thread
    stopped.test_and_set(std::memory_order_seq_cst);
    // unblock thread since it may be waiting on the timer to consume the last piece of data
    data_available.clear();
    data_available.notify_one();
    // let the thread finish the current transaction
    device_reader.join();
    
    // close i2c bus
    close();
}

void I2C_Manager::init() {
    // open i2c bus
    if ((bus_fd = i2c_open(I2C_Manager::BUS_NAME.data())) == -1) {
        ROS_FATAL_STREAM("Error opening i2c bus " << I2C_Manager::BUS_NAME << std::endl);
        exit(-3);
    }
    ROS_INFO("Opened I2C Bus!");

    // initialize device structs
    init_device(arduino, addr::arduino::DEVICE, 0);
    init_device(imu, addr::imu::DEVICE, 1);

    // print i2c device descriptions
    char i2c_dev_desc[128];
    ROS_INFO_STREAM("Arduino: " << i2c_get_device_desc(&arduino, i2c_dev_desc, sizeof(i2c_dev_desc)));
    ROS_INFO_STREAM("IMU: " << i2c_get_device_desc(&imu, i2c_dev_desc, sizeof(i2c_dev_desc)));

    // configuration is mostly based on Adafruit library
    // https://github.com/adafruit/Adafruit_MPU6050/blob/master/Adafruit_MPU6050.cpp#L105-L141

    // reset the imu and disable the temperature sensor
    configure_imu(addr::imu::internal::PWR_MGMT_1, 0b10001000);
    ros::Duration(0.1).sleep(); // sleep for 100ms
    // reset individual sensors and initialize serial interface
    configure_imu(addr::imu::internal::SIGNAL_PATH_RESET, 0b00000111);
    ros::Duration(0.1).sleep(); // sleep for 100ms
    // set sample rate divisor, converts clock rate into measurement rate
    // TODO: this will depend on our clock rate
    configure_imu(addr::imu::internal::SMPRT_DIV, 0);

    // TODO: this got changed
    // set filter bandwidth to 44Hz for accel and 42Hz for gyro
    // ROS node runs at 20 Hz so this rejects anything faster without introducing delay
    configure_imu(addr::imu::internal::CONFIG, 0b00000000);

    // set the gyro scale to 250 deg/s
    configure_imu(addr::imu::internal::GYRO_CONFIG, 0b00000000);
    // set the accelerometer scale to +- 2g
    configure_imu(addr::imu::internal::ACCEL_CONFIG, 0b00000000);
    // set clock config to PLL with Gyro X reference
    configure_imu(addr::imu::internal::PWR_MGMT_1, 0b00001001);
    ros::Duration(0.1).sleep(); // sleep for 100ms

    ROS_INFO("Configured IMU!");

    // setup publish callback now that timer
    timer = nh.createTimer(
        ros::Duration(I2C_Manager::PUBLISH_INTERVAL_MS/1000.0),
        &I2C_Manager::timer_callback, this
    );

    // unblock the device_reader thread now that the i2c bus is initialized
    data_available.clear(std::memory_order_seq_cst);
    data_available.notify_one();
}

void I2C_Manager::close() {
    i2c_close(bus_fd);
}

void I2C_Manager::timer_callback(const ros::TimerEvent&) {
    // spin the node while data is not available
    while (!data_available.test()) {
        ros::spin();
    }

    // publish ultrasonic data on respective topic
    for (size_t i = 0; i < NUM_ULTRASONICS; ++i) {
        std_msgs::Float32 distance;
        distance.data = ultrasonic_buffer[i];
        ultrasonic_publishers[i].publish(distance);
    }

    // publish imu data
    i2c::IMU imu_msg;
    imu_msg.acceleration.x = std::get<0>(imu_accel);
    imu_msg.acceleration.y = std::get<1>(imu_accel);
    imu_msg.acceleration.z = std::get<2>(imu_accel);
    imu_msg.gyro.x = std::get<0>(imu_gyro);
    imu_msg.gyro.y = std::get<1>(imu_gyro);
    imu_msg.gyro.z = std::get<2>(imu_gyro);
    imu_publisher.publish(imu_msg);

    // consumed the data, no longer available
    data_available.clear();
    data_available.notify_one();
}

void I2C_Manager::get_data() {
    while (!stopped.test()) {
        // wait until the node has published the data before making another request to the hardware
        data_available.wait(true, std::memory_order_seq_cst);

        // read ultrasonic data
        if (!read_device(arduino, 0, ultrasonic_buffer)) {
            ROS_ERROR("i2c error when reading ultrasonics!\n");
        }

        // read imu acceleration 
        if (!read_device(imu, addr::imu::internal::ACCEL_OUT, imu_buffer)) {
            ROS_ERROR("i2c error when reading imu acceleration!\n");
        }
        imu_accel = std::make_tuple(
            double(imu_buffer[0] << 8 | imu_buffer[1]) / imu_corrections::ACCEL_SCALE * imu_corrections::SENSORS_GRAVITY_STANDARD,
            double(imu_buffer[2] << 8 | imu_buffer[3]) / imu_corrections::ACCEL_SCALE * imu_corrections::SENSORS_GRAVITY_STANDARD,
            double(imu_buffer[4] << 8 | imu_buffer[5]) / imu_corrections::ACCEL_SCALE * imu_corrections::SENSORS_GRAVITY_STANDARD
        );

        // read imu gyro
        if (!read_device(imu, addr::imu::internal::GYRO_OUT, imu_buffer)) {
            ROS_ERROR("i2c error when reading imu gyro!\n");
        }
        imu_gyro = std::make_tuple(
            (imu_buffer[0] << 8 | imu_buffer[1]) / imu_corrections::GYRO_SCALE * imu_corrections::SENSORS_DPS_TO_RADS,
            (imu_buffer[2] << 8 | imu_buffer[3]) / imu_corrections::GYRO_SCALE * imu_corrections::SENSORS_DPS_TO_RADS,
            (imu_buffer[4] << 8 | imu_buffer[5]) / imu_corrections::GYRO_SCALE * imu_corrections::SENSORS_DPS_TO_RADS
        );

        // std::cout
        //     << std::fixed
        //     << std::setprecision(2)
        //     << "IMU: ("
        //     << std::get<0>(imu_accel) << ", "
        //     << std::get<1>(imu_accel) << ", "
        //     << std::get<2>(imu_accel) << ")\t\tGyro: ("
        //     << std::get<0>(imu_gyro) << ", "
        //     << std::get<1>(imu_gyro) << ", "
        //     << std::get<2>(imu_gyro) << ")"
        //     << std::endl;

        // std::cout << "Got ultrasonic data:" << std::endl;
        //     std::cout << unsigned(ultrasonic_buffer[i]) << " ";
        // }
        // std::cout << std::endl;

        bool data_was_available = data_available.test_and_set(std::memory_order_seq_cst);
        assert(data_was_available == false);
    }
}

void I2C_Manager::init_device(I2CDevice &device, unsigned short address, unsigned int iaddr_bytes) {
    assert(bus_fd != -1);
    
    std::memset(&device, 0, sizeof(device));
    i2c_init_device(&device);
    device.bus = bus_fd;
    device.addr = address & 0x7f;
    device.iaddr_bytes = iaddr_bytes;
}

inline bool I2C_Manager::read_device(const I2CDevice &device, unsigned int iaddr, void *buf) {
    ssize_t ret = i2c_ioctl_read(&device, iaddr, buf, sizeof(buf));
    return (ret != -1 && (size_t)ret == sizeof(buf));
}

bool I2C_Manager::configure_imu(unsigned int iaddr, int8_t val) {
    int8_t buf[1] = {val};
    ssize_t ret = i2c_ioctl_write(&imu, iaddr, buf, sizeof(buf));
    if (ret == -1 || (size_t)ret != sizeof(buf)) {
        ROS_FATAL("i2c error when configuring imu!\n");
        return false;
    }
    return true;
}

}
