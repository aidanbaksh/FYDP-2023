#include "i2c/I2C_Manager.hpp"

#include <cassert>
#include <cstring>  // needed for std::memset
#include <iostream>

#include "std_msgs/Float32.h"

namespace i2c_node {

// the ultrasonic topics must be listed in the order the Jetson recieves data
const std::array<std::string, I2C_Manager::NUM_ULTRASONICS> I2C_Manager::ULTRASONIC_TOPICS = {
    "/ultrasonics/right",
    "/ultrasonics/left",
};

I2C_Manager::I2C_Manager(ros::NodeHandle& nh):
    nh(nh),
    data_available(ATOMIC_FLAG_INIT),
    stopped(ATOMIC_FLAG_INIT),
    timer(
        nh.createTimer(
            ros::Duration(I2C_Manager::PUBLISH_INTERVAL_MS/1000.0),
            &I2C_Manager::timer_callback, this
        )
    ),
    publishers(),
    bus_fd(-1),
    arduino(),
    imu(),
    device_reader(&I2C_Manager::read_devices, this)

{
    // thread is not stopped;
    stopped.clear();
    
    // initially, no data is available 
    // set the flag so the device_reader thread blocks until init is called
    data_available.test_and_set(std::memory_order_seq_cst);

    // create publishers for ultrasonic topics
    for (size_t i = 0; i < NUM_ULTRASONICS; ++i) {
        publishers[i] = nh.advertise<std_msgs::Float32>(ULTRASONIC_TOPICS[i], 5);
    }

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

    // initialize arduino i2c device struct
    std::memset(&arduino, 0, sizeof(arduino));
    i2c_init_device(&arduino);
    arduino.bus = bus_fd;
    arduino.addr = I2C_Manager::ARDUINO_ADDR & 0x7f;
    arduino.iaddr_bytes = 0;
    
    // initialize imu i2c device struct
    std::memset(&imu, 0, sizeof(imu));
    i2c_init_device(&imu);
    imu.bus = bus_fd;
    imu.addr = I2C_Manager::IMU_ADDR & 0x7f;
    imu.iaddr_bytes = 0;

    // print i2c device descriptions
    char i2c_dev_desc[128];
    ROS_INFO_STREAM("Arduino: " << i2c_get_device_desc(&arduino, i2c_dev_desc, sizeof(i2c_dev_desc)) << std::endl);
    ROS_INFO_STREAM("IMU: " << i2c_get_device_desc(&imu, i2c_dev_desc, sizeof(i2c_dev_desc)) << std::endl);

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

    // publish data on respective topic
    std_msgs::Float32 distance;
    for (size_t i = 0; i < NUM_ULTRASONICS; ++i) {
        distance.data = ultrasonic_buffer[i];
        publishers[i].publish(distance);
    }

    // consumed the data, no longer available
    data_available.clear();
    data_available.notify_one();
}

void I2C_Manager::read_devices() {
    while (!stopped.test()) {
        // wait until the node has published the data before making another request to the hardware
        data_available.wait(true, std::memory_order_seq_cst);

        size_t i = 0;
        ssize_t ret = 0;

        ret = i2c_ioctl_read(&arduino, 0, ultrasonic_buffer, sizeof(ultrasonic_buffer));
        if (ret == -1 || (size_t)ret != sizeof(ultrasonic_buffer)) {
            ROS_ERROR("Read i2c error!\n");
            continue;
        }

        std::cout << "Got data:" << std::endl;
        for (size_t i = 0; i < 2; ++i) {
            std::cout << ultrasonic_buffer[i] << " ";
        }
        std::cout << std::endl;

        // sleep at half the rate of the timer 
        // this way, data is available whenever the timer callback is ready
        std::this_thread::sleep_for(std::chrono::milliseconds(I2C_Manager::PUBLISH_INTERVAL_MS/2));

        bool data_was_available = data_available.test_and_set(std::memory_order_seq_cst);
        assert(data_was_available == false);
    }
}

}
