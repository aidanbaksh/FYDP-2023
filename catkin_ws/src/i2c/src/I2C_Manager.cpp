#include "i2c/I2C_Manager.hpp"

#include <cassert>
#include <cstring>  // needed for std::memset
#include <iostream>

namespace i2c_node {

I2C_Manager::I2C_Manager(ros::NodeHandle& nh):
    nh(nh), data_available(ATOMIC_FLAG_INIT), device_reader(&I2C_Manager::read_devices, this)
{
    // initially, no data is available
    data_available.clear(std::memory_order_seq_cst);
}

I2C_Manager::~I2C_Manager() {
    close();
}

void I2C_Manager::init() {
    // open i2c bus
    if ((bus_fd = i2c_open(I2C_Manager::BUS_NAME.data())) == -1) {
        std::cout << "Error opening i2c bus " << I2C_Manager::BUS_NAME << std::endl;
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
    std::cout << "Arduino: " << i2c_get_device_desc(&arduino, i2c_dev_desc, sizeof(i2c_dev_desc)) << std::endl;
    std::cout << "IMU: " << i2c_get_device_desc(&imu, i2c_dev_desc, sizeof(i2c_dev_desc)) << std::endl;
}

void I2C_Manager::close() {
    i2c_close(bus_fd);
}

void I2C_Manager::timer_callback() {
    // spin the node while data is not available
    while (!data_available.test()) {
        ros::spin();
    }

    // TODO: publish data

    // consumed the data, no longer available
    data_available.clear();
}

void I2C_Manager::read_devices() {
    while (true) {
        // wait until the node has published the data before making another request to the hardware
        data_available.wait(true, std::memory_order_seq_cst);

        size_t i = 0;
        ssize_t ret = 0;
        int32_t buf[2];
        size_t buf_size = sizeof(buf);
        std::memset(buf, 0, buf_size);

        ret = i2c_ioctl_read(&arduino, 0, buf, buf_size);
        if (ret == -1 || (size_t)ret != buf_size) {
            std::cout << "Write i2c error!" << std::endl << "\tGot: " << ret << std::endl;
            exit(-4);
        }

        std::cout << "buf size is " << buf_size << std::endl;

        std::cout << "Got data:" << std::endl;
        for (size_t i = 0; i < 2; ++i) {
            std::cout << buf[i] << " ";
        }
        std::cout << std::endl;


        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        bool data_was_available = data_available.test_and_set(std::memory_order_seq_cst);
        assert(data_was_available == false);
    }
}

}
