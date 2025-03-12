#include "i2c_reader.h"
#include <ros/ros.h>
#include <chrono>
#include <thread>

I2CReader::I2CReader(uint8_t i2c_address) : i2c_fd(-1), i2c_address(i2c_address) {
    // Open I2C bus
    if ((i2c_fd = open(I2C_BUS, O_RDWR)) < 0) {
        ROS_ERROR("Failed to open I2C bus: %s", I2C_BUS);
        throw std::runtime_error("Failed to open I2C bus");
    }

    // Set the I2C slave address
    if (ioctl(i2c_fd, I2C_SLAVE, i2c_address) < 0) {
        ROS_ERROR("Failed to set I2C slave address 0x%X", i2c_address);
        close(i2c_fd);
        throw std::runtime_error("Failed to set I2C slave address");
    }

    ROS_INFO("Connected to I2C slave at address 0x%X on bus %s", i2c_address, I2C_BUS);
}

I2CReader::~I2CReader() {
    if (i2c_fd >= 0) {
        close(i2c_fd);
    }
}

bool I2CReader::sendDesiredSpeed(double desired_speed) {
    union speed_u speed_data;
    speed_data.d = (float) desired_speed;

    if (write(i2c_fd, speed_data.b, sizeof(speed_data.b)) != sizeof(speed_data.b)) {
        ROS_ERROR("Failed to send desired speed to I2C slave 0x%X", i2c_address);
        return false;
    }
//    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //ROS_INFO("Sent desired speed to 0x%X: %f", i2c_address, desired_speed);
    return true;
}

double I2CReader::readMeasuredSpeed() {
    union speed_u speed_data;

    if (read(i2c_fd, speed_data.b, sizeof(speed_data.b)) != sizeof(speed_data.b)) {
        ROS_ERROR("Failed to read data from I2C slave 0x%X", i2c_address);
        return std::numeric_limits<double>::quiet_NaN();
    }

    return speed_data.d;
}
