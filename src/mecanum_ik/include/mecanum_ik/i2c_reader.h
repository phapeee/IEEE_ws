#ifndef I2C_READER_H
#define I2C_READER_H

#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <ros/ros.h>
#include <limits>

// I2C setup
#define I2C_BUS "/dev/i2c-1"  // I2C bus on Raspberry Pi

// Define a union to handle speed data
union speed_u {
    uint8_t b[sizeof(float)];
    float d;
};

class I2CReader {
public:
    I2CReader(uint8_t i2c_address);
    ~I2CReader();

    bool sendDesiredSpeed(double desired_speed);
    double readMeasuredSpeed();

private:
    int i2c_fd;
    uint8_t i2c_address;
};

#endif  // I2C_READER_H
