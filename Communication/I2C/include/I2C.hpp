#pragma once

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cstdint>

class I2C
{
  private:
    std::string _i2cDevice;
    int _i2cFd;

  public:
    I2C();
    ~I2C();

    int getFd();

    void init(const std::string& i2cDevice);

    void writeByte(uint8_t deviceAddress, uint8_t reg, uint8_t value);
    void writeMessage(uint8_t deviceAddress, uint8_t *buffer);
    
    uint8_t readByte(uint8_t deviceAddress);
    void readRegister(uint8_t deviceAddress, uint8_t registerAddr, uint8_t *data);
};
