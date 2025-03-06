#pragma once

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cstdint>

#ifdef TEST_MODE
  // Declare your custom functions
  extern "C" int custom_i2c_open(const char* path, int flags);
  extern "C" int custom_i2c_close(int fd);
  extern "C" int custom_i2c_ioctl(int fd, unsigned long request, uint8_t arg);
  extern "C" ssize_t custom_i2c_read(int fd, void* buf, size_t count);
  extern "C" ssize_t custom_i2c_write(int fd, const void* buf, size_t count);
#endif


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
