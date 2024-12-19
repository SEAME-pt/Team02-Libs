#pragma once

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

class SPI
{
  private:
    std::string _spiDevice;
    int _spiFd;

  public:
    SPI();
    ~SPI();
    SPI(const SPI& originalSPI);
    SPI& operator=(const SPI& originalSPI);

    void init(const std::string& spiDevice);
    void writeByte(uint8_t deviceAddress, uint8_t reg, uint8_t value);
    uint8_t readByte(uint8_t deviceAddress);
};
