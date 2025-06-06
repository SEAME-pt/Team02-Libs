#pragma once
#include <iostream>
#include <cstring>

// CAN-SPI register values and defines
#include "MCP_DEFS.hpp"
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>

#ifdef TEST_MODE
  // Declare your custom functions
  extern "C" int custom_open(const char* path, int flags);
  extern "C" int custom_close(int fd);
  extern "C" int custom_ioctl(int fd, unsigned long request, void* arg);
  extern "C" ssize_t custom_read(int fd, void* buf, size_t count);
  extern "C" ssize_t custom_write(int fd, const void* buf, size_t count);
#else
  #include <sys/ioctl.h>
#endif

class CAN
{
  private:
    std::string _canDevice;
    int _canFd;
    uint8_t _txBuffer;

  public:
    CAN();
    ~CAN();
    CAN(const CAN& originalCAN);
    CAN& operator=(const CAN& originalCAN);

    void init(const std::string& CANDevice);

    void reset();
    void setSPI();
    void setBaudRate();
    void setMasksFilters();
    void configureRxBuffers();
    void configureTxBuffers();
    void setNormalMode();

    void spiTransfer(uint8_t *tx_buffer, uint8_t *rx_buffer, size_t len);

    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readRegister(uint8_t address);

    uint8_t readMessage(uint8_t buffer, uint32_t &can_id, uint8_t *data);
    void writeMessage(uint32_t addr, uint8_t *tx, size_t length);

    int checktheReceive();
  
};