#pragma once
#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

//CAN-SPI register values and defines
#include "MCP_DEFS.hpp"


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

    uint8_t *readMessage(uint8_t buffer, uint32_t &can_id, uint8_t *data);
    void writeMessage(uint32_t addr, uint8_t *tx, size_t length);

    int checkReceive();
  
};