// #pragma once
// #include <iostream>
// #include <cstring>

// // CAN-SPI register values and defines
// #include "MCP_DEFS.hpp"
// #include <cstdint>
// #include <fcntl.h>
// #include <unistd.h>
// #include <linux/spi/spidev.h>

// #ifdef TEST_MODE
//   // Declare your custom functions
//   extern "C" int custom_open(const char* path, int flags);
//   extern "C" int custom_close(int fd);
//   extern "C" int custom_ioctl(int fd, unsigned long request, void* arg);
//   extern "C" ssize_t custom_read(int fd, void* buf, size_t count);
//   extern "C" ssize_t custom_write(int fd, const void* buf, size_t count);
// #else
//   #include <sys/ioctl.h>
// #endif

// class CAN
// {
//   private:
//     std::string _canDevice;
//     int _canFd;
//     uint8_t _txBuffer;

//   public:
//     CAN();
//     ~CAN();
//     CAN(const CAN& originalCAN);
//     CAN& operator=(const CAN& originalCAN);

//     void init(const std::string& CANDevice);

//     void reset();
//     void setSPI();
//     void setBaudRate();
//     void setMasksFilters();
//     void configureRxBuffers();
//     void configureTxBuffers();
//     void setNormalMode();

//     void spiTransfer(uint8_t *tx_buffer, uint8_t *rx_buffer, size_t len);

//     void writeRegister(uint8_t address, uint8_t value);
//     uint8_t readRegister(uint8_t address);

//     uint8_t readMessage(uint8_t buffer, uint32_t &can_id, uint8_t *data);
//     void writeMessage(uint32_t addr, uint8_t *tx, size_t length);

//     bool waitForMessage(int timeout_ms = -1); 

//     int checktheReceive();
  
// };

#ifndef CAN_HPP
#define CAN_HPP

#include <cstdint>
#include <linux/spi/spidev.h>
#include <cstddef> 
#include <string> 

struct CANFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};

class CAN {
public:
    CAN();                            // default ctor: picks sane GPIO & bitrate
    ~CAN();

    // Initialize with SPI device path (e.g. "/dev/spidev2.0")
    // Returns true on success, false on any setup failure.
    bool init(const char* spiDev);
    bool init(const std::string& spiDev) { return init(spiDev.c_str()); } 

    // Send a CAN frame (blocking on the SPI transfer)
    bool send(const CANFrame& txf);

    // Non-blocking check: returns 0 if a frame is pending, -1 otherwise
    int  checktheReceive();

    // Read the pending frame into can_id + data[8]
    // 'buf' is ignored (we always use RX buffer 0)
    bool readMessage(int buf, uint32_t &can_id, uint8_t data[8]);

    // Write a CAN message with specified ID, data, and length
    void writeMessage(uint32_t can_id, uint8_t* data, size_t len);

    // Close fds
    void shutdown();

private:
    // Internal blocking read
    bool recv(CANFrame &rxf);

    // Device & config
    const char* spiDev_;
    int         rstGpio_;
    int         intGpio_;
    uint32_t    bitrate_;

    int spiFd_;
    int intFd_;
    struct spi_ioc_transfer xfer_[2];

    // GPIO helpers
    bool exportGpio(int gpio);
    bool setGpioDirection(int gpio, bool input);
    int  openGpioValueFd(int gpio);

    // MCP2515 setup
    bool resetChip();
    bool configBitTiming();
    bool writeReg(uint8_t addr, uint8_t val);
    bool modifyReg(uint8_t addr, uint8_t mask, uint8_t data);

    // SPI transfers
    bool spiXfer(const uint8_t* tx, uint8_t* rx, size_t len);
    bool spiBatch(const uint8_t* cmd,
                  const uint8_t* tx2, uint8_t* rx2, size_t len2);
};

#endif // CAN_HPP
