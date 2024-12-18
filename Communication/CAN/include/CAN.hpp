#pragma once
#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

/********************************************************/
/*                  CAN SPI commands                    */
/********************************************************/

#define CAN_RESET      0xC0
#define CAN_READ       0x03
#define CAN_WRITE      0x02
#define CAN_RTS        0x80
#define CAN_RTS_TXB0   0x81
#define CAN_RTS_TXB1   0x82
#define CAN_RTS_TXB2   0x84
#define CAN_RD_STATUS  0xA0
#define CAN_BIT_MODIFY 0x05  
#define CAN_RX_STATUS  0xB0
#define CAN_RD_RX_BUFF 0x90
#define CAN_LOAD_TX    0x40  



#define SPI_DEVICE "/dev/spidev0.0"
#define SPI_SPEED 500000             // 500 kHz
#define SPI_BITS_PER_WORD 8          // 8 bits per word
#define SPI_MODE SPI_MODE_0          // SPI Mode 0 (CPOL = 0, CPHA = 0)

class CAN
{
  private:
    std::string _canDevice;
    int _canFd;

  public:
    CAN();
    ~CAN();
    CAN(const CAN& originalCAN);
    CAN& operator=(const CAN& originalCAN);

    void init(const std::string& CANDevice);
    void writeByte(uint8_t deviceAddress, uint8_t reg, uint8_t value);
    uint8_t readByte(uint8_t deviceAddress);
};