	
// #include "../include/CAN.hpp"
// #include <iomanip>

// #ifdef TEST_MODE
//   // Define custom function names for testing
//   #define open custom_open
//   #define close custom_close
//   #define ioctl custom_ioctl
//   #define read custom_read
//   #define write custom_write
// #endif

// CAN::CAN()
// {}
    
// CAN::~CAN()
// {
//     close(_canFd);
// }

// CAN::CAN(const CAN& originalCAN)
// {
//     (void) originalCAN;
// }

// CAN& CAN::operator=(const CAN& originalCAN)
// {
//     (void) originalCAN;
//     return *this;
// }

// void CAN::init(const std::string& CANDevice)
// {
//     try{
//         this->_canFd = open(CANDevice.c_str(), O_RDWR);
    
//         this->_txBuffer = 0;
    
//         this->setSPI();
//         this->reset();
//         this->setBaudRate();
//         this->setMasksFilters();
//         this->configureRxBuffers();
//         this->configureTxBuffers();
//         this->setNormalMode();
    
//         printf("Listening for CAN messages...\n");
//     }
//     catch(const std::exception& e){
//         std::cerr << e.what() << std::endl;
//     }
// }

// void CAN::setSPI()
// {
//     uint8_t mode = SPI_MODE_0;
//     uint8_t bits = 8;
//     uint32_t speed = 10000000; // 10 MHz;

//     if (ioctl(this->_canFd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(this->_canFd, SPI_IOC_RD_MODE, &mode) < 0) {
//         throw std::runtime_error("Failed to set SPI mode.");
//         close(this->_canFd);
//         return;
//     }

//     if (ioctl(this->_canFd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 || ioctl(this->_canFd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
//         throw std::runtime_error("Failed to set SPI mode.");
//         close(this->_canFd);
//         return;
//     }

//     if (ioctl(this->_canFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 || ioctl(this->_canFd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
//         throw std::runtime_error("Failed to set SPI mode.");
//         close(this->_canFd);
//         return;
//     }
// }

// void CAN::setBaudRate() {
//     // Configuring for 500 kbps with 8 MHz oscillator
//     this->writeRegister(CNF1, CAN_500Kbps);
//     this->writeRegister(CNF2, 0x91);
//     this->writeRegister(CNF3, 0x01);
//     // this->writeRegister(CNF2, 0x91);
//     // this->writeRegister(CNF3, 0x02);

//     // this->writeRegister(CNF2, 0x80|PHSEG1_3TQ|PRSEG_1TQ);
//     // this->writeRegister(CNF3, PHSEG2_3TQ);
// }

// void CAN::setMasksFilters()
// {
//     // // Set RXM0 (mask for RXB0) to only accept ID 0x01
//     // mcp_write_register(MCP_RXM0SIDH, 0xFF); // Match exact bits
//     // mcp_write_register(MCP_RXM0SIDH + 1, 0xE0); // Standard ID filter

//     // // Set RXF0 (filter for RXB0) to accept ID 0x01
//     // mcp_write_register(MCP_RXF0SIDH, 0x00); // ID High
//     // mcp_write_register(MCP_RXF0SIDH + 1, 0x08); // ID Low (0x01 << 5)
// }

// void CAN::configureRxBuffers()
// {
//     this->writeRegister(RXB0SIDH, 0x00);
// 	this->writeRegister(RXB0SIDL, 0x00);
// 	this->writeRegister(RXB0CTRL, 0x40);
// 	this->writeRegister(RXB0DLC, DLC_8);

// 	this->writeRegister(RXF0SIDH,0xFF);
// 	this->writeRegister(RXF0SIDL,0xE0);
// 	this->writeRegister(RXM0SIDH,0x00);
// 	this->writeRegister(RXM0SIDL,0x00);
// }

// void CAN::configureTxBuffers()
// {
//     this->writeRegister(TXB0SIDH, 0xFF);
// 	this->writeRegister(TXB0SIDL, 0xE0);
// 	this->writeRegister(TXB0DLC, 0x40|DLC_8);
// }

// void CAN::setNormalMode()
// {
//     this->writeRegister(CANINTF,0x00); //clean interrupt flag
// 	this->writeRegister(CANINTE,0x01); //Receive Buffer 0 Full Interrupt Enable Bit

//     // this->writeRegister(CANCTRL, REQOP_NORMAL| CLKOUT_ENABLED); // Set CANCTRL to normal mode
//     this->writeRegister(CANCTRL, 0x00); // Set CANCTRL to normal mode
//     uint8_t mode = this->readRegister(CANCTRL);
//     printf("CANCTRL Mode: 0x%02X\n", mode);
// }

// void CAN::reset() {
//     uint8_t reset_cmd = CAN_RESET;
//     this->spiTransfer(&reset_cmd, NULL, 1);
//     usleep(10000);
// }

// void CAN::spiTransfer(uint8_t *tx_buffer, uint8_t *rx_buffer, size_t len)
// {
//     struct spi_ioc_transfer tr;
//     memset(&tr, 0, sizeof(tr));  // Zero out the structure first
    
//     tr.tx_buf = (unsigned long)tx_buffer;
//     tr.rx_buf = (unsigned long)rx_buffer;
//     tr.len = (__u32)len;
//     tr.speed_hz = 10000000;
//     tr.delay_usecs = 0;
//     tr.bits_per_word = 8;
//     tr.cs_change = 0;
//     tr.tx_nbits = 0;
//     tr.rx_nbits = 0;
//     tr.pad = 0;
//     if (ioctl(this->_canFd, SPI_IOC_MESSAGE(1), &tr) < 0) {
//         throw std::runtime_error("SPI transfer failed.");
//         perror("SPI transfer failed");
//     }
// }

// void CAN::writeRegister(uint8_t address, uint8_t value) {
//     uint8_t tx_buffer[3] = {CAN_WRITE, address, value};
//     this->spiTransfer(tx_buffer, NULL, 3);
// }

// uint8_t CAN::readRegister(uint8_t address) {
//     uint8_t tx_buffer[3] = {CAN_READ, address, 0x00};
//     uint8_t rx_buffer[3] = {0};
//     this->spiTransfer(tx_buffer, rx_buffer, 3);
//     return rx_buffer[2];
// }

// // uint8_t CAN::readMessage(uint8_t buffer, uint32_t &can_id, uint8_t *data) {
// //     //(void) buffer;
// //     uint8_t address = (buffer == 0) ? RXB0SIDH : RXB1SIDH;

// //     uint8_t tx_buffer[13] = {CAN_READ, address};
// //     uint8_t rx_buffer[14] = {0};
// //     this->spiTransfer(tx_buffer, rx_buffer, 13);

// //     uint8_t sidh = this->readRegister(RXB0SIDH);
// //     uint8_t sidl = this->readRegister(RXB0SIDL);
// //     // uint8_t sidh = rx_buffer[1];
// //     // uint8_t sidl = rx_buffer[2];
// //     // printf("SIDH: 0x%02X, SIDL: 0x%02X\n", sidh, sidl);
// //     if (sidl & 0x08) { // Extended ID frame (IDE bit set)
// //         can_id = ((sidh << 3) | (sidl >> 5)) & 0x3FF;
// //         can_id = (can_id << 18) | ((sidl & 0x03) << 16) | (rx_buffer[3] << 8) | rx_buffer[4];
// //     } else { // Standard ID frame
// //         can_id = ((sidh << 3) | (sidl >> 5)) & 0x3FF;
// //     }

// //     // uint8_t data_length = this->readRegister(RXB0DLC);
// //     uint8_t data_length = rx_buffer[5];

// //     //Extract data bytes
// //     for (int i = 0; i < data_length; i++) {
// //         data[i] = rx_buffer[7 + i];
// //     }
// //     //Print the received message
// //     // if( data_length == 6 && data[0] == 6)
// //     // // {
// //         // printf("Received CAN ID: 0x%03X, Length: %d, Data: ", can_id, data_length);
// //         // for (int i = 0; i < data_length; i++) {
// //         //     printf("0x%02X ", data[i]);
// //         // }
// //         // printf("\n");
// //     // }

// //     this->writeRegister(CANINTF, 0);
// // 	this->writeRegister(CANINTE, 0x01);

// // 	this->writeRegister(RXB0SIDH,0x00);
// // 	this->writeRegister(RXB0SIDL,0x60);
// //     return (data_length);
// // }   

// uint8_t CAN::readMessage(uint8_t buffer, uint32_t &can_id, uint8_t *data) {
//     uint8_t address = (buffer == 0) ? RXB0SIDH : RXB1SIDH;

//     // Read all message data in one SPI transaction (more reliable)
//     uint8_t tx_buffer[14] = {CAN_READ, address, 0,0,0,0,0,0,0,0,0,0,0,0};
//     uint8_t rx_buffer[14] = {0};
//     this->spiTransfer(tx_buffer, rx_buffer, 14);

//     // Extract ID from the bulk read data (not separate register reads)
//     uint8_t sidh = rx_buffer[2];  // SIDH from bulk read
//     uint8_t sidl = rx_buffer[3];  // SIDL from bulk read
    
//     if (sidl & 0x08) { // Extended ID frame (IDE bit set)
//         can_id = ((sidh << 3) | (sidl >> 5)) & 0x3FF;
//         can_id = (can_id << 18) | ((sidl & 0x03) << 16) | (rx_buffer[4] << 8) | rx_buffer[5];
//     } else { // Standard ID frame
//         can_id = ((sidh << 3) | (sidl >> 5)) & 0x3FF;
//     }

//     // Get data length from bulk read
//     uint8_t data_length = rx_buffer[6] & 0x0F;  // DLC from bulk read
    
//     // Ensure valid data length
//     if (data_length > 8) data_length = 8;

//     // Extract data bytes from bulk read
//     for (int i = 0; i < 8; i++) {
//         data[i] = (i < data_length) ? rx_buffer[7 + i] : 0;
//     }

//     // Clear the receive buffer interrupt flag ONLY for the buffer we just read
//     uint8_t clearFlag = (buffer == 0) ? ~RX0IF : ~RX1IF;
//     uint8_t currentFlags = this->readRegister(CANINTF);
//     this->writeRegister(CANINTF, currentFlags & clearFlag);

//     return data_length;
// }

// void CAN::writeMessage(uint32_t addr, uint8_t *tx, size_t length)
// {
//     uint8_t tempdata = this->readRegister(CAN_RD_STATUS);
//     this->writeRegister(TXB0SIDH, (addr >>3)&0XFF);
// 	this->writeRegister(TXB0SIDL, (addr & 0x07)<<5);

// 	this->writeRegister(TXB0EID8, 0);
// 	this->writeRegister(TXB0EID0, 0);
// 	this->writeRegister(TXB0DLC, length);

// 	for (size_t j = 0; j < length; j++)
//     {
// 		this->writeRegister(TXB0D0+j, tx[j]);
//     }

// 	if(tempdata & 0x04)
// 	{
//         usleep(10000);
// 		this->writeRegister(TXB0CTRL, 0);
// 		while(1)
//         {
// 			if((this->readRegister(CAN_RD_STATUS) & 0x04) != 1)
// 				break;
//         } 
//     }
//     uint8_t rts_cmd = CAN_RTS_TXB0;
//     spiTransfer(&rts_cmd, NULL, 1);
// }



// // int CAN::checktheReceive() {
// //     uint8_t canintf = this->readRegister(CANINTF);
// //     if (canintf & RX0IF) return 0; // RXB0 has data
// //     if (canintf & RX1IF) return 1; // RXB1 has data
// //     return -1; // No data
// // }


// int CAN::checktheReceive() {
//     uint8_t canintf = this->readRegister(CANINTF);
    
//     // Check if specific receive flags are set AND verify data is actually present
//     if (canintf & RX0IF) {
//         // Verify valid data by checking DLC register
//         uint8_t dlc = this->readRegister(RXB0DLC) & 0x0F; // Get data length
//         if (dlc > 0) {
//             return 0; // RXB0 has valid data
//         }
//     }
    
//     if (canintf & RX1IF) {
//         // Similar check for RXB1
//         uint8_t dlc = this->readRegister(RXB1DLC) & 0x0F;
//         if (dlc > 0) {
//             return 1; // RXB1 has valid data
//         }
//     }
    
//     // Clear spurious interrupt flags
//     this->writeRegister(CANINTF, 0);
    
//     return -1; // No valid data
// }


// bool CAN::waitForMessage(int timeout_ms) {
//     const int POLL_INTERVAL_US = 100; // 100 microseconds
//     int totalWaitTime = 0;
    
//     while (true) {
//         // Check if message is available
//         uint8_t canintf = this->readRegister(CANINTF);
//         if (canintf & (RX0IF | RX1IF)) {
//             return true; // Message available
//         }
        
//         // If timeout is specified and we've exceeded it, return false
//         if (timeout_ms >= 0 && totalWaitTime >= (timeout_ms * 1000)) {
//             return false;
//         }
        
//         // Brief sleep to avoid hammering the SPI bus
//         usleep(POLL_INTERVAL_US);
//         totalWaitTime += POLL_INTERVAL_US;
//     }
// }

#include "../include/CAN.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sched.h>
#include <poll.h>
#include <cstring>
#include <thread>
#include <chrono>
#include <cstdio>
#include <cstddef> 


// MCP2515 SPI instructions
static constexpr uint8_t INSTR_RESET      = 0xC0;
static constexpr uint8_t INSTR_WRITE      = 0x02;
static constexpr uint8_t INSTR_BIT_MODIFY = 0x05;
static constexpr uint8_t INSTR_READ_RX    = 0x90;  // + buffer select
static constexpr uint8_t INSTR_LOAD_TX    = 0x40;  // + buffer select

// Default ctor picks default GPIO pins and bitrate
CAN::CAN()
 : spiDev_(nullptr)
 , rstGpio_(17)      // RESET → header P17 (BCM17, pin 11)
 , intGpio_(5)       // INT   → header pin 29 (BCM5)
 , bitrate_(500000)  // default 500 kbps
 , spiFd_(-1)
 , intFd_(-1)
{
    memset(xfer_, 0, sizeof(xfer_));
}

CAN::~CAN() {
    shutdown();
}

bool CAN::exportGpio(int gpio) {
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return false;
    char buf[8]; int n = snprintf(buf, sizeof(buf), "%d", gpio);
    write(fd, buf, n);
    close(fd);
    return true;
}

bool CAN::setGpioDirection(int gpio, bool input) {
    char path[64];
    snprintf(path, sizeof(path),
             "/sys/class/gpio/gpio%d/direction", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return false;
    write(fd, input ? "in" : "out", input ? 2 : 3);
    close(fd);
    return true;
}

int CAN::openGpioValueFd(int gpio) {
    char path[64];
    snprintf(path, sizeof(path),
             "/sys/class/gpio/gpio%d/value", gpio);
    return open(path, O_RDONLY | O_NONBLOCK);
}

bool CAN::init(const char* spiDev) {
    spiDev_ = spiDev;  // record path

    // 1) Real-time + mlock
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall");
        return false;
    }
    sched_param sp{ .sched_priority = 80 };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("sched_setscheduler");
        return false;
    }
    cpu_set_t cpus; CPU_ZERO(&cpus); CPU_SET(0, &cpus);
    sched_setaffinity(0, sizeof(cpus), &cpus);

    // 2) Open/config SPI
    spiFd_ = open(spiDev_, O_RDWR);
    if (spiFd_ < 0) { perror("open spidev"); return false; }
    uint8_t mode = SPI_MODE_0;
    ioctl(spiFd_, SPI_IOC_WR_MODE, &mode);
    uint32_t speed = 10000000;
    ioctl(spiFd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    // 3) Setup GPIOs
    exportGpio(rstGpio_);
    setGpioDirection(rstGpio_, false);  // RESET = output
    exportGpio(intGpio_);
    setGpioDirection(intGpio_, true);   // INT   = input
    intFd_ = openGpioValueFd(intGpio_);
    if (intFd_ < 0) { perror("open int gpio"); return false; }

    // 4) Reset & bit-timing
    if (!resetChip())       return false;
    if (!configBitTiming()) return false;

    return true;
}

bool CAN::resetChip() {
    uint8_t cmd = INSTR_RESET;
    return spiXfer(&cmd, nullptr, 1);
}

bool CAN::configBitTiming() {
    // Enter config mode
    if (!modifyReg(0x0F, 0xE0, 0x80)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // CNF1 @ 0x2A, CNF2 @ 0x29, CNF3 @ 0x28
    if (!writeReg(0x2A, 0x00)) return false;
    if (!writeReg(0x29, 0x90)) return false;
    if (!writeReg(0x28, 0x02)) return false;
    // Normal mode
    if (!modifyReg(0x0F, 0xE0, 0x00)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return true;
}

bool CAN::writeReg(uint8_t addr, uint8_t val) {
    uint8_t buf[3] = { INSTR_WRITE, addr, val };
    return spiXfer(buf, nullptr, 3);
}

bool CAN::modifyReg(uint8_t addr, uint8_t mask, uint8_t data) {
    uint8_t buf[4] = { INSTR_BIT_MODIFY, addr, mask, data };
    return spiXfer(buf, nullptr, 4);
}

bool CAN::spiXfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    struct spi_ioc_transfer tr = {};
    tr.tx_buf      = (unsigned long)tx;
    tr.rx_buf      = (unsigned long)rx;
    tr.len         = len;
    tr.speed_hz    = 10000000;
    tr.delay_usecs = 0;
    return ioctl(spiFd_, SPI_IOC_MESSAGE(1), &tr) == 1;
}

bool CAN::spiBatch(const uint8_t* cmd,
                   const uint8_t* tx2, uint8_t* rx2, size_t len2)
{
    // phase 1: instr
    xfer_[0] = {};
    xfer_[0].tx_buf   = (unsigned long)cmd;
    xfer_[0].len      = 1;
    // phase 2: data
    xfer_[1] = {};
    xfer_[1].tx_buf   = (unsigned long)(tx2 ? tx2 : cmd);
    xfer_[1].rx_buf   = (unsigned long)rx2;
    xfer_[1].len      = len2;
    xfer_[0].speed_hz = xfer_[1].speed_hz = 10000000;
    return ioctl(spiFd_, SPI_IOC_MESSAGE(2), xfer_) == 2;
}

bool CAN::send(const CANFrame& txf) {
    uint8_t buf[1 + 2 + 1 + 8];
    buf[0] = INSTR_LOAD_TX | 0x00;
    buf[1] = (txf.id >> 3) & 0xFF;
    buf[2] = (txf.id << 5) & 0xE0;
    buf[3] = txf.dlc & 0x0F;
    memcpy(&buf[4], txf.data, 8);
    return spiBatch(buf, &buf[1], nullptr, sizeof(buf) - 1);
}

bool CAN::recv(CANFrame& rxf) {
    pollfd pfd{ .fd = intFd_, .events = POLLPRI, .revents = 0 };
    lseek(intFd_, 0, SEEK_SET);
    read(intFd_, nullptr, 1);
    if (poll(&pfd, 1, -1) <= 0) return false;

    uint8_t cmd = INSTR_READ_RX | 0x00;
    uint8_t rx[14] = {};
    if (!spiBatch(&cmd, nullptr, rx, sizeof(rx))) return false;

    rxf.id  = (rx[1] << 3) | (rx[2] >> 5);
    rxf.dlc = rx[5] & 0x0F;
    memcpy(rxf.data, &rx[6], 8);
    return true;
}

void CAN::shutdown() {
    if (spiFd_ >= 0) close(spiFd_);
    if (intFd_ >= 0) close(intFd_);
}

// -----------------------------------------------------------------
// Methods exposed to Signals::run()
int CAN::checktheReceive() {
    pollfd pfd{ .fd = intFd_, .events = POLLPRI, .revents = 0 };
    lseek(intFd_, 0, SEEK_SET);
    read(intFd_, nullptr, 1);
    return (poll(&pfd, 1, 0) > 0) ? 0 : -1;
}

bool CAN::readMessage(int /*buf*/,
                      uint32_t &can_id,
                      uint8_t data[8])
{
    CANFrame f;
    if (!recv(f)) return false;
    can_id = f.id;
    memcpy(data, f.data, 8);
    return true;
}


void CAN::writeMessage(uint32_t can_id, uint8_t* data, size_t len) {
    // Ensure valid data length
    if (len > 8) len = 8;
    
    CANFrame frame;
    frame.id = can_id;
    frame.dlc = len;
    memcpy(frame.data, data, len);
    
    // Zero out unused data bytes
    if (len < 8) {
        memset(&frame.data[len], 0, 8 - len);
    }
    
    send(frame);
}