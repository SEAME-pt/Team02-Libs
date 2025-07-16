	
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
#include <iomanip>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <arpa/inet.h>

#ifdef TEST_MODE
  // Define custom function names for testing
  #define open custom_open
  #define close custom_close
  #define ioctl custom_ioctl
  #define read custom_read
  #define write custom_write
#endif

CAN::CAN()
{}
    
CAN::~CAN()
{
    if (_canFd >= 0) {
        close(_canFd);
    }
}

CAN::CAN(const CAN& originalCAN)
{
    (void) originalCAN;
}

CAN& CAN::operator=(const CAN& originalCAN)
{
    (void) originalCAN;
    return *this;
}

void CAN::init(const std::string& CANDevice)
{
    try {
        struct ifreq ifr;
        struct sockaddr_can addr;

        // Create CAN socket
        this->_canFd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (this->_canFd < 0) {
            throw std::runtime_error("Cannot create CAN socket!");
        }

        // Set up interface
        strcpy(ifr.ifr_name, CANDevice.c_str());
        if (ioctl(this->_canFd, SIOCGIFINDEX, &ifr) < 0) {
            close(this->_canFd);
            throw std::runtime_error("Cannot get interface index for " + CANDevice);
        }

        // Bind socket to CAN interface
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(this->_canFd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(this->_canFd);
            throw std::runtime_error("Cannot bind CAN socket to " + CANDevice);
        }

        printf("CAN socket bound to %s interface successfully.\n", CANDevice.c_str());
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << std::endl;
        throw;
    }
}

uint8_t CAN::readMessage(uint8_t buffer, uint32_t &can_id, uint8_t *data) {
    (void) buffer; // Buffer parameter not used in socket CAN
    
    struct can_frame frame;
    int nbytes = read(this->_canFd, &frame, sizeof(struct can_frame));
    
    if (nbytes < 0) {
        return 0; // Error reading
    }
    
    if (static_cast<unsigned int> nbytes < sizeof(struct can_frame)) {
        return 0; // Incomplete frame
    }
    
    // Extract CAN ID and data
    can_id = frame.can_id & CAN_SFF_MASK; // Standard frame ID
    
    // Copy data and return length
    uint8_t data_length = frame.can_dlc;
    if (data_length > 8) data_length = 8; // Ensure valid length
    
    for (int i = 0; i < 8; i++) {
        data[i] = (i < data_length) ? frame.data[i] : 0;
    }
    
    return data_length;
}

void CAN::writeMessage(uint32_t addr, uint8_t *tx, size_t length)
{
    struct can_frame frame;
    
    // Set up CAN frame
    frame.can_id = addr & CAN_SFF_MASK; // Standard frame ID
    frame.can_dlc = (length > 8) ? 8 : length; // Ensure valid length
    
    // Copy data
    for (size_t i = 0; i < frame.can_dlc; i++) {
        frame.data[i] = tx[i];
    }
    
    // Clear unused bytes
    for (size_t i = frame.can_dlc; i < 8; i++) {
        frame.data[i] = 0;
    }
    
    // Send frame
    int nbytes = write(this->_canFd, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        throw std::runtime_error("Failed to send CAN message");
    }
}

int CAN::checktheReceive() {
    fd_set readfs;
    struct timeval timeout;
    
    FD_ZERO(&readfs);
    FD_SET(this->_canFd, &readfs);
    
    // Non-blocking check
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    
    int result = select(this->_canFd + 1, &readfs, NULL, NULL, &timeout);
    
    if (result > 0 && FD_ISSET(this->_canFd, &readfs)) {
        return 0; // Data available (using 0 for consistency with original)
    }
    
    return -1; // No data available
}

bool CAN::waitForMessage(int timeout_ms) {
    fd_set readfs;
    struct timeval timeout;
    
    FD_ZERO(&readfs);
    FD_SET(this->_canFd, &readfs);
    
    if (timeout_ms >= 0) {
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;
    }
    
    int result = select(this->_canFd + 1, &readfs, NULL, NULL, 
                       (timeout_ms >= 0) ? &timeout : NULL);
    
    return (result > 0 && FD_ISSET(this->_canFd, &readfs));
}