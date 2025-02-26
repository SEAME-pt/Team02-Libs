	
#include "../include/CAN.hpp"
#include <iomanip>

CAN::CAN()
{}
    
CAN::~CAN()
{
    close(_canFd);
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
    this->_canFd = open(CANDevice.c_str(), O_RDWR);

    this->_txBuffer = 0;

    this->setSPI();
    this->reset();
    this->setBaudRate();
    this->setMasksFilters();
    this->configureRxBuffers();
    this->configureTxBuffers();
    this->setNormalMode();

    printf("Listening for CAN messages...\n");
}

void CAN::setSPI()
{
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 10000000; // 10 MHz;

    if (ioctl(this->_canFd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(this->_canFd, SPI_IOC_RD_MODE, &mode) < 0) {
        std::cerr << "Failed to set SPI mode." << std::endl;
        close(this->_canFd);
        return;
    }

    if (ioctl(this->_canFd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 || ioctl(this->_canFd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
        std::cerr << "Failed to set bits per word." << std::endl;
        close(this->_canFd);
        return;
    }

    if (ioctl(this->_canFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 || ioctl(this->_canFd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
        std::cerr << "Failed to set max speed." << std::endl;
        close(this->_canFd);
        return;
    }
}

void CAN::setBaudRate() {
    // Configuring for 500 kbps with 8 MHz oscillator
    this->writeRegister(CNF1, CAN_500Kbps);
    this->writeRegister(CNF2, 0x91);
    this->writeRegister(CNF3, 0x01);
    // this->writeRegister(CNF2, 0x80|PHSEG1_3TQ|PRSEG_1TQ);
    // this->writeRegister(CNF3, PHSEG2_3TQ);
}

void CAN::setMasksFilters()
{
    // // Set RXM0 (mask for RXB0) to only accept ID 0x01
    // mcp_write_register(MCP_RXM0SIDH, 0xFF); // Match exact bits
    // mcp_write_register(MCP_RXM0SIDH + 1, 0xE0); // Standard ID filter

    // // Set RXF0 (filter for RXB0) to accept ID 0x01
    // mcp_write_register(MCP_RXF0SIDH, 0x00); // ID High
    // mcp_write_register(MCP_RXF0SIDH + 1, 0x08); // ID Low (0x01 << 5)
}

void CAN::configureRxBuffers()
{
    this->writeRegister(RXB0SIDH, 0x00);
	this->writeRegister(RXB0SIDL, 0x60);
	this->writeRegister(RXB0CTRL, 0x60);
	this->writeRegister(RXB0DLC, DLC_8);

	this->writeRegister(RXF0SIDH,0xFF);
	this->writeRegister(RXF0SIDL,0xE0);
	this->writeRegister(RXM0SIDH,0xFF);
	this->writeRegister(RXM0SIDL,0xE0);
}

void CAN::configureTxBuffers()
{
    this->writeRegister(TXB0SIDH, 0xFF);
	this->writeRegister(TXB0SIDL, 0xE0);
	this->writeRegister(TXB0DLC, 0x40|DLC_8);
}

void CAN::setNormalMode()
{
    this->writeRegister(CANINTF,0x00); //clean interrupt flag
	this->writeRegister(CANINTE,0x01); //Receive Buffer 0 Full Interrupt Enable Bit

    this->writeRegister(CANCTRL, REQOP_NORMAL| CLKOUT_ENABLED); // Set CANCTRL to normal mode
    uint8_t mode = this->readRegister(CANCTRL);
    printf("CANCTRL Mode: 0x%02X\n", mode);
}

void CAN::reset() {
    uint8_t reset_cmd = CAN_RESET;
    this->spiTransfer(&reset_cmd, NULL, 1);
    usleep(10000);
}

void CAN::spiTransfer(uint8_t *tx_buffer, uint8_t *rx_buffer, size_t len)
{
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buffer,
        .rx_buf = (unsigned long)rx_buffer,
        .len = (__u32)len,        
        .speed_hz = 10000000,     
        .delay_usecs = 0,         
        .bits_per_word = 8,     
        .cs_change = 0,   
        .tx_nbits = 0,  
        .rx_nbits = 0, 
        .pad = 0, 
    };
    if (ioctl(this->_canFd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("SPI transfer failed");
    }
}

void CAN::writeRegister(uint8_t address, uint8_t value) {
    uint8_t tx_buffer[3] = {CAN_WRITE, address, value};
    this->spiTransfer(tx_buffer, NULL, 3);
}

uint8_t CAN::readRegister(uint8_t address) {
    uint8_t tx_buffer[3] = {CAN_READ, address, 0x00};
    uint8_t rx_buffer[3] = {0};
    this->spiTransfer(tx_buffer, rx_buffer, 3);
    return rx_buffer[2];
}

uint8_t CAN::readMessage(uint8_t buffer, uint32_t &can_id, uint8_t *data) {
    //(void) buffer;
    uint8_t address = (buffer == 0) ? RXB0SIDH : RXB1SIDH;

    uint8_t tx_buffer[13] = {CAN_READ, address};
    uint8_t rx_buffer[13] = {0};
    this->spiTransfer(tx_buffer, rx_buffer, 200);

    uint8_t sidh = this->readRegister(RXB0SIDH);
    uint8_t sidl = this->readRegister(RXB0SIDL);
    // uint8_t sidh = rx_buffer[1];
    // uint8_t sidl = rx_buffer[2];
    // printf("SIDH: 0x%02X, SIDL: 0x%02X\n", sidh, sidl);
    if (sidl & 0x08) { // Extended ID frame (IDE bit set)
        can_id = ((sidh << 3) | (sidl >> 5)) & 0x3FF;
        can_id = (can_id << 18) | ((sidl & 0x03) << 16) | (rx_buffer[3] << 8) | rx_buffer[4];
    } else { // Standard ID frame
        can_id = ((sidh << 3) | (sidl >> 5)) & 0x3FF;
    }

    // uint8_t data_length = this->readRegister(RXB0DLC);
    uint8_t data_length = rx_buffer[5];

    //Extract data bytes
    for (int i = 0; i < data_length; i++) {
        data[i] = rx_buffer[6 + i];
    }
    //Print the received message
    // if( data_length == 6 && data[0] == 6)
    // // {
        printf("Received CAN ID: 0x%03X, Length: %d, Data: ", can_id, data_length);
        for (int i = 0; i < data_length; i++) {
            printf("0x%02X ", data[i]);
        }
        printf("\n");
    // }

    this->writeRegister(CANINTF, 0);
	this->writeRegister(CANINTE, 0x01);

	this->writeRegister(RXB0SIDH,0x00);
	this->writeRegister(RXB0SIDL,0x60);
    return (data_length);
}   

void CAN::writeMessage(uint32_t addr, uint8_t *tx, size_t length)
{
    uint8_t tempdata = this->readRegister(CAN_RD_STATUS);
    this->writeRegister(TXB0SIDH, (addr >>3)&0XFF);
	this->writeRegister(TXB0SIDL, (addr & 0x07)<<5);

	this->writeRegister(TXB0EID8, 0);
	this->writeRegister(TXB0EID0, 0);
	this->writeRegister(TXB0DLC, length);

	for (size_t j = 0; j < length; j++)
    {
		this->writeRegister(TXB0D0+j, tx[j]);
    }

	if(tempdata & 0x04)
	{
        usleep(10000);
		this->writeRegister(TXB0CTRL, 0);
		while(1)
        {
			if((this->readRegister(CAN_RD_STATUS) & 0x04) != 1)
				break;
        } 
    }
    uint8_t rts_cmd = CAN_RTS_TXB0;
    spiTransfer(&rts_cmd, NULL, 1);
}



int CAN::checktheReceive() {
    uint8_t canintf = this->readRegister(CANINTF);
    if (canintf & RX0IF) return 0; // RXB0 has data
    if (canintf & RX1IF) return 1; // RXB1 has data
    return -1; // No data
}



