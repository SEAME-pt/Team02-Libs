
/**
 * @file CAN.cpp
 * @brief Implementation file for the CAN class, managing communication with the MCP2515 CAN controller.
 * 
 * This file contains the implementation of the CAN class, which provides an interface for configuring and operating
 * the MCP2515 CAN controller using SPI communication. The class handles tasks such as initialization, configuration,
 * message transmission, and reception.
 * 
 * Features include:
 * - Initialization of SPI and CAN controller settings.
 * - Configuring baud rate, RX/TX buffers, and normal operation mode.
 * - Reading and writing CAN messages.
 * - Handling interrupts and checking data availability.
 */


#include "../include/CAN.hpp"
#include <iomanip>

/**
 * @brief Default constructor for the CAN class.
 * Initializes a CAN object with default values.
 */
CAN::CAN()
{}

/**
 * @brief Destructor for the CAN class.
 * Closes the CAN file descriptor if it is open.
 */
CAN::~CAN()
{
    close(_canFd);
}

/**
 * @brief Copy constructor for the CAN class.
 * Currently performs no operations as copying is not implemented.
 *
 * @param originalCAN Reference to the original CAN object to copy.
 */
CAN::CAN(const CAN& originalCAN)
{
    (void) originalCAN;
}

/**
 * @brief Copy assignment operator for the CAN class.
 * Currently performs no operations as copying is not implemented.
 *
 * @param originalCAN Reference to the original CAN object to assign from.
 * @return Reference to the assigned CAN object.
 */
CAN& CAN::operator=(const CAN& originalCAN)
{
    (void) originalCAN;
    return *this;
}

/**
 * @brief Initializes the CAN device.
 * Configures SPI settings, resets the device, sets baud rate, masks, and filters,
 * configures RX and TX buffers, and sets the device to normal mode.
 *
 * @param CANDevice Path to the CAN device (e.g., "/dev/spidev0.0").
 */
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

/**
 * @brief Configures SPI settings for the CAN device.
 * Sets SPI mode, bits per word, and maximum speed.
 */
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

/**
 * @brief Configures the CAN device baud rate.
 * Sets the configuration registers for a baud rate of 500 kbps with an 8 MHz oscillator.
 */
void CAN::setBaudRate() {
    this->writeRegister(CNF1, CAN_500Kbps);
    this->writeRegister(CNF2, 0x80|PHSEG1_3TQ|PRSEG_1TQ);
    this->writeRegister(CNF3, PHSEG2_3TQ);
}

/**
 * @brief Configures masks and filters for the CAN device.
 * Currently commented out, intended for setting RX masks and filters.
 */
void CAN::setMasksFilters()
{
    // Example of mask and filter settings:
    // Set RXM0 (mask for RXB0) to only accept ID 0x01
    // this->writeRegister(MCP_RXM0SIDH, 0xFF);
    // this->writeRegister(MCP_RXM0SIDH + 1, 0xE0);
}

/**
 * @brief Configures RX buffers for the CAN device.
 * Sets up RXB0 and RX filters and masks.
 */
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

/**
 * @brief Configures TX buffers for the CAN device.
 * Sets up TXB0 with appropriate settings.
 */
void CAN::configureTxBuffers()
{
    this->writeRegister(TXB0SIDH, 0xFF);
	this->writeRegister(TXB0SIDL, 0xE0);
	this->writeRegister(TXB0DLC, 0x40|DLC_8);
}

/**
 * @brief Sets the CAN device to normal mode.
 * Configures CAN interrupt flags and enables normal operation.
 */
void CAN::setNormalMode()
{
    this->writeRegister(CANINTF,0x00); // Clean interrupt flag
	this->writeRegister(CANINTE,0x01); // Enable RXB0 full interrupt

    this->writeRegister(CANCTRL, REQOP_NORMAL| CLKOUT_ENABLED); // Set CANCTRL to normal mode
    uint8_t mode = this->readRegister(CANCTRL);
    printf("CANCTRL Mode: 0x%02X\n", mode);
}

/**
 * @brief Resets the CAN device.
 * Sends a reset command over SPI and waits for the device to reset.
 */
void CAN::reset() {
    uint8_t reset_cmd = CAN_RESET;
    this->spiTransfer(&reset_cmd, NULL, 1);
    usleep(10000);
}

/**
 * @brief Transfers data to/from the CAN device over SPI.
 *
 * @param tx_buffer Pointer to the buffer containing data to send.
 * @param rx_buffer Pointer to the buffer to store received data.
 * @param len Length of the data to transfer.
 */
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
        .word_delay_usecs = 0,
        .pad = 0, 
    };
    if (ioctl(this->_canFd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("SPI transfer failed");
    }
}

/**
 * @brief Writes a value to a register on the CAN device.
 *
 * @param address Address of the register to write to.
 * @param value Value to write to the register.
 */
void CAN::writeRegister(uint8_t address, uint8_t value) {
    uint8_t tx_buffer[3] = {CAN_WRITE, address, value};
    this->spiTransfer(tx_buffer, NULL, 3);
}

/**
 * @brief Reads a value from a register on the CAN device.
 *
 * @param address Address of the register to read from.
 * @return Value read from the register.
 */
uint8_t CAN::readRegister(uint8_t address) {
    uint8_t tx_buffer[3] = {CAN_READ, address, 0x00};
    uint8_t rx_buffer[3] = {0};
    this->spiTransfer(tx_buffer, rx_buffer, 3);
    return rx_buffer[2];
}

/**
 * @brief Reads a message from the specified RX buffer.
 *
 * @param buffer RX buffer to read from (0 or 1).
 */
void CAN::readMessage(uint8_t buffer) {
    uint8_t address = (buffer == 0) ? RXB0SIDH : RXB1SIDH;

    uint8_t tx_buffer[13] = {CAN_READ, address};
    uint8_t rx_buffer[40] = {0};
    this->spiTransfer(tx_buffer, rx_buffer, 13);

    uint8_t sidh = this->readRegister(RXB0SIDH);
    uint8_t sidl = this->readRegister(RXB0SIDL);

    uint32_t can_id;
    if (sidl & 0x08) { // Extended ID frame (IDE bit set)
        can_id = ((sidh << 3) | (sidl >> 5)) & 0x7FF;
        can_id = (can_id << 18) | ((sidl & 0x03) << 16) | (rx_buffer[3] << 8) | rx_buffer[4];
    } else { // Standard ID frame
        can_id = (sidh << 3) | (sidl >> 5);
    }

    uint8_t data_length = this->readRegister(RXB0DLC);

    // Extract data bytes
    uint8_t data[8];
    for (int i = 0; i < data_length; i++) {
        data[i] = rx_buffer[6 + i];
    }

    // Print the received message
    if( data_length == 6 && data[0] == 6)
    {
        printf("Received CAN ID: 0x%03X, Length: %d, Data: ", can_id, data_length);
        for (int i = 0; i < data_length; i++) {
            printf("0x%02X ", data[i]);
        }
        printf("\n");
    }

    this->writeRegister(CANINTF, 0);
	this->writeRegister(CANINTE,0x01);
	this->writeRegister(RXB0SIDH,0x00);
	this->writeRegister(RXB0SIDL,0x60);
}

/**
 * @brief Writes a message to the CAN device.
 *
 * @param addr Address to write the message to.
 * @param tx Pointer to the buffer containing the message data.
 * @param length Length of the message data.
 */
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

/**
 * @brief Checks if there is data available to receive.
 *
 * @return 0 if RXB0 has data, 1 if RXB1 has data, -1 if no data is available.
 */
int CAN::checkReceive() {
    uint8_t canintf = this->readRegister(CANINTF);
    if (canintf & RX0IF) return 0; // RXB0 has data
    if (canintf & RX1IF) return 1; // RXB1 has data
    return -1; // No data
}
