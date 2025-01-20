/**
 * @file CAN.hpp
 * @brief Header file for the CAN class, managing MCP2515 CAN controller over SPI.
 * 
 * This class provides an interface for initializing and operating the MCP2515 CAN
 * controller, including configuration, message transmission, and reception.
 */

#pragma once

#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

// CAN-SPI register values and defines
#include "MCP_DEFS.hpp"

/**
 * @class CAN
 * @brief Manages MCP2515 CAN controller communication via SPI.
 */
class CAN
{
private:
    std::string _canDevice; /**< The SPI device path for the CAN controller (e.g., /dev/spidev0.0). */
    int _canFd;             /**< File descriptor for the SPI device. */
    uint8_t _txBuffer;      /**< Transmit buffer for sending data. */

public:
    /**
     * @brief Default constructor for the CAN class.
     */
    CAN();

    /**
     * @brief Destructor for the CAN class.
     */
    ~CAN();

    /**
     * @brief Copy constructor for the CAN class.
     * @param originalCAN The original CAN object to copy.
     */
    CAN(const CAN& originalCAN);

    /**
     * @brief Assignment operator for the CAN class.
     * @param originalCAN The original CAN object to assign.
     * @return Reference to the assigned CAN object.
     */
    CAN& operator=(const CAN& originalCAN);

    /**
     * @brief Initializes the CAN controller.
     * @param CANDevice The SPI device path (e.g., /dev/spidev0.0).
     */
    void init(const std::string& CANDevice);

    /**
     * @brief Resets the MCP2515 CAN controller.
     */
    void reset();

    /**
     * @brief Configures the SPI interface for the MCP2515.
     */
    void setSPI();

    /**
     * @brief Sets the baud rate for the CAN communication.
     */
    void setBaudRate();

    /**
     * @brief Configures the receive masks and filters.
     */
    void setMasksFilters();

    /**
     * @brief Configures the MCP2515 receive buffers.
     */
    void configureRxBuffers();

    /**
     * @brief Configures the MCP2515 transmit buffers.
     */
    void configureTxBuffers();

    /**
     * @brief Sets the MCP2515 to normal operation mode.
     */
    void setNormalMode();

    /**
     * @brief Transfers data to/from the MCP2515 via SPI.
     * @param tx_buffer Pointer to the transmit buffer.
     * @param rx_buffer Pointer to the receive buffer.
     * @param len Length of the data to transfer.
     */
    void spiTransfer(uint8_t *tx_buffer, uint8_t *rx_buffer, size_t len);

    /**
     * @brief Writes a value to a specific MCP2515 register.
     * @param address Address of the register to write to.
     * @param value Value to write to the register.
     */
    void writeRegister(uint8_t address, uint8_t value);

    /**
     * @brief Reads a value from a specific MCP2515 register.
     * @param address Address of the register to read from.
     * @return The value read from the register.
     */
    uint8_t readRegister(uint8_t address);

    /**
     * @brief Reads a message from a specific receive buffer.
     * @param buffer The receive buffer to read from.
     */
    void readMessage(uint8_t buffer);

    /**
     * @brief Writes a message to the specified address.
     * @param addr Address of the target CAN ID.
     * @param tx Pointer to the data to transmit.
     * @param length Length of the data to transmit.
     */
    void writeMessage(uint32_t addr, uint8_t *tx, size_t length);

    /**
     * @brief Checks if a new message has been received.
     * @return Non-zero if a message is received; 0 otherwise.
     */
    int checkReceive();
};
