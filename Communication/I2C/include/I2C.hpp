/**
 * @file I2C.hpp
 * @brief Header file for the I2C class, providing an interface for I2C communication.
 * 
 * This class handles initializing the I2C bus, writing and reading data to/from
 * specific devices and registers, and managing the associated file descriptor.
 * It is designed to be used on Linux-based systems with I2C support.
 */


#pragma once

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cstdint>

/**
 * @class I2C
 * @brief A class for handling I2C communication on Linux-based systems.
 */
class I2C
{
  private:
    /**
     * @brief The I2C device file path.
     */
    std::string _i2cDevice;

    /**
     * @brief File descriptor for the I2C device.
     */
    int _i2cFd;

  public:
    /**
     * @brief Default constructor for the I2C class.
     */
    I2C();

    /**
     * @brief Destructor for the I2C class. Closes the I2C file descriptor.
     */
    ~I2C();

    /**
     * @brief Copy constructor for the I2C class.
     * @param originalI2C The original I2C object to copy.
     */
    I2C(const I2C& originalI2C);

    /**
     * @brief Copy assignment operator for the I2C class.
     * @param originalI2C The original I2C object to assign from.
     * @return Reference to the assigned I2C object.
     */
    I2C& operator=(const I2C& originalI2C);

    /**
     * @brief Retrieves the file descriptor for the I2C device.
     * @return The file descriptor of the I2C device.
     */
    int getFd();

    /**
     * @brief Initializes the I2C device.
     * @param i2cDevice The file path of the I2C device (e.g., /dev/i2c-1).
     * @throw std::runtime_error If the device cannot be opened.
     */
    void init(const std::string& i2cDevice);

    /**
     * @brief Writes a single byte to a specific register on the I2C device.
     * @param deviceAddress The I2C device address.
     * @param reg The register address to write to.
     * @param value The value to write.
     * @throw std::runtime_error If the write operation fails.
     */
    void writeByte(uint8_t deviceAddress, uint8_t reg, uint8_t value);

    /**
     * @brief Writes a message to the I2C device.
     * @param deviceAddress The I2C device address.
     * @param buffer The buffer containing the message to write.
     * @throw std::runtime_error If the write operation fails.
     */
    void writeMessage(uint8_t deviceAddress, uint8_t *buffer);

    /**
     * @brief Reads a single byte from the I2C device.
     * @param deviceAddress The I2C device address.
     * @return The byte read from the device.
     * @throw std::runtime_error If the read operation fails.
     */
    uint8_t readByte(uint8_t deviceAddress);

    /**
     * @brief Reads data from a specific register on the I2C device.
     * @param deviceAddress The I2C device address.
     * @param registerAddr The register address to read from.
     * @param data Pointer to the buffer to store the read data.
     * @throw std::runtime_error If the read or write operation fails.
     */
    void readRegister(uint8_t deviceAddress, uint8_t registerAddr, uint8_t *data);
};
